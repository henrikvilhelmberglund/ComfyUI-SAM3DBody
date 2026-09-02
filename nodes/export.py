# Copyright (c) 2025 Andrea Pozzetti
# SPDX-License-Identifier: MIT
"""
Export nodes for SAM 3D Body meshes.

Exports meshes with rigging data to various formats using bpy in isolated venv.
"""

import logging
import os
import json
import time
import tempfile
import numpy as np
import torch
import folder_paths
import glob
from comfy_api.latest import io

log = logging.getLogger("sam3dbody")

def _align_torso_rolls_from_landmarks(bones_dict, rel_joints_corrected, joint_names_list, name_prefix=""):
    """Give the torso chain (root, spines, neck, head) a character-relative roll
    so their local Z-axis points along the character's forward direction.

    Without this, Blender's FBX importer picks an arbitrary roll and any
    full/twist-preserving pose transfer inherits that random twist,
    which cascades up the spine and messes with everything downstream.

    Runs while the armature is still in EDIT mode. `name_prefix` is used by
    the multi-person exporter which prefixes each bone with "P{idx}_".
    """
    from mathutils import Vector
    name_to_idx = {n: i for i, n in enumerate(joint_names_list)}
    try:
        l_upleg = Vector(rel_joints_corrected[name_to_idx['l_upleg']])
        r_upleg = Vector(rel_joints_corrected[name_to_idx['r_upleg']])
        root = Vector(rel_joints_corrected[name_to_idx['root']])
        c_head = Vector(rel_joints_corrected[name_to_idx['c_head']])
    except (KeyError, IndexError):
        return

    right = r_upleg - l_upleg
    up = c_head - root
    if right.length < 1e-4 or up.length < 1e-4:
        return
    right.normalize()
    up.normalize()
    forward = right.cross(up)
    if forward.length < 1e-4:
        return
    forward.normalize()

    torso = ('world', 'root', 'c_spine0', 'c_spine1', 'c_spine2', 'c_spine3', 'c_neck', 'c_head')
    for bn in torso:
        bone = bones_dict.get(name_prefix + bn)
        if bone is not None:
            bone.align_roll(forward)

    # Compute head-specific forward from eye anatomy so head-turn rotation is
    # preserved. Falls back to body forward if eye landmarks are missing.
    try:
        l_eye = Vector(rel_joints_corrected[name_to_idx['l_eye']])
        r_eye = Vector(rel_joints_corrected[name_to_idx['r_eye']])
        c_head_null = Vector(rel_joints_corrected[name_to_idx['c_head_null']])
        c_head_pos = Vector(rel_joints_corrected[name_to_idx['c_head']])
        head_right = r_eye - l_eye
        head_up = c_head_null - c_head_pos
        if head_right.length > 1e-4 and head_up.length > 1e-4:
            head_right.normalize(); head_up.normalize()
            head_forward = head_right.cross(head_up)
            if head_forward.length > 1e-4:
                head_forward.normalize()
                head_bone = bones_dict.get(name_prefix + 'c_head')
                if head_bone is not None:
                    head_bone.align_roll(head_forward)
    except (KeyError, IndexError):
        pass

    # Neck roll: blend body-forward with head-forward so partial neck turn
    # transfers when the head is looking somewhere different from the torso.
    if 'head_forward' in locals() and head_forward.length > 1e-4:
        neck_forward = (forward + head_forward) * 0.5
        if neck_forward.length > 1e-4:
            neck_forward.normalize()
            neck_bone = bones_dict.get(name_prefix + 'c_neck')
            if neck_bone is not None:
                neck_bone.align_roll(neck_forward)

    # Reorient world/root to short vertical bones so their axes exactly match
    # canonical (X=right, Y=up, Z=forward). Otherwise their Y follows their
    # first child (e.g. world -> root, root -> c_spine0) which tilts with any
    # spine curvature, and DELTA-mode retarget picks that up as tilt on the
    # target's master/pelvis gizmo.
    for _nm in ('world', 'root'):
        _bn = bones_dict.get(name_prefix + _nm)
        if _bn is not None:
            _h = Vector(_bn.head)
            _bn.tail = _h + up * 0.1
            _bn.align_roll(forward)

    # Realign spine bones to the OVERALL spine direction (root -> c_head)
    # rather than each pointing at its immediate child. Otherwise c_spine3
    # in particular points at c_neck which sits forward of the mid-chest
    # (natural cervical curve), so c_spine3 tilts forward regardless of the
    # actual spine posture.
    for _sname in ('c_spine0', 'c_spine1', 'c_spine2', 'c_spine3'):
        sb = bones_dict.get(name_prefix + _sname)
        if sb is not None:
            head_v = Vector(sb.head)
            length = (Vector(sb.tail) - head_v).length
            if length < 1e-4:
                length = 0.05
            sb.tail = head_v + up * length
            sb.align_roll(forward)


# Blender/MHR coord flip used by both bake helpers: (x, y, z) → (x, z, -y).
_BAKE_T = np.array([[1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0],
                    [0.0, -1.0, 0.0]], dtype=np.float32)


def _compute_yaw_only_transform(joint_coords):
    """Compute a rotation matrix (MHR space) that ONLY rotates around vertical
    to align character-forward to Blender -Y. Preserves any tilt/lean/pitch,
    so a kneeling character stays kneeling and only the horizontal facing
    direction changes. Returns None if landmarks are missing/degenerate."""
    try:
        from .sam_3d_body.mhr_joint_names import MHR_JOINT_NAMES as _MJN
        _idx = {n: i for i, n in enumerate(_MJN)}
        _to_bl = lambda p: (_BAKE_T @ p.astype(np.float32))

        # Character forward in Blender coords, from hip axis × spine, then
        # flatten to the horizontal plane (yaw only).
        right_bl = _to_bl(joint_coords[_idx['r_upleg']]
                          - joint_coords[_idx['l_upleg']])
        up_bl = _to_bl(joint_coords[_idx['c_head']]
                       - joint_coords[_idx['root']])
        forward_bl = np.cross(up_bl, right_bl)
        forward_xy = np.array([forward_bl[0], forward_bl[1], 0.0],
                              dtype=np.float32)
        n = float(np.linalg.norm(forward_xy))
        if n < 1e-4:
            return None
        forward_xy /= n

        # Signed angle from forward_xy to -Y around +Z.
        target = np.array([0.0, -1.0, 0.0], dtype=np.float32)
        cos_a = float(np.dot(forward_xy, target))
        sin_a = float(forward_xy[0] * target[1] - forward_xy[1] * target[0])
        angle = float(np.arctan2(sin_a, cos_a))

        c, s = np.cos(angle), np.sin(angle)
        R_bl = np.array([[c, -s, 0.0],
                         [s,  c, 0.0],
                         [0.0, 0.0, 1.0]], dtype=np.float32)
        return (_BAKE_T.T @ R_bl @ _BAKE_T).astype(np.float32)
    except (KeyError, IndexError):
        return None


def _compute_bake_facing_transform(joint_coords):
    """Compute (R_mhr, offset_mhr) that canonicalizes a character exported by
    the SAM3D pipeline: orient body (right, up, forward) to world (-X, +Z, -Y)
    (Rigify convention) and shift the world bone (joint 0) to armature origin
    with feet on the ground plane.

    Returns (R_mhr, offset_mhr) — both numpy 3x3 rotation and 3-vector in MHR
    space. Either can be None if the corresponding step is skipped due to
    missing / degenerate landmarks.
    """
    if joint_coords is None or len(joint_coords) != 127:
        return None, None
    try:
        from .sam_3d_body.mhr_joint_names import MHR_JOINT_NAMES as _MJN
        _idx = {n: i for i, n in enumerate(_MJN)}
        _to_bl = lambda p: (_BAKE_T @ p.astype(np.float32))

        _right_bl = _to_bl(joint_coords[_idx['r_upleg']]
                            - joint_coords[_idx['l_upleg']])
        _up_bl = _to_bl(joint_coords[_idx['c_head']]
                         - joint_coords[_idx['root']])
        _rn = float(np.linalg.norm(_right_bl))
        _un = float(np.linalg.norm(_up_bl))
        R_mhr = None
        if _rn > 1e-4 and _un > 1e-4:
            _right_bl /= _rn
            _up_bl /= _un

            _forward_bl = None
            try:
                _le = _to_bl(joint_coords[_idx['l_eye']])
                _re = _to_bl(joint_coords[_idx['r_eye']])
                _hd = _to_bl(joint_coords[_idx['c_head']])
                _f_eye = (_le + _re) * 0.5 - _hd
                _f_eye = _f_eye - _up_bl * float(np.dot(_f_eye, _up_bl))
                _fen = float(np.linalg.norm(_f_eye))
                if _fen > 1e-4:
                    _forward_bl = _f_eye / _fen
            except (KeyError, IndexError):
                pass
            if _forward_bl is None:
                _forward_bl = np.cross(_up_bl, _right_bl)
                _fn = float(np.linalg.norm(_forward_bl))
                if _fn > 1e-4:
                    _forward_bl /= _fn
                else:
                    _forward_bl = None

            if _forward_bl is not None:
                _right_bl = np.cross(_forward_bl, _up_bl)
                _rn2 = float(np.linalg.norm(_right_bl))
                if _rn2 > 1e-4:
                    _right_bl /= _rn2
                    _up_bl = np.cross(_right_bl, _forward_bl)
                    _un2 = float(np.linalg.norm(_up_bl))
                    if _un2 > 1e-4:
                        _up_bl /= _un2
                        _M_char = np.stack(
                            [_right_bl, _up_bl, _forward_bl], axis=1)
                        _M_world = np.array([[-1.0, 0.0,  0.0],
                                             [ 0.0, 0.0, -1.0],
                                             [ 0.0, 1.0,  0.0]],
                                            dtype=np.float32)
                        _R_bl = (_M_world @ _M_char.T).astype(np.float32)
                        R_mhr = (_BAKE_T.T @ _R_bl @ _BAKE_T).astype(np.float32)

        # Offset (ground-under-root) is computed AFTER rotation, so any caller
        # that wants the shared transform must also apply the rotation first.
        # We return R_mhr here; the offset is computed by the apply helper
        # (or by an explicit caller that has already rotated the joints).
        return R_mhr, None
    except (KeyError, ValueError, IndexError):
        return None, None


def _apply_bake_facing(joint_coords, vertices, R_mhr=None, offset_mhr=None,
                       compute_missing=True, mode='full'):
    """Apply an orientation + recenter bake to joint_coords + vertices.

    mode='full' (default) canonicalizes orientation (right→-X, up→+Z, forward→-Y)
    and recenters (world bone under root at ground level).
    mode='yaw' only rotates around vertical to align facing with -Y and
    recenters; any tilt/lean/pitch stays so poses (kneeling, crouching,
    leaning) aren't distorted.
    mode='off' is a no-op.

    If R_mhr / offset_mhr are None and compute_missing=True (the default),
    they are computed from THIS character's own landmarks (single-character
    behavior). Pass a non-None R_mhr and/or offset_mhr to share a transform
    across multiple characters (spatial-relationship preservation).

    Returns (new_joint_coords, new_vertices, R_mhr_used, offset_mhr_used).
    """
    if mode == 'off':
        return joint_coords, vertices, None, None
    if joint_coords is None or vertices is None or len(joint_coords) != 127:
        return joint_coords, vertices, R_mhr, offset_mhr

    joint_coords = joint_coords.astype(np.float32)
    vertices = vertices.astype(np.float32)

    if R_mhr is None and compute_missing:
        if mode == 'yaw':
            R_mhr = _compute_yaw_only_transform(joint_coords)
        else:
            R_mhr, _ = _compute_bake_facing_transform(joint_coords)
    if R_mhr is not None:
        joint_coords = joint_coords @ R_mhr.T
        vertices = vertices @ R_mhr.T

    if offset_mhr is None and compute_missing:
        try:
            from .sam_3d_body.mhr_joint_names import MHR_JOINT_NAMES as _MJN
            _idx = {n: i for i, n in enumerate(_MJN)}
            _joints_bl = joint_coords @ _BAKE_T.T
            _verts_bl = vertices @ _BAKE_T.T
            _root_bl = _joints_bl[_idx['root']]
            _min_z_bl = float(_verts_bl[:, 2].min())
            _target_bl = np.array(
                [_root_bl[0], _root_bl[1], _min_z_bl], dtype=np.float32)
            offset_mhr = _target_bl @ _BAKE_T
        except (KeyError, IndexError):
            offset_mhr = None
    if offset_mhr is not None:
        joint_coords = joint_coords - offset_mhr
        vertices = vertices - offset_mhr
        # Only pin joint 0 to origin when the offset was derived from THIS
        # character (single-character mode); for shared transforms across
        # multiple characters, joint 0 legitimately sits at some offset that
        # encodes the character's spatial position relative to the anchor.
        if compute_missing:
            joint_coords[0] = np.zeros(3, dtype=np.float32)

    return joint_coords, vertices, R_mhr, offset_mhr


def _compute_face_shape_key_data(mesh_data, R_mhr=None, offset_mhr=None):
    """Compute per-vertex data for the 72 MHR face-expression shape keys.

    Runs the MHR model 73 times (base with expr=0, then one basis per
    coefficient with expr[i]=1) and returns:
      - base_vertices: (V, 3) numpy in MHR space, transformed by (R_mhr, offset_mhr)
        the same way the main exported mesh is.
      - deltas: (72, V, 3) numpy — per-basis vertex offsets, rotated by R_mhr but
        NOT shifted (they are vector deltas, not absolute positions).
      - values: (72,) numpy — the CURRENT expression coefficients, so setting
        each shape key to these values reproduces the original posed+expressioned
        mesh (MHR expression is a linear blendshape basis).
      - names: list of "expr_00" .. "expr_71" so users can rename them later.

    Returns None on any failure (missing params, missing MHR path, etc.).
    """
    try:
        pose_params = mesh_data.get("pose_params")
        if not isinstance(pose_params, dict):
            return None
        shape_p = pose_params.get("shape")
        body_p = pose_params.get("body_pose")
        expr_p = pose_params.get("expr")
        if shape_p is None or body_p is None or expr_p is None:
            return None

        mhr_path = find_mhr_model_path(mesh_data)
        if not mhr_path or not os.path.exists(mhr_path):
            log.warning(" Shape-key bake requested but MHR model path not found.")
            return None

        def _to_tensor(x):
            if isinstance(x, torch.Tensor):
                return x.detach().float().cpu()
            return torch.from_numpy(np.asarray(x, dtype=np.float32))

        shape_t = _to_tensor(shape_p).reshape(-1)
        body_t = _to_tensor(body_p).reshape(-1)
        expr_t = _to_tensor(expr_p).reshape(-1)
        n_expr = int(expr_t.shape[0])

        mhr_model = torch.jit.load(mhr_path, map_location='cpu')
        mhr_model.eval()

        with torch.no_grad():
            # Batched forward: row 0 is base (expr=0), rows 1..n are basis
            # meshes (only i-th coefficient set to 1). Batching keeps the
            # forward count to one call instead of 73.
            n_batch = n_expr + 1
            shape_b = shape_t.unsqueeze(0).expand(n_batch, -1).contiguous()
            body_b = body_t.unsqueeze(0).expand(n_batch, -1).contiguous()
            expr_b = torch.zeros((n_batch, n_expr), dtype=torch.float32)
            for i in range(n_expr):
                expr_b[i + 1, i] = 1.0

            try:
                verts_all, _ = mhr_model(shape_b, body_b, expr_b)
            except Exception as _e:
                log.warning(f" Batched shape-key MHR forward failed ({_e}); falling back to per-basis.")
                # Fall back to one-at-a-time if the model can't handle the batch.
                verts_list = []
                for i in range(n_batch):
                    v, _ = mhr_model(shape_b[i:i+1], body_b[i:i+1], expr_b[i:i+1])
                    verts_list.append(v[0])
                verts_all = torch.stack(verts_list, dim=0)

        verts_all_np = verts_all.detach().cpu().numpy().astype(np.float32)
        if verts_all_np.ndim == 3 and verts_all_np.shape[0] == n_batch:
            base = verts_all_np[0]
            basis = verts_all_np[1:]
        else:
            log.warning(f" Unexpected MHR shape-key output shape: {verts_all_np.shape}")
            return None

        # Deltas are vector offsets (no translation), basis - base.
        deltas = basis - base[None, :, :]

        # Apply the SAME bake transform used for the main exported mesh so the
        # shape keys line up. Rotation applies to both base positions and
        # delta vectors; offset applies only to base positions.
        if R_mhr is not None:
            base = base @ R_mhr.T
            # (72, V, 3) @ (3, 3) — reshape via einsum for clarity.
            deltas = np.einsum('bvi,ji->bvj', deltas, R_mhr)
        if offset_mhr is not None:
            base = base - offset_mhr

        # Prefer the FACS-style semantic names from Meta's MHR docs
        # (browLowerer_L, jawDrop, etc.) so shape keys are immediately
        # editable — fall back to generic expr_NN if the coefficient count
        # doesn't match the 72-basis MHR v1.x layout.
        try:
            from .sam_3d_body.mhr_face_expression_names import MHR_FACE_EXPRESSION_NAMES
            if n_expr == len(MHR_FACE_EXPRESSION_NAMES):
                names = list(MHR_FACE_EXPRESSION_NAMES)
            else:
                names = [f"expr_{i:02d}" for i in range(n_expr)]
        except Exception:
            names = [f"expr_{i:02d}" for i in range(n_expr)]

        return {
            "base_vertices": base,
            "deltas": deltas,
            "values": expr_t.numpy().astype(np.float32),
            "names": names,
        }
    except Exception as _e:
        log.warning(f" Shape-key bake failed: {_e}")
        return None


def _apply_face_shape_keys_from_json(mesh_obj, shape_keys_json_path, name_prefix=""):
    """Add MHR face-expression shape keys to a mesh from a JSON file.

    The JSON stores base_vertices (V, 3), deltas (72, V, 3), values (72,), and
    names (list of 72 strings). Basis vertices are set to base_vertices (in
    Blender coords — Y and Z flipped, matching the OBJ we wrote), and each
    expression shape key is created as Basis + delta[i], then activated to the
    stored value so the mesh visually matches the original posed+expressioned
    result.

    Runs inside the bpy venv; the caller has already imported bpy.
    """
    with open(shape_keys_json_path, 'r') as f:
        sk_data = json.load(f)

    base = np.asarray(sk_data["base_vertices"], dtype=np.float32)
    deltas = np.asarray(sk_data["deltas"], dtype=np.float32)
    values = np.asarray(sk_data.get("values", []), dtype=np.float32)
    names = sk_data.get("names") or [f"expr_{i:02d}" for i in range(deltas.shape[0])]

    mesh = mesh_obj.data
    if len(mesh.vertices) != base.shape[0]:
        log.warning(
            f" Shape-key vertex count mismatch: mesh has {len(mesh.vertices)} "
            f"vertices, shape-key data has {base.shape[0]}. Skipping shape keys."
        )
        return

    # Match the OBJ flip we did in _write_obj_file: (x, -y, -z).
    base_bl = base.copy()
    base_bl[:, 1] = -base_bl[:, 1]
    base_bl[:, 2] = -base_bl[:, 2]
    deltas_bl = deltas.copy()
    deltas_bl[:, :, 1] = -deltas_bl[:, :, 1]
    deltas_bl[:, :, 2] = -deltas_bl[:, :, 2]

    # Add Basis shape key with the neutral (expr=0) mesh so shape keys are
    # deltas from a neutral face — otherwise expressions would layer on top
    # of the already-expressioned mesh from the OBJ import.
    basis_key = mesh_obj.shape_key_add(name="Basis", from_mix=False)
    for i in range(base_bl.shape[0]):
        basis_key.data[i].co = (float(base_bl[i, 0]), float(base_bl[i, 1]), float(base_bl[i, 2]))

    # Create expression shape keys: Basis + delta[i].
    for i in range(deltas_bl.shape[0]):
        key_name = f"{name_prefix}{names[i]}" if name_prefix else names[i]
        key = mesh_obj.shape_key_add(name=key_name, from_mix=False)
        key.slider_min = -2.0
        key.slider_max = 2.0
        for v_idx in range(base_bl.shape[0]):
            key.data[v_idx].co = (
                float(base_bl[v_idx, 0] + deltas_bl[i, v_idx, 0]),
                float(base_bl[v_idx, 1] + deltas_bl[i, v_idx, 1]),
                float(base_bl[v_idx, 2] + deltas_bl[i, v_idx, 2]),
            )
        # Activate to the coefficient the source image had, so the default
        # imported mesh looks like the original expression.
        if i < len(values):
            key.value = float(values[i])


class BpyFBXExporter:
    """Isolated bpy-based FBX exporter that runs in the sam3dbody venv."""

    FUNCTION = "export"

    def export(self, input_obj_path, output_fbx_path, skeleton_json_path=None, combined_json_path=None, shape_keys_json_path=None):
        """Export OBJ mesh to FBX using bpy.

        If combined_json_path is provided, exports multiple people into a single FBX.
        Otherwise, exports a single person.

        If shape_keys_json_path is provided, adds MHR face-expression shape keys
        to the mesh before export.
        """
        import bpy
        from mathutils import Vector
        import numpy as np
        import json
        import os

        # Handle combined export mode
        if combined_json_path and os.path.exists(combined_json_path):
            return self._export_combined(combined_json_path, output_fbx_path)

        # Single person export mode - load skeleton data from JSON if provided
        joints = None
        num_joints = 0
        joint_parents_list = None
        skinning_weights = None
        global_rotations = None

        if skeleton_json_path and os.path.exists(skeleton_json_path):
            with open(skeleton_json_path, 'r') as f:
                skeleton_data = json.load(f)

            joint_positions = skeleton_data.get('joint_positions', [])
            num_joints = skeleton_data.get('num_joints', len(joint_positions))
            joint_parents_list = skeleton_data.get('joint_parents')
            skinning_weights = skeleton_data.get('skinning_weights')
            global_rotations_data = skeleton_data.get('global_rotations')
            joint_names_list = skeleton_data.get('joint_names')

            if joint_positions:
                joints = np.array(joint_positions, dtype=np.float32)

            if global_rotations_data:
                global_rotations = np.array(global_rotations_data, dtype=np.float32)
                log.info(f" Loaded global_rotations: shape {global_rotations.shape}")

        # Clean default scene
        for c in bpy.data.actions:
            bpy.data.actions.remove(c)
        for c in bpy.data.armatures:
            bpy.data.armatures.remove(c)
        for c in bpy.data.cameras:
            bpy.data.cameras.remove(c)
        for c in bpy.data.collections:
            bpy.data.collections.remove(c)
        for c in bpy.data.images:
            bpy.data.images.remove(c)
        for c in bpy.data.materials:
            bpy.data.materials.remove(c)
        for c in bpy.data.meshes:
            bpy.data.meshes.remove(c)
        for c in bpy.data.objects:
            bpy.data.objects.remove(c)
        for c in bpy.data.textures:
            bpy.data.textures.remove(c)

        # Create collection
        collection = bpy.data.collections.new('SAM3D_Export')
        bpy.context.scene.collection.children.link(collection)

        # Import OBJ mesh
        bpy.ops.wm.obj_import(filepath=input_obj_path)

        imported_objects = [obj for obj in bpy.context.scene.objects if obj.type == 'MESH']
        if not imported_objects:
            raise RuntimeError("No mesh found after OBJ import")

        mesh_obj = imported_objects[0]
        mesh_obj.name = 'SAM3D_Character'

        # Move to our collection
        if mesh_obj.name in bpy.context.scene.collection.objects:
            bpy.context.scene.collection.objects.unlink(mesh_obj)
        collection.objects.link(mesh_obj)

        # Create armature from skeleton if provided
        if joints is not None and num_joints > 0:
            # Create armature in edit mode
            bpy.ops.object.armature_add(enter_editmode=True)
            armature = bpy.data.armatures.get('Armature')
            armature.name = 'SAM3D_Skeleton'
            armature_obj = bpy.context.active_object
            armature_obj.name = 'SAM3D_Skeleton'

            # Move to our collection
            if armature_obj.name in bpy.context.scene.collection.objects:
                bpy.context.scene.collection.objects.unlink(armature_obj)
            collection.objects.link(armature_obj)

            edit_bones = armature.edit_bones
            extrude_size = 0.05

            # Remove default bone
            default_bone = edit_bones.get('Bone')
            if default_bone:
                edit_bones.remove(default_bone)

            # Center the skeleton on joint 0 (MHR "world" bone) rather than
            # the mean, so world_bone ends up at armature-local origin (0,0,0).
            # This means scaling the source armature via src.scale doesn't
            # move the world bone — critical for keeping the master control
            # in place while nudging character proportions.
            skeleton_center = joints[0].copy() if len(joints) > 0 else joints.mean(axis=0)

            # Make positions relative to skeleton center
            rel_joints = joints - skeleton_center

            # Apply coordinate system correction to match mesh orientation
            rel_joints_corrected = np.zeros_like(rel_joints)
            rel_joints_corrected[:, 0] = rel_joints[:, 0]
            rel_joints_corrected[:, 1] = -rel_joints[:, 2]
            rel_joints_corrected[:, 2] = rel_joints[:, 1]

            use_named = joint_names_list and len(joint_names_list) == num_joints
            def _bone_name(i):
                return joint_names_list[i] if use_named else f'Joint_{i:03d}'

            # Build child map so bones can be oriented head->child instead of head->up
            children_of = [[] for _ in range(num_joints)]
            if joint_parents_list and len(joint_parents_list) == num_joints:
                for c, p in enumerate(joint_parents_list):
                    if 0 <= p < num_joints and p != c:
                        children_of[p].append(c)

            # Anatomical overrides: pick a specific child for bones where "first by index"
            # gives a bad direction (e.g., pelvis -> spine, not -> leg).
            _tail_overrides_by_idx = {}
            if use_named:
                from .sam_3d_body.mhr_joint_names import MHR_TAIL_CHILD_OVERRIDES
                name_to_idx = {n: i for i, n in enumerate(joint_names_list)}
                for parent_name, child_name in MHR_TAIL_CHILD_OVERRIDES.items():
                    pi, ci = name_to_idx.get(parent_name), name_to_idx.get(child_name)
                    if pi is not None and ci is not None and ci in children_of[pi]:
                        _tail_overrides_by_idx[pi] = ci

            def _tail_for(i):
                head = rel_joints_corrected[i]
                kids = children_of[i]
                if kids:
                    chosen = _tail_overrides_by_idx.get(i, kids[0])
                    tail = rel_joints_corrected[chosen]
                    if float(np.linalg.norm(tail - head)) < 1e-4:
                        return head + np.array([0.0, 0.0, extrude_size], dtype=np.float32)
                    return tail
                p = joint_parents_list[i] if joint_parents_list else -1
                if 0 <= p < num_joints and p != i:
                    d = head - rel_joints_corrected[p]
                    n = float(np.linalg.norm(d))
                    if n > 1e-4:
                        return head + d * min(0.25, extrude_size / n)
                return head + np.array([0.0, 0.0, extrude_size], dtype=np.float32)

            # Create all bones
            bones_dict = {}
            for i in range(num_joints):
                bone_name = _bone_name(i)
                bone = edit_bones.new(bone_name)
                bone.head = Vector((float(rel_joints_corrected[i, 0]), float(rel_joints_corrected[i, 1]), float(rel_joints_corrected[i, 2])))
                tail_pos = _tail_for(i)
                bone.tail = Vector((float(tail_pos[0]), float(tail_pos[1]), float(tail_pos[2])))
                bones_dict[bone_name] = bone

            # Build hierarchical structure using joint parents if available
            if joint_parents_list and len(joint_parents_list) == num_joints:
                for i in range(num_joints):
                    parent_idx = joint_parents_list[i]
                    if parent_idx >= 0 and parent_idx < num_joints and parent_idx != i:
                        bones_dict[_bone_name(i)].parent = bones_dict[_bone_name(parent_idx)]
                        bones_dict[_bone_name(i)].use_connect = False
            else:
                # Fallback: create flat hierarchy with the first bone as root
                root_bone_name = _bone_name(0)
                for i in range(1, num_joints):
                    bones_dict[_bone_name(i)].parent = bones_dict[root_bone_name]
                    bones_dict[_bone_name(i)].use_connect = False

            # Give torso bones a character-relative roll so their local Z-axis points
            # forward. Without this, Blender's FBX importer assigns arbitrary rolls
            # and any full/roll-preserving retarget picks up random twist.
            if use_named:
                _align_torso_rolls_from_landmarks(bones_dict, rel_joints_corrected, joint_names_list)

            # Switch to object mode
            bpy.ops.object.mode_set(mode='OBJECT')

            # Position armature at skeleton center
            skeleton_center_corrected = np.zeros(3)
            skeleton_center_corrected[0] = skeleton_center[0]
            skeleton_center_corrected[1] = -skeleton_center[2]
            skeleton_center_corrected[2] = skeleton_center[1]
            armature_obj.location = Vector((skeleton_center_corrected[0], skeleton_center_corrected[1], skeleton_center_corrected[2]))

            # Apply skinning weights if available
            if skinning_weights:
                # Create vertex groups for each bone
                for i in range(num_joints):
                    mesh_obj.vertex_groups.new(name=_bone_name(i))

                # Assign weights to vertices
                num_vertices = len(mesh_obj.data.vertices)
                for vert_idx in range(min(num_vertices, len(skinning_weights))):
                    influences = skinning_weights[vert_idx]
                    if influences and len(influences) > 0:
                        for bone_idx, weight in influences:
                            if 0 <= bone_idx < num_joints and weight > 0.0001:
                                vertex_group = mesh_obj.vertex_groups.get(_bone_name(bone_idx))
                                if vertex_group:
                                    vertex_group.add([vert_idx], weight, 'REPLACE')

            # Deselect all
            for obj in bpy.context.selected_objects:
                obj.select_set(False)

            # Parent mesh to armature
            mesh_obj.select_set(True)
            armature_obj.select_set(True)
            bpy.context.view_layer.objects.active = armature_obj

            if skinning_weights:
                bpy.ops.object.parent_set(type='ARMATURE')
            else:
                bpy.ops.object.parent_set(type='ARMATURE_NAME')

        # Add MHR face-expression shape keys (if requested). This runs BEFORE
        # the double-sided duplication so that when the duplicated vertices
        # inherit the Basis shape key positions, the expressions still deform
        # the back-face copies correctly.
        if shape_keys_json_path and os.path.exists(shape_keys_json_path):
            _apply_face_shape_keys_from_json(mesh_obj, shape_keys_json_path)

        # Make mesh double-sided AFTER skinning (so duplicated vertices inherit weights)
        bpy.context.view_layer.objects.active = mesh_obj
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.select_all(action='SELECT')
        bpy.ops.mesh.duplicate()
        bpy.ops.mesh.flip_normals()
        bpy.ops.object.mode_set(mode='OBJECT')

        # Export to FBX
        os.makedirs(os.path.dirname(output_fbx_path) if os.path.dirname(output_fbx_path) else '.', exist_ok=True)

        # Select all objects in our collection
        for obj in bpy.context.selected_objects:
            obj.select_set(False)
        for obj in collection.objects:
            obj.select_set(True)

        # Export FBX
        bpy.ops.export_scene.fbx(
            filepath=output_fbx_path,
            check_existing=False,
            use_selection=True,
            add_leaf_bones=False,
            path_mode='COPY',
            embed_textures=True,
        )

        return {"success": True, "output_path": output_fbx_path}

    def _export_combined(self, combined_json_path, output_fbx_path):
        """
        Export multiple people into a single combined FBX file.

        Args:
            combined_json_path: Path to JSON file containing list of people data
            output_fbx_path: Output path for the combined FBX file
        """
        import bpy
        from mathutils import Vector
        import numpy as np
        import json
        import os

        # Load the combined data from JSON
        with open(combined_json_path, 'r') as f:
            people_data = json.load(f)

        # Clean default scene
        for c in bpy.data.actions:
            bpy.data.actions.remove(c)
        for c in bpy.data.armatures:
            bpy.data.armatures.remove(c)
        for c in bpy.data.cameras:
            bpy.data.cameras.remove(c)
        for c in bpy.data.collections:
            bpy.data.collections.remove(c)
        for c in bpy.data.images:
            bpy.data.images.remove(c)
        for c in bpy.data.materials:
            bpy.data.materials.remove(c)
        for c in bpy.data.meshes:
            bpy.data.meshes.remove(c)
        for c in bpy.data.objects:
            bpy.data.objects.remove(c)
        for c in bpy.data.textures:
            bpy.data.textures.remove(c)

        # Create collection for all exported objects
        collection = bpy.data.collections.new('SAM3D_Export')
        bpy.context.scene.collection.children.link(collection)

        # Process each person
        for person in people_data:
            obj_path = person["obj_path"]
            skeleton_json_path = person.get("skeleton_json_path")
            idx = person["index"]

            # Load skeleton data from JSON if provided
            joints = None
            num_joints = 0
            joint_parents_list = None
            skinning_weights = None
            global_rotations = None

            if skeleton_json_path and os.path.exists(skeleton_json_path):
                with open(skeleton_json_path, 'r') as f:
                    skeleton_data = json.load(f)

                joint_positions = skeleton_data.get('joint_positions', [])
                num_joints = skeleton_data.get('num_joints', len(joint_positions))
                joint_parents_list = skeleton_data.get('joint_parents')
                skinning_weights = skeleton_data.get('skinning_weights')
                global_rotations_data = skeleton_data.get('global_rotations')
                joint_names_list = skeleton_data.get('joint_names')

                if joint_positions:
                    joints = np.array(joint_positions, dtype=np.float32)

                if global_rotations_data:
                    global_rotations = np.array(global_rotations_data, dtype=np.float32)
                    log.info(f" Person {idx}: Loaded global_rotations shape {global_rotations.shape}")

            # Import OBJ mesh
            bpy.ops.wm.obj_import(filepath=obj_path)

            imported_objects = [obj for obj in bpy.context.selected_objects if obj.type == 'MESH']
            if not imported_objects:
                continue

            mesh_obj = imported_objects[0]
            mesh_obj.name = f'SAM3D_Character_{idx}'

            # Move to our collection
            if mesh_obj.name in bpy.context.scene.collection.objects:
                bpy.context.scene.collection.objects.unlink(mesh_obj)
            collection.objects.link(mesh_obj)

            # Create armature from skeleton if provided
            if joints is not None and num_joints > 0:
                # Create armature in edit mode
                bpy.ops.object.armature_add(enter_editmode=True)
                armature = bpy.context.active_object.data
                armature.name = f'SAM3D_Skeleton_{idx}'
                armature_obj = bpy.context.active_object
                armature_obj.name = f'SAM3D_Skeleton_{idx}'

                # Move to our collection
                if armature_obj.name in bpy.context.scene.collection.objects:
                    bpy.context.scene.collection.objects.unlink(armature_obj)
                collection.objects.link(armature_obj)

                edit_bones = armature.edit_bones
                extrude_size = 0.05

                # Remove default bone
                default_bone = edit_bones.get('Bone')
                if default_bone:
                    edit_bones.remove(default_bone)

                # Center on joint 0 (world) so scaling doesn't move it.
                skeleton_center = joints[0].copy() if len(joints) > 0 else joints.mean(axis=0)

                # Make positions relative to skeleton center
                rel_joints = joints - skeleton_center

                # Apply coordinate system correction to match mesh orientation
                rel_joints_corrected = np.zeros_like(rel_joints)
                rel_joints_corrected[:, 0] = rel_joints[:, 0]
                rel_joints_corrected[:, 1] = -rel_joints[:, 2]
                rel_joints_corrected[:, 2] = rel_joints[:, 1]

                # Create all bones with per-person prefix so multi-armature scenes stay unique
                use_named = joint_names_list and len(joint_names_list) == num_joints
                def _bone_name(i):
                    base = joint_names_list[i] if use_named else f'Joint_{i:03d}'
                    return f'P{idx}_{base}'

                children_of = [[] for _ in range(num_joints)]
                if joint_parents_list and len(joint_parents_list) == num_joints:
                    for c, p in enumerate(joint_parents_list):
                        if 0 <= p < num_joints and p != c:
                            children_of[p].append(c)

                _tail_overrides_by_idx = {}
                if use_named:
                    from .sam_3d_body.mhr_joint_names import MHR_TAIL_CHILD_OVERRIDES
                    name_to_idx = {n: i for i, n in enumerate(joint_names_list)}
                    for parent_name, child_name in MHR_TAIL_CHILD_OVERRIDES.items():
                        pi, ci = name_to_idx.get(parent_name), name_to_idx.get(child_name)
                        if pi is not None and ci is not None and ci in children_of[pi]:
                            _tail_overrides_by_idx[pi] = ci

                def _tail_for(i):
                    head = rel_joints_corrected[i]
                    kids = children_of[i]
                    if kids:
                        chosen = _tail_overrides_by_idx.get(i, kids[0])
                        tail = rel_joints_corrected[chosen]
                        if float(np.linalg.norm(tail - head)) < 1e-4:
                            return head + np.array([0.0, 0.0, extrude_size], dtype=np.float32)
                        return tail
                    p = joint_parents_list[i] if joint_parents_list else -1
                    if 0 <= p < num_joints and p != i:
                        d = head - rel_joints_corrected[p]
                        n = float(np.linalg.norm(d))
                        if n > 1e-4:
                            return head + d * min(0.25, extrude_size / n)
                    return head + np.array([0.0, 0.0, extrude_size], dtype=np.float32)

                bones_dict = {}
                for i in range(num_joints):
                    bone_name = _bone_name(i)
                    bone = edit_bones.new(bone_name)
                    bone.head = Vector((float(rel_joints_corrected[i, 0]), float(rel_joints_corrected[i, 1]), float(rel_joints_corrected[i, 2])))
                    tail_pos = _tail_for(i)
                    bone.tail = Vector((float(tail_pos[0]), float(tail_pos[1]), float(tail_pos[2])))
                    bones_dict[bone_name] = bone

                # Build hierarchical structure using joint parents if available
                if joint_parents_list and len(joint_parents_list) == num_joints:
                    for i in range(num_joints):
                        parent_idx = joint_parents_list[i]
                        if parent_idx >= 0 and parent_idx < num_joints and parent_idx != i:
                            bones_dict[_bone_name(i)].parent = bones_dict[_bone_name(parent_idx)]
                            bones_dict[_bone_name(i)].use_connect = False
                else:
                    # Fallback: create flat hierarchy with the first bone as root
                    root_bone_name = _bone_name(0)
                    for i in range(1, num_joints):
                        bones_dict[_bone_name(i)].parent = bones_dict[root_bone_name]
                        bones_dict[_bone_name(i)].use_connect = False

                if use_named:
                    _align_torso_rolls_from_landmarks(
                        bones_dict, rel_joints_corrected, joint_names_list,
                        name_prefix=f'P{idx}_',
                    )

                # Switch to object mode
                bpy.ops.object.mode_set(mode='OBJECT')

                # Position armature at skeleton center
                skeleton_center_corrected = np.zeros(3)
                skeleton_center_corrected[0] = skeleton_center[0]
                skeleton_center_corrected[1] = -skeleton_center[2]
                skeleton_center_corrected[2] = skeleton_center[1]
                armature_obj.location = Vector((skeleton_center_corrected[0], skeleton_center_corrected[1], skeleton_center_corrected[2]))

                # Apply skinning weights if available
                if skinning_weights:
                    # Create vertex groups for each bone
                    for i in range(num_joints):
                        mesh_obj.vertex_groups.new(name=_bone_name(i))

                    # Assign weights to vertices
                    num_vertices = len(mesh_obj.data.vertices)
                    for vert_idx in range(min(num_vertices, len(skinning_weights))):
                        influences = skinning_weights[vert_idx]
                        if influences and len(influences) > 0:
                            for bone_idx, weight in influences:
                                if 0 <= bone_idx < num_joints and weight > 0.0001:
                                    vertex_group = mesh_obj.vertex_groups.get(_bone_name(bone_idx))
                                    if vertex_group:
                                        vertex_group.add([vert_idx], weight, 'REPLACE')

                # Deselect all
                for obj in bpy.context.selected_objects:
                    obj.select_set(False)

                # Parent mesh to armature
                mesh_obj.select_set(True)
                armature_obj.select_set(True)
                bpy.context.view_layer.objects.active = armature_obj

                if skinning_weights:
                    bpy.ops.object.parent_set(type='ARMATURE')
                else:
                    bpy.ops.object.parent_set(type='ARMATURE_NAME')

            # Make mesh double-sided AFTER skinning (so duplicated vertices inherit weights)
            bpy.context.view_layer.objects.active = mesh_obj
            bpy.ops.object.mode_set(mode='EDIT')
            bpy.ops.mesh.select_all(action='SELECT')
            bpy.ops.mesh.duplicate()
            bpy.ops.mesh.flip_normals()
            bpy.ops.object.mode_set(mode='OBJECT')

        # Export to FBX
        os.makedirs(os.path.dirname(output_fbx_path) if os.path.dirname(output_fbx_path) else '.', exist_ok=True)

        # Select all objects in our collection
        for obj in bpy.context.selected_objects:
            obj.select_set(False)
        for obj in collection.objects:
            obj.select_set(True)

        # Export FBX with all objects
        bpy.ops.export_scene.fbx(
            filepath=output_fbx_path,
            check_existing=False,
            use_selection=True,
            add_leaf_bones=False,
            path_mode='COPY',
            embed_textures=True,
        )

        return {"success": True, "output_path": output_fbx_path}

class BpyPoseApplier:
    """Isolated bpy-based pose applier that runs in the sam3dbody venv."""

    FUNCTION = "apply_pose"

    def apply_pose(self, input_fbx_path, output_fbx_path, transforms_json_path):
        """
        Load an FBX, apply bone transforms, and export to new FBX.

        Args:
            input_fbx_path: Path to input FBX file
            output_fbx_path: Path to output FBX file
            transforms_json_path: Path to JSON file containing bone transforms
        """
        import bpy
        import mathutils
        import json
        import os

        log.info(f" Loading FBX: {input_fbx_path}")

        # Clear the scene
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete()

        # Import the FBX
        bpy.ops.import_scene.fbx(filepath=input_fbx_path)

        # Load bone transforms from JSON
        with open(transforms_json_path, 'r') as f:
            bone_transforms = json.load(f)

        log.info(f" Loaded {len(bone_transforms)} bone transforms")

        # Find the armature
        armature = None
        for obj in bpy.data.objects:
            if obj.type == 'ARMATURE':
                armature = obj
                break

        if not armature:
            return {"success": False, "error": "No armature found in FBX"}

        log.info(f" Found armature: {armature.name}")
        log.info(f" Armature has {len(armature.pose.bones)} pose bones")

        # Apply transforms to pose bones
        applied_count = 0
        for bone_name, transform in bone_transforms.items():
            if bone_name not in armature.pose.bones:
                log.info(f" WARNING: Bone '{bone_name}' not found in armature")
                continue

            pose_bone = armature.pose.bones[bone_name]

            # Apply position delta (offset from rest pose)
            pos_delta = transform.get('position', {})
            if pos_delta:
                pose_bone.location.x += pos_delta.get('x', 0)
                pose_bone.location.y += pos_delta.get('y', 0)
                pose_bone.location.z += pos_delta.get('z', 0)

            # Apply rotation delta (quaternion multiply)
            quat_delta = transform.get('quaternion', {})
            if quat_delta:
                delta_quat = mathutils.Quaternion((
                    quat_delta.get('w', 1.0),
                    quat_delta.get('x', 0.0),
                    quat_delta.get('y', 0.0),
                    quat_delta.get('z', 0.0)
                ))
                # Multiply current rotation by delta
                pose_bone.rotation_quaternion = pose_bone.rotation_quaternion @ delta_quat

            # Apply scale delta (multiply)
            scale_delta = transform.get('scale', {})
            if scale_delta:
                pose_bone.scale.x *= scale_delta.get('x', 1.0)
                pose_bone.scale.y *= scale_delta.get('y', 1.0)
                pose_bone.scale.z *= scale_delta.get('z', 1.0)

            applied_count += 1

        log.info(f" Applied transforms to {applied_count} bones")

        # Update the scene to apply transforms
        bpy.context.view_layer.update()

        # Export to FBX with current pose
        os.makedirs(os.path.dirname(output_fbx_path) if os.path.dirname(output_fbx_path) else '.', exist_ok=True)

        log.info(f" Exporting posed FBX: {output_fbx_path}")
        bpy.ops.export_scene.fbx(
            filepath=output_fbx_path,
            use_selection=False,
            apply_scale_options='FBX_SCALE_ALL',
            bake_anim=False,
            add_leaf_bones=False,
        )

        log.info(f" Export complete")
        return {"success": True, "output_path": output_fbx_path}


def find_mhr_model_path(mesh_data=None):
    """
    Find the MHR model path using multiple fallback strategies.

    Args:
        mesh_data: Optional mesh_data dict that may contain mhr_path

    Returns:
        str: Path to mhr_model.pt or None if not found
    """
    # Strategy 1: Check mesh_data for explicitly provided path
    if mesh_data and mesh_data.get("mhr_path"):
        mhr_path = mesh_data["mhr_path"]
        if os.path.exists(mhr_path):
            return mhr_path

    # Strategy 2: Check environment variable
    env_path = os.environ.get("SAM3D_MHR_PATH", "")
    if env_path and os.path.exists(env_path):
        return env_path

    # Strategy 3: Search ComfyUI models/sam3dbody/ folder
    sam3dbody_dir = os.path.join(folder_paths.models_dir, "sam3dbody", "assets", "mhr_model.pt")
    if os.path.exists(sam3dbody_dir):
        return sam3dbody_dir

    # Strategy 4 (legacy): Search HuggingFace cache for backwards compatibility
    hf_cache_base = os.path.expanduser("~/.cache/huggingface/hub/models--facebook--sam-3d-body-dinov3")
    if os.path.exists(hf_cache_base):
        pattern = os.path.join(hf_cache_base, "snapshots", "*", "assets", "mhr_model.pt")
        matches = glob.glob(pattern)
        if matches:
            matches.sort(key=os.path.getmtime, reverse=True)
            return matches[0]

    return None


class SAM3DBodyExportFBX(io.ComfyNode):
    """
    Export SAM3D Body mesh with skeleton to FBX format.

    Takes mesh data from SAM3D and exports it as a rigged FBX file
    using Blender for format conversion.
    """

    @classmethod
    def define_schema(cls):
        return io.Schema(
            node_id="SAM3DBodyExportFBX",
            display_name="SAM 3D Body: Export FBX",
            category="SAM3DBody/export",
            is_output_node=True,
            inputs=[
                io.Custom("SAM3D_OUTPUT").Input("mesh_data",
                    tooltip="Mesh data from SAM3D Body Process node"),
                io.String.Input("output_filename", default="sam3d_rigged.fbx",
                    tooltip="Output filename for the FBX file"),
                io.Boolean.Input("overwrite", default=False,
                    tooltip="When enabled, always write to the exact filename above (overwriting). "
                            "When disabled, appends an incrementing counter so each run creates a new file."),
                io.Combo.Input("bake_facing",
                    options=["off", "yaw", "full"], default="off",
                    tooltip="off: no reorientation.  yaw: rotate around vertical only so the "
                            "character faces world -Y (Rigify convention); any tilt/lean/pitch of "
                            "the pose is preserved, so kneeling / crouching / leaning poses look "
                            "the same.  full: also make the character perfectly upright (may "
                            "reinterpret leaned poses — e.g. a leaning-forward kneel becomes an "
                            "upright squat).  Both baked modes also recenter the world bone under "
                            "the character's feet."),
                io.Boolean.Input("bake_face_shape_keys", default=False,
                    tooltip="When enabled, adds 72 MHR face-expression shape keys to the mesh so "
                            "you can edit each blendshape (jaw, brow, lip corners, cheeks, etc.) "
                            "independently in Blender. Shape keys are initialized to the "
                            "coefficients detected in the source image, so the default view "
                            "matches the original expression. Adds ~1 MHR forward pass with 73 "
                            "batch rows, so exports are noticeably slower."),
            ],
            outputs=[
                io.String.Output(display_name="fbx_path"),
            ],
        )

    @classmethod
    def execute(cls, mesh_data, output_filename, overwrite=False, bake_facing="off", bake_face_shape_keys=False):
        """Export mesh with skeleton to FBX format."""

        # Extract mesh data
        vertices = mesh_data.get("vertices")
        faces = mesh_data.get("faces")
        joint_coords = mesh_data.get("joint_coords")  # 127 joints

        if vertices is None or faces is None:
            raise RuntimeError("Mesh vertices or faces not found in mesh_data")

        # Convert tensors to numpy if needed
        if isinstance(vertices, torch.Tensor):
            vertices = vertices.cpu().numpy()
        if isinstance(faces, torch.Tensor):
            faces = faces.cpu().numpy()
        if joint_coords is not None and isinstance(joint_coords, torch.Tensor):
            joint_coords = joint_coords.cpu().numpy()

        # Prepare output path: either overwrite the fixed filename, or use
        # Comfy's incrementing counter so each run creates a fresh file.
        if not output_filename.endswith('.fbx'):
            output_filename = output_filename + '.fbx'
        if overwrite:
            output_fbx_path = os.path.join(folder_paths.get_output_directory(), output_filename)
        else:
            filename_prefix = output_filename[:-4]
            full_output_folder, filename_base, counter, _subfolder, _filename_prefix = \
                folder_paths.get_save_image_path(filename_prefix, folder_paths.get_output_directory())
            output_filename = f"{filename_base}_{counter:05}_.fbx"
            output_fbx_path = os.path.join(full_output_folder, output_filename)

        # Optional orientation bake — logic is in _apply_bake_facing so it can
        # be shared with the two-character export node (which needs to apply
        # ONE transform to both characters to preserve their spatial relation).
        # We hold onto (R_used, offset_used) so shape-key baking can apply the
        # same transform to its base/basis meshes.
        R_used, offset_used = None, None
        if bake_facing and bake_facing != "off":
            joint_coords, vertices, R_used, offset_used = _apply_bake_facing(
                joint_coords, vertices, mode=bake_facing)

        # Create a simple OBJ file first (Blender can import this easily)
        temp_dir = folder_paths.get_temp_directory()
        temp_obj_path = os.path.join(temp_dir, f"temp_mesh_{int(time.time())}.obj")

        # Write OBJ file
        cls._write_obj_file(temp_obj_path, vertices, faces)

        # Save skeleton data if available
        skeleton_json_path = None
        if joint_coords is not None:
            skeleton_json_path = os.path.join(temp_dir, f"skeleton_{int(time.time())}.json")

            # Convert mesh bounds to plain Python types (with coordinate transform applied)
            mesh_min = vertices.min(axis=0)
            mesh_max = vertices.max(axis=0)
            if isinstance(mesh_min, np.ndarray):
                mesh_min = [float(x) for x in mesh_min]
            if isinstance(mesh_max, np.ndarray):
                mesh_max = [float(x) for x in mesh_max]
            # Apply same transform as mesh: flip Y and Z axes
            mesh_min = [mesh_min[0], -mesh_min[1], -mesh_min[2]]
            mesh_max = [mesh_max[0], -mesh_max[1], -mesh_max[2]]
            # Ensure min < max after flipping (signs reverse order)
            mesh_min, mesh_max = [min(mesh_min[i], mesh_max[i]) for i in range(3)], [max(mesh_min[i], mesh_max[i]) for i in range(3)]

            # Apply coordinate transform to joint positions to match mesh (flip Y and Z)
            joint_coords_flipped = joint_coords.copy()
            joint_coords_flipped[:, 1] = -joint_coords_flipped[:, 1]
            joint_coords_flipped[:, 2] = -joint_coords_flipped[:, 2]

            skeleton_data = {
                "joint_positions": joint_coords_flipped.tolist(),
                "num_joints": len(joint_coords),
                "mesh_vertices_bounds_min": mesh_min,
                "mesh_vertices_bounds_max": mesh_max,
            }

            if len(joint_coords) == 127:
                from .sam_3d_body.mhr_joint_names import MHR_JOINT_NAMES
                skeleton_data["joint_names"] = MHR_JOINT_NAMES

            # Extract skinning weights from MHR model
            try:
                mhr_model_path = find_mhr_model_path(mesh_data)

                if mhr_model_path and os.path.exists(mhr_model_path):
                    mhr_model = torch.jit.load(mhr_model_path, map_location='cpu')
                    lbs = mhr_model.character_torch.linear_blend_skinning

                    vert_indices = lbs.vert_indices_flattened.cpu().numpy().astype(int)
                    skin_indices = lbs.skin_indices_flattened.cpu().numpy().astype(int)
                    skin_weights = lbs.skin_weights_flattened.cpu().numpy().astype(float)

                    vertex_weights = {}
                    for i in range(len(vert_indices)):
                        vert_idx = int(vert_indices[i])
                        bone_idx = int(skin_indices[i])
                        weight = float(skin_weights[i])

                        if vert_idx not in vertex_weights:
                            vertex_weights[vert_idx] = []
                        vertex_weights[vert_idx].append([bone_idx, weight])

                    skinning_data = []
                    num_vertices = len(vertices)
                    for vert_idx in range(num_vertices):
                        if vert_idx in vertex_weights:
                            skinning_data.append(vertex_weights[vert_idx])
                        else:
                            skinning_data.append([])

                    skeleton_data["skinning_weights"] = skinning_data
            except Exception:
                pass  # Skip skinning weights if extraction fails

            # Get joint parent hierarchy from mesh_data
            joint_parents = None
            joint_rotations = mesh_data.get("joint_rotations")

            if isinstance(joint_rotations, dict) and "joint_parents" in joint_rotations:
                joint_parents_data = joint_rotations["joint_parents"]
            else:
                joint_parents_data = mesh_data.get("joint_parents")

            if joint_parents_data is not None:
                if isinstance(joint_parents_data, np.ndarray):
                    joint_parents = joint_parents_data.astype(int).tolist()
                elif isinstance(joint_parents_data, torch.Tensor):
                    joint_parents = joint_parents_data.cpu().numpy().astype(int).tolist()
                else:
                    joint_parents = [int(p) for p in joint_parents_data]
                skeleton_data["joint_parents"] = joint_parents
            else:
                # Load joint parents from MHR model if we have 127 joints
                if len(joint_coords) == 127:
                    try:
                        mhr_model_path = find_mhr_model_path(mesh_data)
                        if mhr_model_path and os.path.exists(mhr_model_path):
                            mhr_model = torch.jit.load(mhr_model_path, map_location='cpu')
                            joint_parents_tensor = mhr_model.character_torch.skeleton.joint_parents
                            joint_parents = joint_parents_tensor.cpu().numpy().astype(int).tolist()
                            skeleton_data["joint_parents"] = joint_parents
                    except Exception:
                        pass

            # Add camera and focal length if available
            camera = mesh_data.get("camera")
            focal_length = mesh_data.get("focal_length")
            if camera is not None:
                if isinstance(camera, torch.Tensor):
                    camera = camera.cpu().numpy()
                skeleton_data["camera"] = [float(x) for x in camera.flatten()] if isinstance(camera, np.ndarray) else camera
            if focal_length is not None:
                if isinstance(focal_length, (torch.Tensor, np.ndarray)):
                    focal_length = float(focal_length.item() if hasattr(focal_length, 'item') else focal_length)
                skeleton_data["focal_length"] = float(focal_length)

            with open(skeleton_json_path, 'w') as f:
                json.dump(skeleton_data, f)

        # Optional face-expression shape keys: run MHR 73 times (base + 72
        # basis) with the SAME shape+pose but varying expr, then write to a
        # temp JSON that the bpy exporter turns into Blender shape keys.
        shape_keys_json_path = None
        if bake_face_shape_keys:
            # If bake_facing is "off" here but the mesh_data carries a
            # _bake_transform (e.g. the two-character export node already ran
            # the bake with a shared transform), use that so shape-key basis
            # meshes align with the OBJ vertices.
            _R_for_sk, _off_for_sk = R_used, offset_used
            if _R_for_sk is None and _off_for_sk is None:
                _pre = mesh_data.get('_bake_transform') if isinstance(mesh_data, dict) else None
                if isinstance(_pre, dict):
                    _R_for_sk = _pre.get('R')
                    _off_for_sk = _pre.get('offset')
            sk_data = _compute_face_shape_key_data(
                mesh_data, R_mhr=_R_for_sk, offset_mhr=_off_for_sk
            )
            if sk_data is not None:
                shape_keys_json_path = os.path.join(
                    temp_dir, f"shape_keys_{int(time.time())}.json"
                )
                with open(shape_keys_json_path, 'w') as f:
                    json.dump({
                        "base_vertices": sk_data["base_vertices"].tolist(),
                        "deltas": sk_data["deltas"].tolist(),
                        "values": sk_data["values"].tolist(),
                        "names": sk_data["names"],
                    }, f)

        try:
            # Use isolated bpy exporter in sam3dbody venv
            exporter = BpyFBXExporter()
            result = exporter.export(
                input_obj_path=temp_obj_path,
                output_fbx_path=output_fbx_path,
                skeleton_json_path=skeleton_json_path,
                shape_keys_json_path=shape_keys_json_path,
            )

            if not result.get("success"):
                raise RuntimeError(f"FBX export failed")

            if not os.path.exists(output_fbx_path):
                raise RuntimeError(f"Export completed but output file not found: {output_fbx_path}")

            return io.NodeOutput(os.path.basename(output_fbx_path))

        finally:
            # Clean up temporary files
            if os.path.exists(temp_obj_path):
                os.unlink(temp_obj_path)
            if skeleton_json_path and os.path.exists(skeleton_json_path):
                os.unlink(skeleton_json_path)
            if shape_keys_json_path and os.path.exists(shape_keys_json_path):
                os.unlink(shape_keys_json_path)

    @staticmethod
    def _write_obj_file(filepath, vertices, faces):
        """Write mesh to OBJ file format."""
        with open(filepath, 'w') as f:
            for v in vertices:
                f.write(f"v {v[0]:.6f} {-v[1]:.6f} {-v[2]:.6f}\n")
            for face in faces:
                f.write(f"f {face[0]+1} {face[1]+1} {face[2]+1}\n")


class SAM3DBodyExportMultipleFBX(io.ComfyNode):
    """
    Export multiple SAM3D Body meshes with skeletons to a single FBX file.

    Takes multi-person mesh data and exports all meshes with their armatures
    into a single combined FBX file.
    """

    @classmethod
    def define_schema(cls):
        return io.Schema(
            node_id="SAM3DBodyExportMultipleFBX",
            display_name="SAM 3D Body: Export Multiple FBX",
            category="SAM3DBody/export",
            is_output_node=True,
            inputs=[
                io.Custom("SAM3D_MULTI_OUTPUT").Input("multi_mesh_data",
                    tooltip="Multi-person mesh data from SAM3D Body Process Multiple node"),
                io.String.Input("output_filename", default="sam3d_multi_rigged.fbx",
                    tooltip="Output filename for the combined FBX file"),
                io.Boolean.Input("combine", default=True,
                    tooltip="When enabled, exports all people into a single FBX file (works with Preview3D). When disabled, creates separate FBX files per person."),
            ],
            outputs=[
                io.String.Output(display_name="fbx_path"),
            ],
        )

    @classmethod
    def execute(cls, multi_mesh_data, output_filename, combine):
        """Export all meshes with skeletons to FBX file(s).

        Args:
            multi_mesh_data: Multi-person mesh data from SAM3D Body Process Multiple node
            output_filename: Output filename for the FBX file
            combine: If True, export all people into single FBX. If False, create separate FBX per person.
        """
        import comfy.model_management
        import comfy.utils
        import bpy

        num_people = multi_mesh_data.get("num_people", 0)
        people = multi_mesh_data.get("people", [])
        faces = multi_mesh_data.get("faces")

        log.info(f" num_people from data: {num_people}")
        log.info(f" actual people list length: {len(people)}")

        if num_people == 0 or len(people) == 0:
            raise RuntimeError("No mesh data to export")

        # Setup output path
        output_dir = folder_paths.get_output_directory()
        if not output_filename.endswith('.fbx'):
            output_filename = output_filename + '.fbx'
        output_fbx_path = os.path.join(output_dir, output_filename)

        # Find MHR model path for skinning weights
        mhr_model_path = find_mhr_model_path(multi_mesh_data)

        # Load skinning data once (same for all people)
        skinning_data = None
        joint_parents = None
        if mhr_model_path and os.path.exists(mhr_model_path):
            try:
                mhr_model = torch.jit.load(mhr_model_path, map_location='cpu')
                lbs = mhr_model.character_torch.linear_blend_skinning

                vert_indices = lbs.vert_indices_flattened.cpu().numpy().astype(int)
                skin_indices = lbs.skin_indices_flattened.cpu().numpy().astype(int)
                skin_weights = lbs.skin_weights_flattened.cpu().numpy().astype(float)

                vertex_weights = {}
                for j in range(len(vert_indices)):
                    vert_idx = int(vert_indices[j])
                    bone_idx = int(skin_indices[j])
                    weight = float(skin_weights[j])
                    if vert_idx not in vertex_weights:
                        vertex_weights[vert_idx] = []
                    vertex_weights[vert_idx].append([bone_idx, weight])

                # Get joint parents
                joint_parents = mhr_model.character_torch.skeleton.joint_parents.cpu().numpy().astype(int).tolist()
            except Exception:
                pass

        # Build combined data structure for all people
        temp_files = []
        combined_data = {
            "output_path": output_fbx_path,
            "people": [],
        }

        try:
            pbar = comfy.utils.ProgressBar(len(people))
            for i, person in enumerate(people):
                comfy.model_management.throw_exception_if_processing_interrupted()
                vertices = person.get("pred_vertices")
                joint_coords = person.get("pred_joint_coords")
                cam_t = person.get("pred_cam_t")  # Camera translation for world positioning
                global_rots = person.get("pred_global_rots")  # Global joint rotations for bone orientations

                if vertices is None:
                    pbar.update(1)
                    continue

                # Convert to numpy
                if isinstance(vertices, torch.Tensor):
                    vertices = vertices.cpu().numpy()
                if joint_coords is not None and isinstance(joint_coords, torch.Tensor):
                    joint_coords = joint_coords.cpu().numpy()
                if cam_t is not None and isinstance(cam_t, torch.Tensor):
                    cam_t = cam_t.cpu().numpy()
                if global_rots is not None and isinstance(global_rots, torch.Tensor):
                    global_rots = global_rots.cpu().numpy()

                # Apply world position offset from camera translation
                if cam_t is not None:
                    vertices = vertices + cam_t  # Broadcast adds cam_t to each vertex
                    if joint_coords is not None:
                        joint_coords = joint_coords + cam_t

                # Write OBJ file for this person
                temp_obj = tempfile.NamedTemporaryFile(suffix=f'_person{i}.obj', delete=False)
                temp_files.append(temp_obj.name)
                cls._write_obj_file(temp_obj.name, vertices, faces)

                # Prepare skeleton data
                skeleton_info = {}
                if joint_coords is not None:
                    # Apply coordinate transform to joint positions (flip Y and Z)
                    joint_coords_flipped = joint_coords.copy()
                    joint_coords_flipped[:, 1] = -joint_coords_flipped[:, 1]
                    joint_coords_flipped[:, 2] = -joint_coords_flipped[:, 2]

                    skeleton_info = {
                        "joint_positions": joint_coords_flipped.tolist(),
                        "num_joints": len(joint_coords),
                    }

                    if len(joint_coords) == 127:
                        from .sam_3d_body.mhr_joint_names import MHR_JOINT_NAMES
                        skeleton_info["joint_names"] = MHR_JOINT_NAMES

                    # Add skinning weights (build per-vertex data)
                    if vertex_weights:
                        num_vertices = len(vertices)
                        skinning_list = []
                        for vert_idx in range(num_vertices):
                            if vert_idx in vertex_weights:
                                skinning_list.append(vertex_weights[vert_idx])
                            else:
                                skinning_list.append([])
                        skeleton_info["skinning_weights"] = skinning_list

                    # Add joint parents
                    if joint_parents:
                        skeleton_info["joint_parents"] = joint_parents

                    # Add global joint rotations for better bone orientations in FBX
                    if global_rots is not None:
                        skeleton_info["global_rotations"] = global_rots.tolist()
                        log.info(f" Person {i}: Including global_rots shape {global_rots.shape}")

                # Add person to combined data
                combined_data["people"].append({
                    "obj_path": temp_obj.name,
                    "skeleton": skeleton_info,
                    "index": i,
                })
                pbar.update(1)

            log.info(f" people added to combined_data: {len(combined_data['people'])}")
            log.info(f" combine: {combine}")

            if not combined_data["people"]:
                raise RuntimeError("No valid mesh data to export")

            exporter = BpyFBXExporter()

            if combine:
                # Combined mode: export all people into a single FBX file
                # Write skeleton JSON files for each person and build combined data
                people_data_for_export = []
                for person_data in combined_data["people"]:
                    idx = person_data["index"]
                    skeleton_info = person_data.get("skeleton", {})

                    # Write skeleton JSON for this person if available
                    person_skeleton_json = None
                    if skeleton_info:
                        person_skeleton_json = tempfile.NamedTemporaryFile(
                            suffix=f'_person{idx}_skeleton.json', delete=False, mode='w'
                        )
                        temp_files.append(person_skeleton_json.name)
                        json.dump(skeleton_info, person_skeleton_json)
                        person_skeleton_json.close()
                        person_skeleton_json = person_skeleton_json.name

                    people_data_for_export.append({
                        "obj_path": person_data["obj_path"],
                        "skeleton_json_path": person_skeleton_json,
                        "index": idx,
                    })

                # Write combined data JSON file
                combined_json = tempfile.NamedTemporaryFile(
                    suffix='_combined_export.json', delete=False, mode='w'
                )
                temp_files.append(combined_json.name)
                json.dump(people_data_for_export, combined_json)
                combined_json.close()

                # Export all people into single FBX via combined_json_path
                result = exporter.export(
                    input_obj_path=None,
                    output_fbx_path=output_fbx_path,
                    combined_json_path=combined_json.name
                )

                if not result.get("success"):
                    raise RuntimeError("Combined FBX export failed")

                log.info(f" Combined FBX created: {output_fbx_path}")
                return io.NodeOutput(os.path.basename(output_fbx_path))

            else:
                # Separate mode: export each person to individual FBX files
                exported_files = []
                pbar_export = comfy.utils.ProgressBar(len(combined_data["people"]))

                for person_data in combined_data["people"]:
                    comfy.model_management.throw_exception_if_processing_interrupted()
                    obj_path = person_data["obj_path"]
                    idx = person_data["index"]
                    skeleton_info = person_data.get("skeleton", {})

                    # Create per-person FBX filename
                    if len(combined_data["people"]) == 1:
                        person_fbx_path = output_fbx_path
                    else:
                        person_fbx_path = output_fbx_path.replace('.fbx', f'_person{idx}.fbx')

                    # Write skeleton JSON for this person if available
                    person_skeleton_json = None
                    if skeleton_info:
                        person_skeleton_json = tempfile.NamedTemporaryFile(
                            suffix=f'_person{idx}_skeleton.json', delete=False, mode='w'
                        )
                        temp_files.append(person_skeleton_json.name)
                        json.dump(skeleton_info, person_skeleton_json)
                        person_skeleton_json.close()
                        person_skeleton_json = person_skeleton_json.name

                    # Export using isolated bpy
                    result = exporter.export(
                        input_obj_path=obj_path,
                        output_fbx_path=person_fbx_path,
                        skeleton_json_path=person_skeleton_json
                    )

                    if result.get("success"):
                        exported_files.append(person_fbx_path)
                    else:
                        raise RuntimeError(f"FBX export failed for person {idx}")
                    pbar_export.update(1)

                if not exported_files:
                    raise RuntimeError("No FBX files were exported")

                # Return the first exported file (separate mode returns first file for compatibility)
                output_fbx_path = exported_files[0]
                log.info(f" Separate FBX files created: {len(exported_files)} files")
                return io.NodeOutput(os.path.basename(output_fbx_path))

        finally:
            # Clean up temp files
            for temp_file in temp_files:
                if os.path.exists(temp_file):
                    try:
                        os.unlink(temp_file)
                    except Exception:
                        pass

    @staticmethod
    def _write_obj_file(filepath, vertices, faces):
        """Write mesh to OBJ file format."""
        with open(filepath, 'w') as f:
            for v in vertices:
                f.write(f"v {v[0]:.6f} {-v[1]:.6f} {-v[2]:.6f}\n")
            for face in faces:
                f.write(f"f {face[0]+1} {face[1]+1} {face[2]+1}\n")


class SAM3DBodyExportTwoCharactersFBX(io.ComfyNode):
    """
    Process one image containing TWO characters with a mask that isolates
    character A. Runs the SAM3D pipeline twice — once with the mask (character
    A), once with the inverted mask (character B) — and exports each result
    to its own FBX file.
    """

    @classmethod
    def define_schema(cls):
        return io.Schema(
            node_id="SAM3DBodyExportTwoCharactersFBX",
            display_name="SAM 3D Body: Export Two Characters (FBX)",
            category="SAM3DBody/export",
            is_output_node=True,
            inputs=[
                io.Custom("SAM3D_MODEL").Input("model",
                    tooltip="Loaded SAM 3D Body model from Load node"),
                io.Image.Input("image",
                    tooltip="Input image containing two humans"),
                io.Mask.Input("mask",
                    tooltip="Mask isolating character A"),
                io.Mask.Input("mask_b", optional=True,
                    tooltip="Optional mask isolating character B. If omitted, the inverted "
                            "'mask' input is used — but that inverted region covers most of the "
                            "image so the detector can accidentally pick character A again. "
                            "Provide an explicit tight mask for character B (e.g. from another "
                            "SAM/RMBG pass) to guarantee the correct character is detected."),
                io.String.Input("output_filename_a", default="sam3d_char_a.fbx",
                    tooltip="Output FBX filename for character A (the one under the given mask)"),
                io.String.Input("output_filename_b", default="sam3d_char_b.fbx",
                    tooltip="Output FBX filename for character B (the one under the inverted mask)"),
                io.Boolean.Input("overwrite", default=False,
                    tooltip="When enabled, both filenames are written verbatim (overwriting). "
                            "When disabled, an incrementing counter is appended so each run creates new files."),
                io.Combo.Input("bake_facing",
                    options=["off", "yaw", "full"], default="off",
                    tooltip="off: no reorientation.  yaw: rotate around vertical only to face -Y "
                            "(preserves tilt/lean/pitch — kneeling stays kneeling).  full: also "
                            "straighten the character upright.  In both baked modes the transform "
                            "is computed from CHARACTER A and applied to BOTH, so the two-character "
                            "spatial relationship is preserved."),
                io.Boolean.Input("preserve_scene_positions", default=True,
                    tooltip="When enabled, each character's model output is placed at its "
                            "camera-space world position (via pred_cam_t) BEFORE the bake, so the "
                            "two characters land at the spatial offset they had in the original "
                            "image. Disable to have both characters centered on their own body "
                            "(they will overlap when imported into the same scene)."),
                io.Float.Input("bbox_threshold", default=0.8, min=0.0, max=1.0, step=0.05,
                    tooltip="Detection confidence threshold"),
                io.Float.Input("nms_threshold", default=0.3, min=0.0, max=1.0, step=0.05,
                    tooltip="NMS threshold"),
                io.Combo.Input("inference_type", options=["full", "body", "hand"],
                    default="full",
                    tooltip="full = body + hand decoders, body = body decoder only, hand = hand only"),
                io.Boolean.Input("bake_face_shape_keys", default=False,
                    tooltip="Add 72 MHR face-expression shape keys to each character's mesh so "
                            "each blendshape can be sculpted independently in Blender. "
                            "Doubles the MHR forward count (once per character), so exports are "
                            "noticeably slower."),
            ],
            outputs=[
                io.String.Output(display_name="fbx_path_a"),
                io.String.Output(display_name="fbx_path_b"),
            ],
        )

    @classmethod
    def execute(cls, model, image, mask,
                output_filename_a="sam3d_char_a.fbx",
                output_filename_b="sam3d_char_b.fbx",
                overwrite=False, bake_facing="off",
                preserve_scene_positions=True,
                bbox_threshold=0.8, nms_threshold=0.3,
                inference_type="full", mask_b=None,
                bake_face_shape_keys=False):
        # Import the process node lazily to avoid a circular import at module load.
        from .process import SAM3DBodyProcessAdvanced

        # Character B mask: prefer the caller-supplied mask_b (tight per-character
        # bbox = reliable detection). Otherwise fall back to inverted mask_a,
        # which covers most of the image and can occasionally let the detector
        # pick character A again for both outputs.
        if mask_b is not None:
            char_b_mask = mask_b
        elif isinstance(mask, torch.Tensor):
            char_b_mask = 1.0 - mask
        else:
            import numpy as _np
            char_b_mask = 1.0 - _np.asarray(mask)

        def _process(m):
            out = SAM3DBodyProcessAdvanced.execute(
                model=model, image=image,
                bbox_threshold=bbox_threshold, nms_threshold=nms_threshold,
                inference_type=inference_type,
                detector_name="none", segmentor_name="none", fov_name="none",
                detector_path="", segmentor_path="", fov_path="",
                mask=m,
            )
            # NodeOutput exposes the return tuple via .args; mesh_data is [0].
            return out.args[0]

        mesh_a = _process(mask)
        mesh_b = _process(char_b_mask)

        # Place each character at its camera-space world position (pred_cam_t)
        # so the two-character spatial offset from the source image is preserved.
        # Without this, both characters are output centered on their own body
        # and overlap when imported into the same Blender scene.
        if preserve_scene_positions:
            def _apply_cam_t(md):
                cam_t = md.get("camera")
                if cam_t is None:
                    return
                if isinstance(cam_t, torch.Tensor):
                    cam_t = cam_t.cpu().numpy()
                cam_t = np.asarray(cam_t, dtype=np.float32).reshape(-1)
                v = md.get("vertices")
                j = md.get("joint_coords")
                if v is not None:
                    if isinstance(v, torch.Tensor):
                        v = v.cpu().numpy()
                    md["vertices"] = v.astype(np.float32) + cam_t
                if j is not None:
                    if isinstance(j, torch.Tensor):
                        j = j.cpu().numpy()
                    md["joint_coords"] = j.astype(np.float32) + cam_t
                # Remember cam_t so shape-key basis meshes (computed later
                # from raw MHR output) can be shifted by the same amount.
                # _apply_bake_facing SUBTRACTS offset from vertices, so we
                # store -cam_t here to undo the addition consistently.
                md['_bake_transform'] = {'R': None, 'offset': -cam_t}

            _apply_cam_t(mesh_a)
            _apply_cam_t(mesh_b)

        # If bake_facing is on, compute the transform ONCE from character A
        # and apply the SAME transform to both. That way A ends up at the
        # canonical origin and B lands at its correct offset relative to A —
        # spatial relationship between the two characters is preserved
        # instead of each getting individually re-centered to origin.
        if bake_facing and bake_facing != "off":
            def _to_np(x):
                return x.cpu().numpy() if isinstance(x, torch.Tensor) else x

            ja = _to_np(mesh_a.get('joint_coords'))
            va = _to_np(mesh_a.get('vertices'))
            jb = _to_np(mesh_b.get('joint_coords'))
            vb = _to_np(mesh_b.get('vertices'))

            # Character A: compute + apply, get the shared (R, offset) back.
            ja, va, R_shared, off_shared = _apply_bake_facing(
                ja, va, mode=bake_facing)
            # Character B: apply the SAME (R, offset). compute_missing=False
            # tells the helper NOT to derive its own transform from B's
            # landmarks and NOT to pin B's joint 0 to origin — its post-shift
            # position encodes B's spatial location relative to A.
            jb, vb, _, _ = _apply_bake_facing(
                jb, vb, R_mhr=R_shared, offset_mhr=off_shared,
                compute_missing=False, mode=bake_facing)

            mesh_a['joint_coords'] = ja
            mesh_a['vertices'] = va
            mesh_b['joint_coords'] = jb
            mesh_b['vertices'] = vb
            # Stash the shared transform so downstream shape-key baking (which
            # sees bake_facing="off" and would otherwise assume no transform)
            # can apply the SAME rotation+offset to the base/basis meshes it
            # computes from MHR.
            mesh_a['_bake_transform'] = {'R': R_shared, 'offset': off_shared}
            mesh_b['_bake_transform'] = {'R': R_shared, 'offset': off_shared}

        # Export both with bake_facing=False (the transform, if any, was
        # already applied above with a shared reference).
        def _export_no_bake(mesh_data, filename):
            out = SAM3DBodyExportFBX.execute(
                mesh_data=mesh_data,
                output_filename=filename,
                overwrite=overwrite,
                bake_facing="off",
                bake_face_shape_keys=bake_face_shape_keys,
            )
            return out.args[0]

        path_a = _export_no_bake(mesh_a, output_filename_a)
        path_b = _export_no_bake(mesh_b, output_filename_b)
        return io.NodeOutput(path_a, path_b)


# Register nodes
NODE_CLASS_MAPPINGS = {
    "SAM3DBodyExportFBX": SAM3DBodyExportFBX,
    "SAM3DBodyExportMultipleFBX": SAM3DBodyExportMultipleFBX,
    "SAM3DBodyExportTwoCharactersFBX": SAM3DBodyExportTwoCharactersFBX,
}

NODE_DISPLAY_NAME_MAPPINGS = {
    "SAM3DBodyExportFBX": "SAM 3D Body: Export FBX",
    "SAM3DBodyExportMultipleFBX": "SAM 3D Body: Export Multiple FBX",
    "SAM3DBodyExportTwoCharactersFBX": "SAM 3D Body: Export Two Characters (FBX)",
}
