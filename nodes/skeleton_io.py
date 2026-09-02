# Copyright (c) 2025 Andrea Pozzetti
# SPDX-License-Identifier: MIT
"""
Skeleton I/O nodes for SAM 3D Body.

Save, load, and manipulate skeleton data.
"""

import logging
import os
import json
import time
import subprocess
import numpy as np
import torch
import folder_paths
from comfy_api.latest import io

log = logging.getLogger("sam3dbody")


class SAM3DBodySaveSkeleton(io.ComfyNode):
    """
    Save skeleton data to file in multiple formats.

    Exports skeleton with joint positions, rotations, and MHR parameters
    to JSON, BVH, or FBX format.
    """

    @classmethod
    def define_schema(cls):
        return io.Schema(
            node_id="SAM3DBodySaveSkeleton",
            display_name="SAM 3D Body: Save Skeleton",
            category="SAM3DBody/skeleton",
            is_output_node=True,
            inputs=[
                io.Custom("SKELETON").Input("skeleton",
                    tooltip="Skeleton data from SAM3D Body Process node"),
                io.String.Input("output_filename", default="skeleton",
                    tooltip="Output filename (extension will be added based on format)"),
                io.Combo.Input("format", options=["json", "bvh", "fbx"],
                    default="json",
                    tooltip="Export format: JSON (full data), BVH (animation), or FBX (armature)"),
            ],
            outputs=[
                io.String.Output(display_name="filepath"),
            ],
        )

    @classmethod
    def execute(cls, skeleton, output_filename, format="json"):
        """Save skeleton to file in specified format."""
        log.info(f" Saving skeleton as {format.upper()}...")

        # Prepare output path
        output_dir = folder_paths.get_output_directory()

        # Add extension if not present
        if not output_filename.endswith(f'.{format}'):
            output_filename = f"{output_filename}.{format}"

        output_path = os.path.join(output_dir, output_filename)

        # Save based on format
        if format == "json":
            cls._save_json(skeleton, output_path)
        elif format == "bvh":
            cls._save_bvh(skeleton, output_path)
        elif format == "fbx":
            cls._save_fbx(skeleton, output_path)
        else:
            raise ValueError(f"Unsupported format: {format}")

        log.info(f" OK Saved to: {output_path}")
        return io.NodeOutput(os.path.basename(output_path))

    @staticmethod
    def _save_json(skeleton, output_path):
        """Save skeleton to JSON format (full data)."""
        # Convert tensors to numpy/lists
        json_data = {}

        for key, value in skeleton.items():
            if value is None:
                json_data[key] = None
            elif isinstance(value, torch.Tensor):
                json_data[key] = value.cpu().numpy().tolist()
            elif isinstance(value, np.ndarray):
                json_data[key] = value.tolist()
            else:
                json_data[key] = value

        with open(output_path, 'w') as f:
            json.dump(json_data, f, indent=2)

        log.info(f" Saved JSON with {len(json_data)} fields")

    @staticmethod
    def _save_bvh(skeleton, output_path):
        """Save skeleton to BVH format (animation format)."""
        joint_positions = skeleton.get("joint_positions")
        joint_rotations = skeleton.get("joint_rotations")

        if joint_positions is None:
            raise RuntimeError("Skeleton has no joint_positions data")

        # Convert to numpy if needed
        if isinstance(joint_positions, torch.Tensor):
            joint_positions = joint_positions.cpu().numpy()
        if joint_rotations is not None and isinstance(joint_rotations, torch.Tensor):
            joint_rotations = joint_rotations.cpu().numpy()

        # Create BVH file
        with open(output_path, 'w') as f:
            # Write header
            f.write("HIERARCHY\n")
            f.write("ROOT Hips\n")
            f.write("{\n")

            # For simplicity, create a flat hierarchy (all joints as children of root)
            # In a full implementation, you'd use the proper MHR hierarchy
            root_pos = joint_positions[0] if len(joint_positions) > 0 else [0, 0, 0]
            f.write(f"  OFFSET {root_pos[0]:.6f} {root_pos[1]:.6f} {root_pos[2]:.6f}\n")
            f.write("  CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation\n")

            # Write joints (simplified - first 20 joints)
            import comfy.model_management
            import comfy.utils
            num_joints = min(len(joint_positions), 20)
            pbar = comfy.utils.ProgressBar(num_joints)
            for i in range(1, num_joints):
                comfy.model_management.throw_exception_if_processing_interrupted()
                pos = joint_positions[i]
                f.write(f"  JOINT Joint_{i:03d}\n")
                f.write("  {\n")
                f.write(f"    OFFSET {pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f}\n")
                f.write("    CHANNELS 3 Zrotation Xrotation Yrotation\n")

                # End site for leaf joints
                if i == num_joints - 1:
                    f.write("    End Site\n")
                    f.write("    {\n")
                    f.write("      OFFSET 0.0 0.0 0.0\n")
                    f.write("    }\n")

                f.write("  }\n")
                pbar.update(1)

            f.write("}\n")

            # Write motion data (single frame)
            f.write("MOTION\n")
            f.write("Frames: 1\n")
            f.write("Frame Time: 0.033333\n")

            # Write frame data (positions + rotations)
            # Root position
            f.write(f"{root_pos[0]:.6f} {root_pos[1]:.6f} {root_pos[2]:.6f} ")
            f.write("0.0 0.0 0.0 ")  # Root rotation (simplified)

            # Joint rotations (simplified - zeros for now)
            for i in range(1, num_joints):
                f.write("0.0 0.0 0.0 ")

            f.write("\n")

        log.info(f" Saved BVH with {num_joints} joints")

    @staticmethod
    def _save_fbx(skeleton, output_path):
        """Save skeleton to FBX format (armature only) using Blender."""
        joint_positions = skeleton.get("joint_positions")

        if joint_positions is None:
            raise RuntimeError("Skeleton has no joint_positions data")

        # Convert to numpy if needed
        if isinstance(joint_positions, torch.Tensor):
            joint_positions = joint_positions.cpu().numpy()

        # Center on joint 0 (world) so world bone ends up at armature origin.
        # That way scaling the source armature keeps world in place — critical
        # for the target master control staying at the correct height.
        if len(joint_positions) > 0:
            world_offset = joint_positions[0].copy()
            joint_positions = joint_positions - world_offset

        # Save skeleton data to temporary JSON
        temp_dir = folder_paths.get_temp_directory()
        skeleton_json_path = os.path.join(temp_dir, f"skeleton_{int(time.time())}.json")

        skeleton_data = {
            "joint_positions": joint_positions.tolist(),
            "num_joints": len(joint_positions),
        }

        joint_parents = skeleton.get("joint_parents")
        if joint_parents is not None:
            if isinstance(joint_parents, torch.Tensor):
                joint_parents = joint_parents.cpu().numpy()
            if isinstance(joint_parents, np.ndarray):
                joint_parents = joint_parents.astype(int).tolist()
            else:
                joint_parents = [int(p) for p in joint_parents]
            skeleton_data["joint_parents"] = joint_parents

        if len(joint_positions) == 127:
            from .sam_3d_body.mhr_joint_names import MHR_JOINT_NAMES, MHR_TAIL_CHILD_OVERRIDES
            skeleton_data["joint_names"] = MHR_JOINT_NAMES
            name_to_idx = {n: i for i, n in enumerate(MHR_JOINT_NAMES)}
            resolved = {}
            for parent_name, child_name in MHR_TAIL_CHILD_OVERRIDES.items():
                pi, ci = name_to_idx.get(parent_name), name_to_idx.get(child_name)
                if pi is not None and ci is not None:
                    resolved[str(pi)] = ci
            if resolved:
                skeleton_data["tail_overrides"] = resolved

        with open(skeleton_json_path, 'w') as f:
            json.dump(skeleton_data, f)

        try:
            # Find Blender
            blender_exe = SAM3DBodySaveSkeleton._find_blender()

            if not blender_exe or not os.path.exists(blender_exe):
                raise RuntimeError("Blender not found. Set BLENDER_EXE environment variable or install Blender.")

            # Create Blender script
            blender_script = SAM3DBodySaveSkeleton._create_blender_skeleton_export_script()
            script_path = os.path.join(temp_dir, f"export_skeleton_{int(time.time())}.py")

            with open(script_path, 'w') as f:
                f.write(blender_script)

            # Run Blender (set SAM3DBODY_BLENDER_GUI=1 to run non-headless for debugging)
            gui_debug = os.environ.get("SAM3DBODY_BLENDER_GUI") == "1"
            cmd = [blender_exe]
            if not gui_debug:
                cmd.append('--background')
            cmd += ['--python', script_path, '--', skeleton_json_path, output_path]

            log.info(f" Running Blender to export FBX... (gui_debug={gui_debug})")
            result = subprocess.run(cmd, capture_output=True, text=True,
                                    timeout=None if gui_debug else 60,
                                    encoding='utf-8', errors='replace')

            if result.returncode != 0:
                log.info(f" Blender stderr: {result.stderr}")
                log.info(f" Blender stdout: {result.stdout}")
                raise RuntimeError(f"Blender export failed with return code {result.returncode}")

            if not os.path.exists(output_path):
                raise RuntimeError(
                    f"Export completed but output file not found: {output_path}\n"
                    f"--- Blender stdout ---\n{result.stdout}\n"
                    f"--- Blender stderr ---\n{result.stderr}"
                )

            log.info(f" Exported skeleton as FBX")

        finally:
            # Clean up temporary files
            if os.path.exists(skeleton_json_path):
                os.unlink(skeleton_json_path)
            if 'script_path' in locals() and os.path.exists(script_path):
                os.unlink(script_path)

    @staticmethod
    def _find_blender():
        """Try to find Blender executable."""
        possible_paths = [
            "/usr/bin/blender",
            "/usr/local/bin/blender",
            "C:\\Program Files\\Blender Foundation\\Blender 4.0\\blender.exe",
            "C:\\Program Files\\Blender Foundation\\Blender 3.6\\blender.exe",
        ]

        env_path = os.environ.get("BLENDER_EXE")
        if env_path:
            possible_paths.insert(0, env_path)

        for path in possible_paths:
            if os.path.exists(path):
                return path

        try:
            result = subprocess.run(['which', 'blender'], capture_output=True, text=True)
            if result.returncode == 0:
                return result.stdout.strip()
        except Exception as e:
            log.debug("Failed to locate blender via 'which': %s", e)

        return None

    @staticmethod
    def _create_blender_skeleton_export_script():
        """Create Blender script for exporting skeleton to FBX."""
        return """
import bpy
import sys
import json
import traceback

MARK = "@@SAM3D@@"
def say(msg):
    print(f"{MARK} {msg}", flush=True)

try:
    say("script start")

    args = sys.argv[sys.argv.index("--") + 1:]
    skeleton_json = args[0]
    fbx_path = args[1]
    say(f"skeleton_json={skeleton_json}")
    say(f"fbx_path={fbx_path}")

    with open(skeleton_json, 'r') as f:
        skeleton_data = json.load(f)
    joint_positions = skeleton_data['joint_positions']
    joint_names = skeleton_data.get('joint_names')
    joint_parents = skeleton_data.get('joint_parents')
    tail_overrides = {int(k): int(v) for k, v in (skeleton_data.get('tail_overrides') or {}).items()}
    say(f"loaded {len(joint_positions)} joints (named={bool(joint_names)}, parented={bool(joint_parents)}, overrides={len(tail_overrides)})")

    try:
        bpy.ops.preferences.addon_enable(module="io_scene_fbx")
        say("io_scene_fbx addon enabled")
    except Exception as e:
        say(f"addon_enable failed (may already be enabled): {e}")

    for obj in list(bpy.data.objects):
        bpy.data.objects.remove(obj, do_unlink=True)
    say("scene cleared")

    armature = bpy.data.armatures.new("SAM3D_Skeleton_Armature")
    armature_obj = bpy.data.objects.new("SAM3D_Skeleton", armature)
    bpy.context.scene.collection.objects.link(armature_obj)
    bpy.context.view_layer.objects.active = armature_obj
    armature_obj.select_set(True)
    say("armature created")

    bpy.ops.object.mode_set(mode='EDIT')

    n = len(joint_positions)
    use_named = joint_names and len(joint_names) == n
    use_parents = joint_parents and len(joint_parents) == n

    children_of = [[] for _ in range(n)]
    if use_parents:
        for c, p in enumerate(joint_parents):
            if 0 <= p < n and p != c:
                children_of[p].append(c)

    def _tail(i):
        h = joint_positions[i]
        kids = children_of[i]
        if kids:
            chosen = tail_overrides.get(i)
            if chosen is None or chosen not in kids:
                chosen = kids[0]
            t = joint_positions[chosen]
            dx, dy, dz = t[0]-h[0], t[1]-h[1], t[2]-h[2]
            if (dx*dx + dy*dy + dz*dz) ** 0.5 < 1e-4:
                return (h[0], h[1] + 0.05, h[2])
            return (t[0], t[1], t[2])
        p = joint_parents[i] if use_parents else -1
        if 0 <= p < n and p != i:
            ph = joint_positions[p]
            dx, dy, dz = h[0]-ph[0], h[1]-ph[1], h[2]-ph[2]
            m = (dx*dx + dy*dy + dz*dz) ** 0.5
            if m > 1e-4:
                s = min(0.25, 0.05 / m)
                return (h[0] + dx*s, h[1] + dy*s, h[2] + dz*s)
        return (h[0], h[1] + 0.05, h[2])

    created = []
    for i, pos in enumerate(joint_positions):
        name = joint_names[i] if use_named else f"Joint_{i:03d}"
        bone = armature.edit_bones.new(name)
        bone.head = (pos[0], pos[1], pos[2])
        bone.tail = _tail(i)
        created.append(bone)
    say(f"added {n} bones")

    if use_parents:
        for i, p in enumerate(joint_parents):
            if 0 <= p < n and p != i:
                created[i].parent = created[p]
                created[i].use_connect = False
        say("parents wired")

    # Character-relative roll for torso bones (character forward = right x up),
    # so full/twist-preserving retargets don't inherit a random Blender-default roll.
    if use_named:
        from mathutils import Vector as _V
        _idx = {nm: i for i, nm in enumerate(joint_names)}
        try:
            _l = _V(joint_positions[_idx['l_upleg']])
            _r = _V(joint_positions[_idx['r_upleg']])
            _root = _V(joint_positions[_idx['root']])
            _head = _V(joint_positions[_idx['c_head']])
            _right = _r - _l
            _up = _head - _root
            if _right.length > 1e-4 and _up.length > 1e-4:
                _right.normalize(); _up.normalize()
                _fwd = _right.cross(_up)
                if _fwd.length > 1e-4:
                    _fwd.normalize()
                    for _bn in ('world', 'root', 'c_spine0', 'c_spine1', 'c_spine2', 'c_spine3', 'c_neck', 'c_head'):
                        _b = armature.edit_bones.get(_bn)
                        if _b is not None:
                            _b.align_roll(_fwd)
                    # Reorient world/root to short vertical bones so their axes
                    # exactly match canonical — otherwise DELTA-mode retarget
                    # picks up spine-curvature or origin offset as tilt.
                    for _nm in ('world', 'root'):
                        _bn = armature.edit_bones.get(_nm)
                        if _bn is not None:
                            _h = _bn.head
                            _bn.tail = (_h[0] + _up.x*0.1, _h[1] + _up.y*0.1, _h[2] + _up.z*0.1)
                            _bn.align_roll(_fwd)
                    # Realign spine bones to overall spine direction rather than
                    # each pointing at its immediate child. Otherwise c_spine3
                    # points at c_neck which sits forward (cervical curve),
                    # tilting the chest bone regardless of actual spine posture.
                    for _sname in ('c_spine0', 'c_spine1', 'c_spine2', 'c_spine3'):
                        _sb = armature.edit_bones.get(_sname)
                        if _sb is not None:
                            _sh = _sb.head
                            _st = _sb.tail
                            _length = ((_st[0]-_sh[0])**2 + (_st[1]-_sh[1])**2 + (_st[2]-_sh[2])**2) ** 0.5
                            if _length < 1e-4:
                                _length = 0.05
                            _sb.tail = (_sh[0] + _up.x*_length, _sh[1] + _up.y*_length, _sh[2] + _up.z*_length)
                            _sb.align_roll(_fwd)
                    # Head roll from eye anatomy so head turn transfers
                    _head_fwd = None
                    try:
                        _le = _V(joint_positions[_idx['l_eye']])
                        _re = _V(joint_positions[_idx['r_eye']])
                        _cn = _V(joint_positions[_idx['c_head_null']])
                        _ch = _V(joint_positions[_idx['c_head']])
                        _hright = _re - _le
                        _hup = _cn - _ch
                        if _hright.length > 1e-4 and _hup.length > 1e-4:
                            _hright.normalize(); _hup.normalize()
                            _hf = _hright.cross(_hup)
                            if _hf.length > 1e-4:
                                _hf.normalize()
                                _head_fwd = _hf
                                _hb = armature.edit_bones.get('c_head')
                                if _hb is not None:
                                    _hb.align_roll(_head_fwd)
                    except (KeyError, IndexError):
                        pass
                    # Neck roll: blend body and head forward
                    if _head_fwd is not None:
                        _nf = (_fwd + _head_fwd) * 0.5
                        if _nf.length > 1e-4:
                            _nf.normalize()
                            _nb = armature.edit_bones.get('c_neck')
                            if _nb is not None:
                                _nb.align_roll(_nf)
                    say("torso rolls aligned; head roll from eye anatomy; spines to overall spine direction")
        except (KeyError, IndexError):
            pass

    bpy.ops.object.mode_set(mode='OBJECT')

    say("calling export_scene.fbx ...")
    bpy.ops.export_scene.fbx(
        filepath=fbx_path,
        use_selection=False,
        object_types={'ARMATURE'},
        add_leaf_bones=False,
    )
    say(f"export_scene.fbx returned")

    import os as _os
    say(f"file exists after export: {_os.path.exists(fbx_path)}")
    say("script done OK")

except Exception:
    say("SCRIPT FAILED with exception:")
    for line in traceback.format_exc().splitlines():
        say(line)
    raise
"""


class SAM3DBodyLoadSkeleton(io.ComfyNode):
    """
    Load skeleton data from file.

    Supports JSON, BVH, and FBX formats.
    """

    @classmethod
    def define_schema(cls):
        return io.Schema(
            node_id="SAM3DBodyLoadSkeleton",
            display_name="SAM 3D Body: Load Skeleton",
            category="SAM3DBody/skeleton",
            inputs=[
                io.String.Input("filepath", default="",
                    tooltip="Path to skeleton file (JSON, BVH, or FBX)"),
            ],
            outputs=[
                io.Custom("SKELETON").Output(display_name="skeleton"),
            ],
        )

    @classmethod
    def execute(cls, filepath):
        """Load skeleton from file."""
        log.info(f" Loading skeleton from: {filepath}")

        if not os.path.exists(filepath):
            # Try in output directory
            output_dir = folder_paths.get_output_directory()
            filepath = os.path.join(output_dir, filepath)

            if not os.path.exists(filepath):
                raise RuntimeError(f"Skeleton file not found: {filepath}")

        # Determine format from extension
        ext = os.path.splitext(filepath)[1].lower()

        if ext == '.json':
            skeleton = cls._load_json(filepath)
        elif ext == '.bvh':
            skeleton = cls._load_bvh(filepath)
        elif ext == '.fbx':
            skeleton = cls._load_fbx(filepath)
        else:
            raise ValueError(f"Unsupported file format: {ext}")

        log.info(f" OK Loaded skeleton")
        return io.NodeOutput(skeleton)

    @staticmethod
    def _load_json(filepath):
        """Load skeleton from JSON format."""
        with open(filepath, 'r') as f:
            json_data = json.load(f)

        # Convert lists back to numpy arrays
        skeleton = {}
        for key, value in json_data.items():
            if value is None:
                skeleton[key] = None
            elif isinstance(value, list):
                skeleton[key] = np.array(value)
            else:
                skeleton[key] = value

        log.info(f" Loaded JSON with {len(skeleton)} fields")
        return skeleton

    @staticmethod
    def _load_bvh(filepath):
        """Load skeleton from BVH format."""
        # Simplified BVH parser - extract joint positions from hierarchy
        joint_positions = []

        with open(filepath, 'r') as f:
            lines = f.readlines()

        # Parse joint positions from OFFSET lines
        for line in lines:
            if 'OFFSET' in line:
                parts = line.strip().split()
                if len(parts) == 4:  # OFFSET x y z
                    try:
                        pos = [float(parts[1]), float(parts[2]), float(parts[3])]
                        joint_positions.append(pos)
                    except ValueError:
                        continue

        if not joint_positions:
            raise RuntimeError("No valid joint positions found in BVH file")

        # Create skeleton dictionary
        skeleton = {
            "joint_positions": np.array(joint_positions),
            "joint_rotations": None,
            "pose_params": None,
            "shape_params": None,
            "scale_params": None,
            "hand_pose": None,
            "global_rot": None,
            "expr_params": None,
            "camera": None,
            "focal_length": None,
        }

        log.info(f" Loaded BVH with {len(joint_positions)} joints")
        return skeleton

    @staticmethod
    def _load_fbx(filepath):
        """Load skeleton from FBX format using Blender."""
        # This would require Blender to extract skeleton data from FBX
        # For now, raise not implemented
        raise NotImplementedError("Loading from FBX not yet implemented. Use JSON format for full compatibility.")


class SAM3DBodyAddMeshToSkeleton(io.ComfyNode):
    """
    Generate mesh from skeleton using MHR model.

    Takes skeleton data (pose, shape, scale parameters) and uses the
    MHR parametric model to generate the corresponding mesh.
    """

    @classmethod
    def define_schema(cls):
        return io.Schema(
            node_id="SAM3DBodyAddMeshToSkeleton",
            display_name="SAM 3D Body: Add Mesh to Skeleton",
            category="SAM3DBody/skeleton",
            inputs=[
                io.Custom("SKELETON").Input("skeleton",
                    tooltip="Skeleton data with pose/shape/scale parameters"),
                io.Custom("SAM3D_MODEL").Input("model",
                    tooltip="Loaded SAM3D Body model (needed for MHR)"),
            ],
            outputs=[
                io.Custom("SAM3D_OUTPUT").Output(display_name="mesh_data"),
            ],
        )

    @classmethod
    def execute(cls, skeleton, model):
        """Generate mesh from skeleton parameters using MHR model."""
        log.info(f" Generating mesh from skeleton...")

        try:
            # Extract MHR model from loaded model
            sam_3d_model = model["model"]

            # Get skeleton parameters
            pose_params = skeleton.get("pose_params")
            shape_params = skeleton.get("shape_params")
            scale_params = skeleton.get("scale_params")
            hand_pose = skeleton.get("hand_pose")
            global_rot = skeleton.get("global_rot")
            expr_params = skeleton.get("expr_params")

            # Check if we have the necessary parameters
            if pose_params is None or shape_params is None or scale_params is None:
                raise RuntimeError("Skeleton missing required parameters (pose_params, shape_params, scale_params)")

            # Convert numpy arrays to tensors if needed
            if isinstance(pose_params, np.ndarray):
                pose_params = torch.from_numpy(pose_params).float()
            if isinstance(shape_params, np.ndarray):
                shape_params = torch.from_numpy(shape_params).float()
            if isinstance(scale_params, np.ndarray):
                scale_params = torch.from_numpy(scale_params).float()
            if hand_pose is not None and isinstance(hand_pose, np.ndarray):
                hand_pose = torch.from_numpy(hand_pose).float()
            if global_rot is not None and isinstance(global_rot, np.ndarray):
                global_rot = torch.from_numpy(global_rot).float()
            if expr_params is not None and isinstance(expr_params, np.ndarray):
                expr_params = torch.from_numpy(expr_params).float()

            # Add batch dimension if needed
            if pose_params.dim() == 1:
                pose_params = pose_params.unsqueeze(0)
            if shape_params.dim() == 1:
                shape_params = shape_params.unsqueeze(0)
            if scale_params.dim() == 1:
                scale_params = scale_params.unsqueeze(0)
            if hand_pose is not None and hand_pose.dim() == 1:
                hand_pose = hand_pose.unsqueeze(0)
            if global_rot is not None and global_rot.dim() == 1:
                global_rot = global_rot.unsqueeze(0)
            if expr_params is not None and expr_params.dim() == 1:
                expr_params = expr_params.unsqueeze(0)

            # Move to same device as model
            device = next(sam_3d_model.parameters()).device
            pose_params = pose_params.to(device)
            shape_params = shape_params.to(device)
            scale_params = scale_params.to(device)
            if hand_pose is not None:
                hand_pose = hand_pose.to(device)
            if global_rot is not None:
                global_rot = global_rot.to(device)
            if expr_params is not None:
                expr_params = expr_params.to(device)

            # Use MHR model to generate mesh
            # Access MHR from the model
            mhr = sam_3d_model.mhr_head.mhr if hasattr(sam_3d_model, 'mhr_head') else None

            if mhr is None:
                raise RuntimeError("MHR model not found in SAM3D model. Cannot generate mesh from skeleton.")

            # Build model parameters for MHR
            # Combine pose parameters: global_rot + pose_params + hand_pose
            # This is based on the MHR input format
            with torch.no_grad():
                # Generate mesh using MHR
                # Note: This is a simplified version - may need to adjust based on actual MHR interface
                vertices, joint_coords = mhr(
                    shape_params,
                    pose_params,
                    expr_params if expr_params is not None else torch.zeros(1, 72, device=device),
                )

            # Get faces from model
            faces = sam_3d_model.mhr_head.mhr.faces.cpu().numpy() if hasattr(sam_3d_model.mhr_head.mhr, 'faces') else None

            # Create mesh_data dictionary
            mesh_data = {
                "vertices": vertices,
                "faces": faces,
                "joints": None,  # 70 keypoints not directly available
                "joint_coords": joint_coords,  # 127 joints
                "joint_rotations": skeleton.get("joint_rotations"),
                "camera": skeleton.get("camera"),
                "focal_length": skeleton.get("focal_length"),
                "bbox": None,
                "pose_params": {
                    "body_pose": pose_params,
                    "hand_pose": hand_pose,
                    "global_rot": global_rot,
                    "shape": shape_params,
                    "scale": scale_params,
                    "expr": expr_params,
                },
                "raw_output": {},
                "all_people": [],
            }

            log.info(f" OK Generated mesh with {len(vertices[0])} vertices")
            return io.NodeOutput(mesh_data)

        except Exception as e:
            log.error(f"Failed to generate mesh: {str(e)}", exc_info=True)
            raise


# Register nodes
NODE_CLASS_MAPPINGS = {
    "SAM3DBodySaveSkeleton": SAM3DBodySaveSkeleton,
    "SAM3DBodyLoadSkeleton": SAM3DBodyLoadSkeleton,
    "SAM3DBodyAddMeshToSkeleton": SAM3DBodyAddMeshToSkeleton,
}

NODE_DISPLAY_NAME_MAPPINGS = {
    "SAM3DBodySaveSkeleton": "SAM 3D Body: Save Skeleton",
    "SAM3DBodyLoadSkeleton": "SAM 3D Body: Load Skeleton",
    "SAM3DBodyAddMeshToSkeleton": "SAM 3D Body: Add Mesh to Skeleton",
}
