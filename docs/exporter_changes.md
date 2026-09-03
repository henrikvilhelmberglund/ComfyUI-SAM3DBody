# Exporter changes vs. upstream ComfyUI-SAM3DBody

This fork extends the ComfyUI-SAM3DBody export pipeline with orientation
baking, roll alignment for cross-rig retargeting, MHR face-expression shape
key support, and a dedicated two-character export node. This document
summarises what changed and why.

All changes live in [nodes/export.py](../nodes/export.py) and
[nodes/process.py](../nodes/process.py), plus a small new module
[nodes/sam_3d_body/mhr_face_expression_names.py](../nodes/sam_3d_body/mhr_face_expression_names.py).

## Table of contents

- [Summary of what's new](#summary-of-whats-new)
- [`bake_facing` — orientation baking](#bake_facing--orientation-baking)
- [`align_all_rolls` — limb roll alignment](#align_all_rolls--limb-roll-alignment)
- [`bake_face_shape_keys` — 72 face-expression shape keys](#bake_face_shape_keys--72-face-expression-shape-keys)
- [`SAM3DBodyExportTwoCharactersFBX` — two-character node](#sam3dbodyexporttwocharactersfbx--two-character-node)
- [`SAM3DBodyFaceExpression` — inspect node](#sam3dbodyfaceexpression--inspect-node)
- [Torso roll alignment (already in the base)](#torso-roll-alignment-already-in-the-base)
- [Coordinate conventions cheat sheet](#coordinate-conventions-cheat-sheet)

## Summary of what's new

| Feature | Where | What it does |
| --- | --- | --- |
| `bake_facing` combo (`off / yaw / full`) | `SAM3DBodyExportFBX`, `SAM3DBodyExportTwoCharactersFBX` | Rotates the character during export to face a canonical direction (world -Y). `yaw` preserves tilt/lean/pitch; `full` also uprights the character. |
| `align_all_rolls` bool | Same two nodes | Extends the torso-chain roll alignment to arms/legs/clavicles so limb bones share a "Z along character-forward" convention with CloudRig/Rigify. |
| `bake_face_shape_keys` bool | Same two nodes | Adds 72 named MHR face-expression basis shape keys to the exported mesh, plus a Basis. Shape keys are activated to the source image's coefficients so the mesh looks correct at rest and each blendshape can be sculpted independently. |
| `preserve_scene_positions` bool | `SAM3DBodyExportTwoCharactersFBX` | For the two-character export, offsets each character by its `pred_cam_t` so the two characters land in their original spatial relationship instead of overlapping. |
| `mask_b` optional input | Same node | Second character mask. Falls back to inverted mask A if omitted, but an explicit tight mask B gives more reliable detection. |
| `SAM3DBodyExportTwoCharactersFBX` node | new | End-to-end two-character workflow: mask A + mask B → two individually-processed FBX exports with a shared orientation transform for spatial coherence. |
| `SAM3DBodyFaceExpression` node | new (in `process.py`) | Extracts the 72 face-expression coefficients from a processed result and outputs them as a JSON summary + a dict with FACS-style semantic names. |
| `MHR_FACE_EXPRESSION_NAMES` | new (`mhr_face_expression_names.py`) | The 72 FACS-style names from Meta's MHR docs, index-aligned to `face_expr_coeffs`. Used to name the shape keys and label the summary output. |

## `bake_facing` — orientation baking

Combo input on both single- and two-character export nodes.

- `off` — no reorientation. Character exports in whatever direction the
  MHR output produced.
- `yaw` — rotates the character around the vertical axis only, so its face
  ends up pointing at Blender's world `-Y` (the Rigify convention). Any
  tilt/lean/pitch in the pose is preserved: a kneeling character stays
  kneeling, only the horizontal facing changes.
- `full` — same as yaw, plus fully uprights the character. May reinterpret
  leaned poses (e.g. leaning-forward kneel becomes upright squat) but
  gives you a perfectly-canonical rest.

Both baked modes also recenter the "world" bone under the character's feet
so the FBX origin sits at ground level.

**Implementation**: `_apply_bake_facing` in `nodes/export.py`. Computes a
rotation matrix in MHR space from anatomical landmarks (`l_upleg / r_upleg`
for character-right, `c_head - root` for up, and — critically — the eye
midpoint minus head center for character-forward, so the sign of the
forward vector matches the direction the face is actually pointing rather
than the geometric cross-product's ambiguous sign).

## `align_all_rolls` — limb roll alignment

Bool toggle on both export nodes (default off).

When enabled, the exporter aligns the local Z axis of each limb bone
(clavicles, upper arms, forearms, wrists, thighs, shins, feet) to point at
the character's forward direction — the same convention CloudRig and
Rigify use for their limb bones.

Without this, Blender's FBX importer picks per-bone rolls essentially at
random, and cross-rig FULL-mode retargeting picks up that random twist as
visible pose noise. With this on, the retargeter can use FULL mode on
limbs without per-bone rotation offsets.

Fingers are intentionally skipped: at rest a finger's Y-axis often lies
near the character-forward direction, which would make `align_roll(forward)`
numerically unstable.

**Implementation**: `_align_limb_rolls_from_landmarks` in
`nodes/export.py`, gated by the `align_all_rolls` skeleton-JSON field
which the bpy exporter reads and honours.

## `bake_face_shape_keys` — 72 face-expression shape keys

Bool toggle. When enabled, the exporter runs the MHR model 73 times
(one base with `expr=0`, then 72 basis meshes each with a single
coefficient set to 1) in a single batched forward pass, computes the
per-basis vertex deltas, and writes them into the FBX as Blender shape
keys.

The Basis shape key holds the neutral (expr=0) mesh; each expression key
is `Basis + delta[i]`. The keys are named with the FACS-style semantic
names from Meta's MHR docs: `browLowerer_L`, `jawDrop`, `cheekPuff_R`,
`lipCornerPuller_L`, etc. (Full list in
[nodes/sam_3d_body/mhr_face_expression_names.py](../nodes/sam_3d_body/mhr_face_expression_names.py).)

Each key is set to its source-image coefficient so the imported mesh looks
identical to a non-shape-keyed export. Setting a shape key value to zero
returns that basis to neutral; sculpt/edit each independently.

The keys are baked in the same coordinate system as the mesh (MHR → OBJ
`(x, -y, -z)` flip, then Blender OBJ import's `(x, -z, y)` — net effect
BAKE_T for both mesh and keys). The bake_facing rotation and offset, if
any, are also applied to the shape-key deltas so they line up with the
rotated mesh.

**Implementation**: `_compute_face_shape_key_data` (main process) +
`_apply_face_shape_keys_from_json` (inside the bpy venv). Adds one MHR
forward pass with 73 batch rows per exported character.

The Blender addon has an operator that copies these shape keys onto a
different mesh (different topology allowed, via Surface Deform bake) —
see the [addon docs](sam3d_pose_copy_addon.md#face-expression-shape-key-transfer).

## `SAM3DBodyExportTwoCharactersFBX` — two-character node

Dedicated node for the common case: one image containing two characters,
a mask isolating character A, and optionally a tight mask for character B.

Node runs the SAM3D pipeline twice (once with mask A, once with the
inverted or explicit mask B), then exports each result to its own FBX.

Key details:

- If `mask_b` is provided, uses it directly — much more reliable than
  inverting mask A, since an inverted A covers most of the image and can
  let the detector re-pick character A for both outputs.
- `preserve_scene_positions` (default on) shifts each character's
  vertices/joints by its `pred_cam_t` before export, so the two characters
  land in the same spatial relationship they had in the original image.
- `bake_facing` is computed **once from character A** and applied to
  **both** characters, so A ends up at the canonical origin and B lands
  at its correct offset relative to A. Character-A's landmarks anchor the
  shared frame, character B piggybacks — spatial coherence between them
  is preserved through the transform.
- `bake_face_shape_keys` applies to both characters if enabled.

Outputs two FBX paths (`fbx_path_a`, `fbx_path_b`).

**Implementation**: `SAM3DBodyExportTwoCharactersFBX.execute` in
`nodes/export.py`. Stashes the shared `_bake_transform` (rotation +
offset) on each character's mesh_data dict so downstream shape-key baking
inside `SAM3DBodyExportFBX` can apply the same transform to its base +
basis meshes (instead of re-deriving from each character's own landmarks,
which would break the shared alignment).

## `SAM3DBodyFaceExpression` — inspect node

New processing node in `nodes/process.py`. Inputs a processed mesh_data,
outputs:

- **expression** (custom SAM3D_EXPRESSION type) — a dict with `values`
  (numpy array of 72 coefficients), `num_coefficients`, and `names`
  (list of FACS-style semantic names).
- **summary_json** (string) — human-readable JSON listing the top-K
  most-active coefficients by absolute magnitude, each with its
  numeric index, semantic name, and value. Useful for debugging what
  expressions the model reconstructed from the source image.

Semantic names come from `MHR_FACE_EXPRESSION_NAMES` — Meta's published
FACS-style labels for the 72 v1.x face-expression blendshapes
(see the [upstream MHR docs](https://github.com/facebookresearch/MHR/blob/main/docs/face-expressions.md)).
The shipped TorchScript model doesn't embed these names; they come from
the docs and are index-stable across MHR LODs 0-6.

## Torso roll alignment (already in the base)

For context: the base exporter already aligns the roll of torso-chain
bones (`world, root, c_spine0..3, c_neck, c_head`) using landmark
positions, plus a head-specific eye-anatomy override so head-turn
transfers cleanly. This lives in `_align_torso_rolls_from_landmarks`.

`align_all_rolls` (new) extends this pattern to the limbs.

## Coordinate conventions cheat sheet

Useful when debugging math changes in the export pipeline:

- **MHR internal space**: MHR joints and vertices come out in MHR's own
  coordinate system.
- **`BAKE_T` matrix** (in `export.py`): `[[1,0,0],[0,0,1],[0,-1,0]]`.
  Maps MHR → Blender via `(x, y, z) → (x, z, -y)`. Used inside
  `_apply_bake_facing` and the yaw/full transform computations.
- **OBJ round-trip flip**: `_write_obj_file` writes `(x, -y, -z)`; Blender's
  OBJ importer then applies `(x, -z, y)`. The composition equals `BAKE_T`,
  so mesh and skeleton end up in the same Blender space.
- **After `bake_facing=yaw` or `full`**: character faces Blender world
  `-Y`, up is world `+Z`, character-right is world `-X` (character faces
  the viewer with their right on the viewer's left).
- **After `align_all_rolls`**: limb bones have local Z pointing along the
  character-forward direction (matches CloudRig/Rigify convention for
  main-body bones).
