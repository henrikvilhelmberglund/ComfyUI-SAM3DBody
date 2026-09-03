# SAM3D Pose Copy — Blender Addon

Blender addon that retargets a posed MHR (Meta Human Rig) skeleton — such as
one exported from the SAM 3D Body pipeline in this repo — onto an arbitrary
target rig (Rigify, CloudRig Sintel, custom character rigs, etc.).

Lives in [tools/sam3d_pose_copy/](../tools/sam3d_pose_copy/) and ships as a
zip in that same folder for `Edit → Preferences → Add-ons → Install…`.

## Table of contents

- [What problem it solves](#what-problem-it-solves)
- [Installation](#installation)
- [The workflow, top to bottom](#the-workflow-top-to-bottom)
- [Character profiles (tabs)](#character-profiles-tabs)
- [Mapping list — per-row anatomy](#mapping-list--per-row-anatomy)
- [Modes](#modes)
- [Axis dropdowns](#axis-dropdowns)
- [Rotation offset](#rotation-offset)
- [IK pole targets](#ik-pole-targets)
- [Presets](#presets)
- [Face expression shape-key transfer](#face-expression-shape-key-transfer)
- [Diagnose — what the numbers mean](#diagnose--what-the-numbers-mean)
- [Copy options](#copy-options)
- [Common failure modes and fixes](#common-failure-modes-and-fixes)

## What problem it solves

The SAM 3D Body pipeline produces an MHR-topology armature (127 bones,
Meta's own naming: `root, c_spine0..3, l_upleg, l_lowleg, l_foot, l_hand,
l_thumb0..3, c_head, ...`). To use those poses in production you want them
on a rig you actually animate with — Rigify, CloudRig, an in-house character
rig — which has its own bone names, its own axis conventions, and its own
IK/FK control layout.

This addon does that pose transfer: name mapping, axis-convention handling,
IK end-effector positioning with chain-length scaling, and per-bone rotation
modes chosen for what actually gives good results on each body part.

## Installation

1. In Blender: `Edit → Preferences → Add-ons → Install…`.
2. Pick [tools/sam3d_pose_copy.zip](../tools/sam3d_pose_copy.zip).
3. Enable the "SAM3D Pose Copy" checkbox.
4. Panel appears in the 3D viewport sidebar under the **SAM3D** tab
   (press `N` to open the sidebar).

## The workflow, top to bottom

The panel's "Workflow" box walks through the steps in order:

1. **Auto-match / Fill from source** — populate the mapping table by
   matching MHR bone names against the target armature's names.
2. **Align facing / Flip 180° / Apply master yaw** — orient the source
   armature so its character faces the same direction as the target. Often
   unnecessary if the FBX was exported with `bake_facing=yaw`.
3. **Scale to target** — uniformly scale the source armature so its leg
   length matches the target's. Skips if source and target are already
   similar in size.
4. **Auto-set modes** — analyses rest-pose axis angles per row and picks a
   sensible mode (FULL / AIM / DELTA / POS) based on the axis mismatch.
5. **Clear IK rotation offsets** — zero out the `rotation_offset` field on
   POS rows, in case earlier tweaking left stale offsets.
6. **Copy all enabled** — apply all rows in order and produce the final pose.

Or just use a **preset** (Rigify or CloudRig Sintel) and skip step 1-4.

## Character profiles (tabs)

The top row of the panel shows tabs, one per character profile. Each profile
carries its own:

- Source armature (typically the SAM3D FBX)
- Target armature (Rigify/CloudRig/whatever)
- Mapping list, rotation offsets, per-row axis dropdowns
- Source FBX path (for the "Reimport source" button)
- IK pole targets
- Spine bend amplify slider
- Face shape-key source/target meshes

Use profiles when retargeting multiple characters from the same source
image (the two-character export from the ComfyUI side is the typical
use case) — one profile per character, keeping their mappings independent.

Copy/paste mapping rows between profiles via the **Copy** / **Paste**
buttons on the profile-tab row.

## Mapping list — per-row anatomy

Each row: `[✓] [source bone] → [target bone] [mode] [src-axis] [tgt-axis] [▶]`

- **✓** — enabled toggle
- **source bone** — the MHR bone name (validated against the source
  armature; red-icon if missing)
- **target bone** — the target rig's bone name
- **mode** — see [Modes](#modes) below
- **src-axis / tgt-axis** — see [Axis dropdowns](#axis-dropdowns) below.
  Greyed out for modes where axes don't apply (only AIM / AIM_ROLL use them).
- **▶** — apply just this one row (for testing without a full recopy)

Selecting a row exposes the **Rotation offset** field below the list (Euler
XYZ, bone-local frame, degrees).

## Modes

| Mode | Best for | What it does |
| --- | --- | --- |
| **FULL** | FK controls where source and target axes align | Copies source's world rotation, position follows parent chain. |
| **AIM** | Bones where only "point Y at source Y" matters, roll can float | Rotates target so its Y-axis aligns with source's Y-axis; leaves twist around Y unset. |
| **AIM_ROLL** | Limbs (arms, legs, clavicles) — the workhorse mode | AIM plus twist match around Y: source's X-axis projected perpendicular to Y aligns to target's. Convention-agnostic and reliable. |
| **DELTA** | Torso (spine, hip, chest) | Applies source's rotation relative to a **fixed character-forward canonical** (X = char-right, Y = up, Z = char-back). Works cleanly for torso because torso bones already live in that convention. |
| **FINGER** | Fingers | Palm-normal anatomical frame transfer — same technique as hand-IK. Convention-invariant, carries curl properly. Only meaningful for source bones that MHR recognises as fingers (`l_thumb0..3`, `l_index1..3`, etc.). |
| **POS** | IK end effectors (`IK-Hand.L/R`, `IK-Foot.L/R`) | Sets the target's world position to a chain-scaled projection of source's wrist/ankle direction. Rotation uses an anatomical frame from finger/toe landmarks. |
| **POS_RAW** | IK end effectors when source/target proportions match | Same as POS but skips chain scaling — pins the IK to the source's raw wrist/ankle world position. Use when POS lands "slightly too high" and you don't want the proportional adjustment. |
| **SKIP** | Bones you want to preserve across re-copies | Do not touch this bone. Combine with `reset_before_copy` for manual overrides that survive re-copies. |

## Axis dropdowns

Two per-row dropdowns: **src-axis** and **tgt-axis**, with values
`+X, +Y, +Z, -X, -Y, -Z`.

Default is `+Y / +Y` — natural along-bone aim (each bone's Y axis is the
head-to-tail direction in Blender). Change these when the two rigs use
opposite conventions on some bone.

Typical cases:

- **`+Y → -Y`** — 180° flip around a horizontal axis. Fixes "correct tilt
  axis but reversed sign" issues. Example: Sintel `FK-Neck` needs `-Y`
  because MHR neck aims one side, Sintel aims the other.
- **`+Y → +X`** or **`+Z`** — 90° remap for rigs whose bones are set up on
  a different axis entirely (rare).

The Auto-set modes button also flips a row's `tgt-axis` to `-Y` when it
detects opposite Z conventions between source and target rest poses.

## Rotation offset

Free-form Euler-XYZ offset in the bone's local frame, applied as a post
rotation after the mode's computation. Use for small residual corrections
the axis dropdowns can't express — a few degrees here or there.

## IK pole targets

Below the mapping list, a section lets you pick per-side elbow / knee pole
bones on the target rig. When set, "Copy all enabled" also positions those
pole targets by computing a point in the elbow/knee bend plane at
`pole_distance` metres from the joint.

Without pole targets, the target's IK solver picks arbitrary bend directions
and arms/legs can bend backwards.

## Presets

Two built-in presets, loadable via one click:

- **Load Rigify preset** — the built-in Blender Rigify Human meta-rig
  naming (`spine_fk`, `hand_ik.L`, `f_middle.01.L`, `foot_ik.L`, etc.).
- **Load Sintel (CloudRig) preset** — CloudRig Sintel-sample naming
  (`TORSO-Spine`, `FK-Chest`, `FK-Shoulder.L`, `IK-Hand.L`, `Finger_Middle1.L`,
  `POLE-Arm.L`, etc.). Includes the `-Y` target-axis override on `c_neck`.

Both presets:

- Set all rows to the right mode for each body part (torso DELTA, limbs
  AIM_ROLL, fingers FINGER, IK POS).
- Populate the IK pole target names.
- Skip toe rows (SAM3D toe reconstruction is essentially noise).

Custom presets: after tuning a mapping, click **Save preset** to a JSON
file, and reuse via **Load preset**. The JSON carries source/target names,
modes, axes, and rotation offsets.

## Face expression shape-key transfer

For characters exported with `bake_face_shape_keys=True` on the ComfyUI
side (see [exporter_changes.md](exporter_changes.md)), the imported mesh
carries 72 named shape keys — `browLowerer_L`, `jawDrop`, `lipCornerPuller_R`,
etc., the FACS-style names from Meta's MHR docs.

The addon can copy those shape keys onto your target character's face mesh
even when the two meshes have different topology:

1. In the profile panel's "Face expression shape keys" section, pick the
   source mesh (SAM3D FBX character mesh, with the shape keys) and the
   target mesh (your character's face mesh).
2. Click **Transfer to target mesh**.

Under the hood: a temporary Surface Deform modifier binds target-to-source;
each source shape key is activated in turn, the deformed target vertex
positions are captured, and a matching-name shape key is added to the target.
Modifier is removed afterwards.

Then you can sculpt/modify each blendshape independently on your target
character.

## Diagnose — what the numbers mean

**Diagnose rest-pose alignment** button prints per-row diagnostics to the
system console. Columns:

- **Y°** — angle between source Y-axis and target Y-axis in world (rest
  pose). Big means the bones don't point the same direction at rest.
- **Z°** — angle between source Z (roll) and target Z. Big means opposite
  roll conventions.
- **drift** — distance in cm between source's current-pose head position
  and target's current-pose head position, after the last copy. Useful for
  POS rows to detect chain-scale problems.
- **suggestion** — actionable hint per row (or "OK").

Interpretation heuristics:

- Big Y° on a FULL row → switch to AIM_ROLL.
- Big Z° on a FULL row → switch to DELTA (torso) or AIM_ROLL (limb).
- Big Y/Z° on DELTA torso → OK; DELTA uses a fixed canonical, not the rest
  axes.
- Big Y/Z° on AIM_ROLL / AIM / FINGER → OK; these modes are convention-
  agnostic.
- Big drift on POS → check the source armature's scale/facing; possibly
  try POS_RAW if source and target have similar proportions.

## Copy options

- **Reset pose first** — clear target pose to identity before applying rows.
  Default on. Turn off if you want to layer copies without losing prior
  manual adjustments (bones on SKIP rows are always preserved).
- **Position only** — force POS mode globally.
- **Live update** — re-run the full copy on every row edit (mode, axis,
  bone-name change). Turn off for large presets if the panel feels sluggish.
- **Spine bend amplify** — multiplier on the torso DELTA rotation delta.
  Bump above 1.0 if the target's rest curvature dampens the source's
  spine bend.

## Common failure modes and fixes

- **All bones show "Y axis 180° off" in diagnose** — the source armature
  is oriented 180° from the target. Run "Align facing" or re-export with
  `bake_facing=yaw` on the ComfyUI side.
- **Head faces backward** — set the head row's target-axis to `-Y`.
- **Fingers stay straight** — mode must be FINGER, not AIM/AIM_ROLL.
- **IK hand/foot floats far in front of character** — source armature has
  been translated in Object mode (often by Scale to target). Reset with
  `Alt+G` or move it manually to align with the target.
- **IK hand/foot drift 40+ cm** — arm/leg length mismatch after leg-based
  scale-to-target. Try POS_RAW (skips chain scaling), or accept the
  proportional placement.
- **Face rig upside-down + backward** — apply target-axis `-Y` on the head
  row (or on both head and neck). If still wrong, try mode = AIM (drops
  twist match, which is often the culprit).
