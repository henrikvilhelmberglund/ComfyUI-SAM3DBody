# Preset reference: Rigify (standard)

Target: vanilla Blender Rigify Human rig, generated from
`Add → Armature → Human Meta-Rig → Rigify Buttons → Generate Rig`.

Loaded in the addon via the **"Rigify (standard)"** button.

Source constant: `RIGIFY_STANDARD_MAPPING` in
[tools/sam3d_pose_copy/__init__.py](../tools/sam3d_pose_copy/__init__.py).

## IK pole targets

| Slot | Target bone |
| --- | --- |
| L elbow | `upper_arm_ik_target.L` |
| R elbow | `upper_arm_ik_target.R` |
| L knee | `thigh_ik_target.L` |
| R knee | `thigh_ik_target.R` |

## Mapping table

Columns:

- **MHR source** — MHR bone the pose is read from.
- **Target** — Rigify control bone the pose is applied to.
- **Mode** — see [addon docs](sam3d_pose_copy_addon.md#modes) for full descriptions.
- **Axes** — source-axis / target-axis dropdowns. `+Y / +Y` (blank in the
  table below) is the default. Only listed when non-default.

### Torso — DELTA

Torso bones live in a Y-up canonical that matches DELTA's fixed
"character-forward" reference frame.

| MHR source | Target | Mode | Axes | Notes |
| --- | --- | --- | --- | --- |
| `world` | `root` | POS | | Ground master — anchors character to origin |
| `root` | `spine_fk` | DELTA | | MHR `root` = pelvis; Rigify's `spine_fk` is at the pelvis |
| `c_spine0` | `spine_fk.001` | DELTA | | |
| `c_spine1` | `spine_fk.002` | DELTA | | |
| `c_spine2` | `spine_fk.003` | DELTA | | |
| `c_spine3` | `chest` | DELTA | | Top of Rigify FK spine chain |

### Neck / head / face — DELTA

| MHR source | Target | Mode | Axes | Notes |
| --- | --- | --- | --- | --- |
| `c_neck` | `neck` | DELTA | | |
| `c_head` | `head` | DELTA | | |
| `c_jaw` | `jaw_master` | DELTA | | |
| `l_eye` | `eye.L` | DELTA | | |
| `r_eye` | `eye.R` | DELTA | | |

### Left arm — AIM_ROLL limbs + POS IK target

| MHR source | Target | Mode | Axes | Notes |
| --- | --- | --- | --- | --- |
| `l_clavicle` | `shoulder.L` | AIM_ROLL | | |
| `l_uparm` | `upper_arm_fk.L` | AIM_ROLL | | |
| `l_lowarm` | `forearm_fk.L` | AIM_ROLL | | |
| `l_wrist` | `hand_ik.L` | POS | | IK end effector — chain-scaled position + anatomical rotation |
| `l_wrist` | `hand_fk.L` | AIM_ROLL | | FK backup — same source drives both IK and FK controls |

### Right arm

| MHR source | Target | Mode | Axes | Notes |
| --- | --- | --- | --- | --- |
| `r_clavicle` | `shoulder.R` | AIM_ROLL | | |
| `r_uparm` | `upper_arm_fk.R` | AIM_ROLL | | |
| `r_lowarm` | `forearm_fk.R` | AIM_ROLL | | |
| `r_wrist` | `hand_ik.R` | POS | | |
| `r_wrist` | `hand_fk.R` | AIM_ROLL | | |

### Left leg

| MHR source | Target | Mode | Axes | Notes |
| --- | --- | --- | --- | --- |
| `l_upleg` | `thigh_fk.L` | AIM_ROLL | | |
| `l_lowleg` | `shin_fk.L` | AIM_ROLL | | |
| `l_subtalar` | `foot_ik.L` | POS | | Subtalar (below-ankle) gives correct heel height for the IK target |
| `l_ball` | `foot_fk.L` | AIM_ROLL | | `l_ball` (base of toes) gives a forward-pointing Y that matches Rigify's `foot_fk` Y — MHR `l_foot` points at subtalar via a tail override, wrong direction |
| `l_ball` | `toe_fk.L` | SKIP | | SAM3D doesn't reconstruct meaningful toe motion — leave at rest |

### Right leg

| MHR source | Target | Mode | Axes | Notes |
| --- | --- | --- | --- | --- |
| `r_upleg` | `thigh_fk.R` | AIM_ROLL | | |
| `r_lowleg` | `shin_fk.R` | AIM_ROLL | | |
| `r_subtalar` | `foot_ik.R` | POS | | |
| `r_ball` | `foot_fk.R` | AIM_ROLL | | |
| `r_ball` | `toe_fk.R` | SKIP | | |

### Fingers — FINGER (palm-normal anatomical transfer)

MHR tracks 3 phalanges per non-thumb finger, plus a carpal joint on the
pinky (`l_pinky0`). Rigify's `f_XXX.03.L` is the last phalanx / fingertip
anchor — there is no 4th phalanx to map. Pinky carpal has no Rigify
equivalent.

| MHR source | Target | Mode | Axes | Notes |
| --- | --- | --- | --- | --- |
| `l_thumb0` | `thumb.01.L` | FINGER | | MHR thumb0 = carpometacarpal joint |
| `l_thumb1` | `thumb.02.L` | FINGER | | |
| `l_thumb2` | `thumb.03.L` | FINGER | | |
| `l_index1` | `f_index.01.L` | FINGER | | |
| `l_index2` | `f_index.02.L` | FINGER | | |
| `l_index3` | `f_index.03.L` | FINGER | | |
| `l_middle1` | `f_middle.01.L` | FINGER | | |
| `l_middle2` | `f_middle.02.L` | FINGER | | |
| `l_middle3` | `f_middle.03.L` | FINGER | | |
| `l_ring1` | `f_ring.01.L` | FINGER | | |
| `l_ring2` | `f_ring.02.L` | FINGER | | |
| `l_ring3` | `f_ring.03.L` | FINGER | | |
| `l_pinky1` | `f_pinky.01.L` | FINGER | | Note: MHR `l_pinky0` (carpal) has no Rigify target |
| `l_pinky2` | `f_pinky.02.L` | FINGER | | |
| `l_pinky3` | `f_pinky.03.L` | FINGER | | |

Right side mirrors left: `r_thumb0..3 → thumb.01..03.R`,
`r_index1..3 → f_index.01..03.R`, etc.

## Building your own preset from scratch

If you're targeting a different Rigify variant or a completely new
skeleton, use this preset as a template:

1. **Start with the mode conventions** — DELTA for torso, AIM_ROLL for
   limbs, FINGER for fingers, POS for IK end effectors. These are
   convention-agnostic enough that they usually work regardless of the
   target rig's specific axis choices.
2. **Fill in target bone names** from your rig's control bones (use
   the "Dump target bones" button in the addon panel).
3. **Set the IK pole targets** by picking the elbow/knee pole controls
   from the dropdown in the "IK pole targets" panel section.
4. **Run diagnose** — see which rows show large angle mismatches. Torso
   rows are OK regardless (DELTA is canonical-based); limb/finger rows on
   AIM_ROLL/FINGER are also OK regardless.
5. **Fix per-row issues** — usually one of:
   - Head/neck facing wrong: set target-axis to `-Y`
   - IK too high/low: switch POS → POS_RAW to skip chain scaling
   - Foot pointing wrong direction: change source from `l_foot` to `l_ball`

## Common pitfalls specific to Rigify

- **`toe.L/R` doesn't exist** on a vanilla generated rig — the toe bone is
  `toe_fk.L/R` (split-toe) or `toe.L/R` only if the metarig had non-split
  toes at generation time. The "Rigify (edit)" preset has the old
  incorrect name; the "Rigify (standard)" preset has it right.
- **`foot_ik.L` position** works via the addon's chain-scaled IK, which
  uses source's subtalar as the anchor. Ankle (`l_foot`) would put the IK
  target at the ankle joint rather than under the heel.
- **Rigify's `spine_fk`** is at the PELVIS, not the middle of the spine —
  so MHR `root` (pelvis joint) maps to it, and `c_spine0..3` map to
  `spine_fk.001..003 / chest` (the four spine bones above).
