# Preset reference: Sintel (CloudRig)

Target: a rig generated from the CloudRig Sintel metarig sample, or any
rig that uses the same `FK-*` / `IK-*` / `DEF-*` / `TORSO-*` naming
convention.

Loaded in the addon via the **"Sintel (CloudRig)"** button.

Source constant: `SINTEL_DEFAULT_MAPPING` in
[tools/sam3d_pose_copy/__init__.py](../tools/sam3d_pose_copy/__init__.py).

## IK pole targets

| Slot | Target bone |
| --- | --- |
| L elbow | `POLE-Arm.L` |
| R elbow | `POLE-Arm.R` |
| L knee | `POLE-Leg.L` |
| R knee | `POLE-Leg.R` |

## Mapping table

Columns:

- **MHR source** — MHR bone the pose is read from.
- **Target** — CloudRig Sintel control bone the pose is applied to.
- **Mode** — see [addon docs](sam3d_pose_copy_addon.md#modes).
- **Axes** — source-axis / target-axis dropdowns. `+Y / +Y` (blank) is
  the default; only listed when non-default.

### Torso — DELTA

CloudRig Sintel exposes `TORSO-Spine` as a top-level torso translation +
rotation control (analogous to Rigify's `torso`). MHR `root` (pelvis)
maps to it. The FK spine chain has only 3 tiers (`FK-Hips`/`FK-Spine`/
`FK-Chest`) so MHR's 4 spine bones can't all map — the middle two
(`c_spine1`, `c_spine2`) are SKIP by default.

| MHR source | Target | Mode | Axes | Notes |
| --- | --- | --- | --- | --- |
| `world` | `root` | POS | | Ground master — anchors character |
| `root` | `TORSO-Spine` | DELTA | | Top-level torso control (position + rotation) |
| `c_spine0` | `FK-Spine` | DELTA | | Lower spine |
| `c_spine1` | *(empty)* | SKIP | | No target — Sintel FK spine is 3-tier |
| `c_spine2` | *(empty)* | SKIP | | See above; bump "Spine bend amplify" to compensate |
| `c_spine3` | `FK-Chest` | DELTA | | Upper spine / chest |

### Neck / head / face — AIM (not AIM_ROLL)

Plain AIM for head/neck because AIM_ROLL's twist match picks the wrong
sign of rotation around Y when source/target X-axes are on opposite
sides of Y (which happens with Sintel FK-Head vs MHR c_head). Result
without the fix: face upside down + backward. AIM carries tilt/nod but
skips yaw/head-turn transfer.

The `-Y` target axis on `c_neck` fixes a specific Sintel neck-rest
convention where the target Y aims one direction and MHR aims the other.

| MHR source | Target | Mode | Axes | Notes |
| --- | --- | --- | --- | --- |
| `c_neck` | `FK-Neck` | AIM | `+Y / -Y` | Target-axis `-Y` flip for Sintel convention |
| `c_head` | `FK-Head` | AIM | | |
| `c_jaw` | `Jaw` | AIM | | |
| `l_eye` | `Eye.L` | AIM | | |
| `r_eye` | `Eye.R` | AIM | | |

### Left arm — AIM_ROLL limbs + POS IK target

| MHR source | Target | Mode | Axes | Notes |
| --- | --- | --- | --- | --- |
| `l_clavicle` | `FK-Shoulder.L` | AIM_ROLL | | |
| `l_uparm` | `FK-UpperArm.L` | AIM_ROLL | | |
| `l_lowarm` | `FK-Forearm.L` | AIM_ROLL | | |
| `l_wrist` | `IK-Hand.L` | POS | | IK end effector — chain-scaled |
| `l_wrist` | `FK-Hand.L` | AIM_ROLL | | FK backup |

### Right arm

| MHR source | Target | Mode | Axes | Notes |
| --- | --- | --- | --- | --- |
| `r_clavicle` | `FK-Shoulder.R` | AIM_ROLL | | |
| `r_uparm` | `FK-UpperArm.R` | AIM_ROLL | | |
| `r_lowarm` | `FK-Forearm.R` | AIM_ROLL | | |
| `r_wrist` | `IK-Hand.R` | POS | | |
| `r_wrist` | `FK-Hand.R` | AIM_ROLL | | |

### Left leg

| MHR source | Target | Mode | Axes | Notes |
| --- | --- | --- | --- | --- |
| `l_upleg` | `FK-Thigh.L` | AIM_ROLL | | |
| `l_lowleg` | `FK-Knee.L` | AIM_ROLL | | Sintel names the shin `Knee` |
| `l_subtalar` | `IK-Foot.L` | POS | | Subtalar gives correct heel height |
| `l_ball` | `FK-Foot.L` | AIM_ROLL | | `l_ball` for forward-pointing Y (matches Sintel foot convention) |
| `l_ball` | `FK-Toes.L` | SKIP | | SAM3D doesn't reconstruct toe motion |

### Right leg

| MHR source | Target | Mode | Axes | Notes |
| --- | --- | --- | --- | --- |
| `r_upleg` | `FK-Thigh.R` | AIM_ROLL | | |
| `r_lowleg` | `FK-Knee.R` | AIM_ROLL | | |
| `r_subtalar` | `IK-Foot.R` | POS | | |
| `r_ball` | `FK-Foot.R` | AIM_ROLL | | |
| `r_ball` | `FK-Toes.R` | SKIP | | |

### Fingers — FINGER (palm-normal anatomical transfer)

Sintel has 4 phalanges per finger; MHR only tracks 3 for non-thumb
fingers, so `Finger_XN4` targets stay unmapped for index/middle/ring.
Thumb has 4 in both (`thumb0..3 → Thumb1..4`). Pinky has a carpal
(`l_pinky0`) mapped to `Finger_Pinky_Carpal`.

| MHR source | Target | Mode | Axes | Notes |
| --- | --- | --- | --- | --- |
| `l_thumb0` | `Finger_Thumb1.L` | FINGER | | |
| `l_thumb1` | `Finger_Thumb2.L` | FINGER | | |
| `l_thumb2` | `Finger_Thumb3.L` | FINGER | | |
| `l_thumb3` | `Finger_Thumb4.L` | FINGER | | 4th thumb phalanx |
| `l_index1` | `Finger_Index1.L` | FINGER | | (Sintel `Finger_Index4.L` unmapped) |
| `l_index2` | `Finger_Index2.L` | FINGER | | |
| `l_index3` | `Finger_Index3.L` | FINGER | | |
| `l_middle1..3` | `Finger_Middle1..3.L` | FINGER | | Same 3-phalanx pattern |
| `l_ring1..3` | `Finger_Ring1..3.L` | FINGER | | |
| `l_pinky0` | `Finger_Pinky_Carpal.L` | FINGER | | MHR pinky is the only non-thumb finger with a mapped carpal |
| `l_pinky1..3` | `Finger_Pinky1..3.L` | FINGER | | |

Right side mirrors left.

## Building your own preset from scratch (using this as a template)

CloudRig / Sintel has some unusual conventions worth knowing about:

- **`TORSO-Spine`** is the master translation control (like Rigify `torso`).
  It's the target for MHR `root` (pelvis).
- **Sintel names the shin "Knee"** — `FK-Knee.L` is what you'd normally
  call `FK-Shin.L`.
- **Sintel FK spine is 3-tier** (`Hips` / `Spine` / `Chest`) vs. MHR's
  4-tier. Bump the **"Spine bend amplify"** slider if the target
  underbends. Or add intermediate mappings to helper bones like
  `STR-Chest1` for more granularity.
- **CloudRig's face rest orientations can flip signs** compared to MHR.
  Any face bone that ends up upside-down or backward likely needs a
  `-Y` on either the source or target axis dropdown. `c_neck` is the
  known case in this preset.

For a rig with different naming, use this preset as a template:

1. Dump target rig control bone names (the "Dump target bones" panel
   button; filter out DEF-/MCH-/ORG- internals).
2. Fill in target names in the mapping table; keep the modes as-is.
3. Set IK pole targets from your rig's pole bone names.
4. Run diagnose — investigate any rows that fail with high angles on
   FULL/DELTA modes (fine on AIM/AIM_ROLL/FINGER/POS regardless).
5. For any body part where the target rig's rest orientation flips
   from MHR's, set the row's `Tgt axis` dropdown to `-Y` (or the axis
   that matches your rig).
