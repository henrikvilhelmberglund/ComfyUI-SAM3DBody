# Preset reference: Rigify (edit)

Legacy preset — kept for backwards compatibility with the older mode
assignment scheme (blanket FULL for most rows, DELTA only for `root`).
Also has `toe.L/R` as leg endpoints, which is a Rigify metarig
non-split-toe naming that doesn't match a vanilla generated rig.

**For a vanilla Blender Rigify Human rig, prefer the
[(standard) preset](preset_rigify_standard.md).** This preset is retained
only for cases where you have an older setup that expected these specific
mode/name assignments.

Loaded in the addon via the **"Rigify (edit)"** button.

Source constant: `RIGIFY_EDIT_MAPPING` in
[tools/sam3d_pose_copy/__init__.py](../tools/sam3d_pose_copy/__init__.py).

## IK pole targets

| Slot | Target bone |
| --- | --- |
| L elbow | `upper_arm_ik_target.L` |
| R elbow | `upper_arm_ik_target.R` |
| L knee | `thigh_ik_target.L` |
| R knee | `thigh_ik_target.R` |

## Mapping table

Mode is assigned at load time as:
- **DELTA** if the target bone name is `root`, `root.001`, `master`, or `world`
- **FULL** for everything else

Axes are always default (`+Y / +Y`).

### Torso

| MHR source | Target | Mode | Notes |
| --- | --- | --- | --- |
| `world` | `root` | DELTA | Ground master |
| `root` | `spine_fk` | FULL | Pelvis |
| `c_spine0` | `spine_fk.001` | FULL | |
| `c_spine1` | `spine_fk.002` | FULL | |
| `c_spine2` | `spine_fk.003` | FULL | |
| `c_spine3` | `chest` | FULL | |

### Neck / head / face

| MHR source | Target | Mode |
| --- | --- | --- |
| `c_neck` | `neck` | FULL |
| `c_head` | `head` | FULL |
| `c_jaw` | `jaw_master` | FULL |
| `l_eye` | `eye.L` | FULL |
| `r_eye` | `eye.R` | FULL |

### Left arm

| MHR source | Target | Mode |
| --- | --- | --- |
| `l_clavicle` | `shoulder.L` | FULL |
| `l_uparm` | `upper_arm_fk.L` | FULL |
| `l_lowarm` | `forearm_fk.L` | FULL |
| `l_wrist` | `hand_ik.L` | FULL |
| `l_wrist` | `hand_fk.L` | FULL |

### Right arm

| MHR source | Target | Mode |
| --- | --- | --- |
| `r_clavicle` | `shoulder.R` | FULL |
| `r_uparm` | `upper_arm_fk.R` | FULL |
| `r_lowarm` | `forearm_fk.R` | FULL |
| `r_wrist` | `hand_ik.R` | FULL |
| `r_wrist` | `hand_fk.R` | FULL |

### Left leg

| MHR source | Target | Mode | Notes |
| --- | --- | --- | --- |
| `l_upleg` | `thigh_fk.L` | FULL | |
| `l_lowleg` | `shin_fk.L` | FULL | |
| `l_subtalar` | `foot_ik.L` | FULL | |
| `l_foot` | `foot_fk.L` | FULL | |
| `l_ball` | `toe.L` | FULL | Target name `toe.L` only exists on non-split-toe metarigs — vanilla Rigify generates `toe_fk.L/toe_ik.L` |

### Right leg

| MHR source | Target | Mode | Notes |
| --- | --- | --- | --- |
| `r_upleg` | `thigh_fk.R` | FULL | |
| `r_lowleg` | `shin_fk.R` | FULL | |
| `r_subtalar` | `foot_ik.R` | FULL | |
| `r_foot` | `foot_fk.R` | FULL | |
| `r_ball` | `toe.R` | FULL | See left-side note |

### Fingers

All FULL mode, all default axes.

| MHR source | Target |
| --- | --- |
| `l_thumb1` | `thumb.01.L` |
| `l_thumb2` | `thumb.02.L` |
| `l_thumb3` | `thumb.03.L` |
| `l_index1..3` | `f_index.01..03.L` |
| `l_middle1..3` | `f_middle.01..03.L` |
| `l_ring1..3` | `f_ring.01..03.L` |
| `l_pinky1..3` | `f_pinky.01..03.L` |

Right side mirrors left.

Note: unlike the standard preset, thumb here starts at `l_thumb1` (MCP
joint) not `l_thumb0` (carpometacarpal), so the thumb base rotation isn't
transferred.

## Migration to the standard preset

If you've been using this preset and want to move to the standard one:

1. Click "Rigify (standard)" to load the new preset (replaces the mapping list).
2. If you had per-row rotation offsets or SKIP overrides, you'll need to
   reapply them — mode changes may make some old offsets obsolete anyway.
3. Test: modes are now DELTA/AIM_ROLL/FINGER/POS per body part instead of
   blanket FULL. Head/neck rows on DELTA may need `-Y` target axis if your
   character shows head-facing sign flip.
