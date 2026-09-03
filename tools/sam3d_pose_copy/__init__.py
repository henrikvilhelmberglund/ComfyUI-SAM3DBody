bl_info = {
    "name": "SAM3D Pose Copy",
    "author": "ComfyUI-SAM3DBody",
    "version": (4, 7, 0),
    "blender": (3, 0, 0),
    "location": "View3D > Sidebar > SAM3D",
    "description": "Interactively map bones from a SAM3D-Body MHR posed skeleton onto any target armature.",
    "category": "Animation",
}

import json
import os

import bpy
from mathutils import Matrix, Quaternion
from bpy.props import (
    BoolProperty,
    CollectionProperty,
    IntProperty,
    PointerProperty,
    StringProperty,
)
from bpy.types import Operator, Panel, PropertyGroup, UIList
from bpy_extras.io_utils import ExportHelper, ImportHelper


# For each MHR bone, an ordered list of common target-bone-name candidates.
# Order matters — first hit wins in auto-match. Plain names first, then
# Rigify (fk/ik), then Mixamo, then Unreal-style, then anything else.
MHR_TARGET_SYNONYMS = {
    # MHR joint 0 — the virtual world/ground root pymomentum inserts above
    # the FBX rig. Maps to the target rig's ground master control.
    "world":      ["root", "root.001", "Root", "master", "MASTER", "world"],

    # MHR joint 1 "root" is the *pelvis*.
    # Deliberately NOT matching "hip" or literal "root": in many rigs "hip" is
    # a lower-body-only belt control (which yanks the whole character weird
    # when full-copied), and "root" is the ground master (handled by "world").
    # Preferring torso / spine-base names.
    "root":       ["torso", "hips", "pelvis", "Hips", "Pelvis", "spine_fk", "spine.001", "spine", "Spine", "spine_01", "pelvis_l"],
    "c_spine0":   ["spine.001", "spine_fk.001", "spine.002", "Spine1", "spine_01", "spine_02"],
    "c_spine1":   ["spine.002", "spine_fk.002", "spine.003", "Spine2", "spine_02", "spine_03"],
    "c_spine2":   ["spine.003", "spine_fk.003", "spine.004", "Spine3", "spine_03", "spine_04"],
    "c_spine3":   ["chest", "spine.004", "spine_fk.004", "spine.005", "Spine4", "spine_04", "spine_05"],

    # Neck / head
    "c_neck":     ["neck", "Neck", "neck_01", "spine.005", "spine.006"],
    "c_head":     ["head", "Head", "head_01", "spine.006"],

    # Face (optional)
    "c_jaw":      ["jaw", "jaw_master", "Jaw", "CC_Base_JawRoot"],
    "l_eye":      ["eye.L", "LeftEye", "eye_l", "l_eye"],
    "r_eye":      ["eye.R", "RightEye", "eye_r", "r_eye"],

    # Left arm — for the WRIST, prefer IK target so IK solver drives the chain.
    # FK version is also added below in auto-match so it works if the rig is in FK mode.
    "l_clavicle": ["shoulder.L", "clavicle.L", "LeftShoulder", "clavicle_l", "shoulder_l", "l_clavicle"],
    "l_uparm":    ["upper_arm.L", "arm.L", "LeftArm", "upperarm_l", "upper_arm_fk.L", "upperarm.L", "l_upperarm"],
    "l_lowarm":   ["forearm.L", "lowerarm.L", "LeftForeArm", "lowerarm_l", "forearm_fk.L", "l_forearm"],
    "l_wrist":    ["hand_ik.L", "hand.L", "wrist.L", "LeftHand", "hand_l", "hand_fk.L", "l_hand"],

    # Right arm
    "r_clavicle": ["shoulder.R", "clavicle.R", "RightShoulder", "clavicle_r", "shoulder_r", "r_clavicle"],
    "r_uparm":    ["upper_arm.R", "arm.R", "RightArm", "upperarm_r", "upper_arm_fk.R", "upperarm.R", "r_upperarm"],
    "r_lowarm":   ["forearm.R", "lowerarm.R", "RightForeArm", "lowerarm_r", "forearm_fk.R", "r_forearm"],
    "r_wrist":    ["hand_ik.R", "hand.R", "wrist.R", "RightHand", "hand_r", "hand_fk.R", "r_hand"],

    # Left leg
    "l_upleg":    ["thigh.L", "upleg.L", "LeftUpLeg", "thigh_l", "thigh_fk.L", "upperleg.L", "l_thigh"],
    "l_lowleg":   ["shin.L", "calf.L", "LeftLeg", "calf_l", "shin_fk.L", "lowerleg.L", "l_shin"],
    # l_foot: ankle-level bone. Prefer FK targets (foot_ik target already
    # gets driven by l_subtalar which is at a better height for IK).
    "l_foot":     ["foot_fk.L", "foot.L", "LeftFoot", "foot_l", "foot_ik.L", "l_foot"],
    "l_subtalar": ["foot_ik.L", "foot.L", "LeftFoot", "foot_l", "foot_fk.L", "l_foot"],
    "l_ball":     ["toe.L", "LeftToeBase", "ball.L", "ball_l", "l_toe", "toe_l"],

    # Right leg
    "r_upleg":    ["thigh.R", "upleg.R", "RightUpLeg", "thigh_r", "thigh_fk.R", "upperleg.R", "r_thigh"],
    "r_lowleg":   ["shin.R", "calf.R", "RightLeg", "calf_r", "shin_fk.R", "lowerleg.R", "r_shin"],
    "r_foot":     ["foot_fk.R", "foot.R", "RightFoot", "foot_r", "foot_ik.R", "r_foot"],
    "r_subtalar": ["foot_ik.R", "foot.R", "RightFoot", "foot_r", "foot_fk.R", "r_foot"],
    "r_ball":     ["toe.R", "RightToeBase", "ball.R", "ball_r", "r_toe", "toe_r"],

    # Left fingers
    "l_thumb1":   ["thumb.01.L", "LeftHandThumb1", "thumb_01_l", "f_thumb.01.L", "l_thumb_01"],
    "l_thumb2":   ["thumb.02.L", "LeftHandThumb2", "thumb_02_l", "f_thumb.02.L", "l_thumb_02"],
    "l_thumb3":   ["thumb.03.L", "LeftHandThumb3", "thumb_03_l", "f_thumb.03.L", "l_thumb_03"],
    "l_index1":   ["index.01.L", "f_index.01.L", "LeftHandIndex1", "index_01_l", "l_index_01"],
    "l_index2":   ["index.02.L", "f_index.02.L", "LeftHandIndex2", "index_02_l", "l_index_02"],
    "l_index3":   ["index.03.L", "f_index.03.L", "LeftHandIndex3", "index_03_l", "l_index_03"],
    "l_middle1":  ["middle.01.L", "f_middle.01.L", "LeftHandMiddle1", "middle_01_l", "l_middle_01"],
    "l_middle2":  ["middle.02.L", "f_middle.02.L", "LeftHandMiddle2", "middle_02_l", "l_middle_02"],
    "l_middle3":  ["middle.03.L", "f_middle.03.L", "LeftHandMiddle3", "middle_03_l", "l_middle_03"],
    "l_ring1":    ["ring.01.L", "f_ring.01.L", "LeftHandRing1", "ring_01_l", "l_ring_01"],
    "l_ring2":    ["ring.02.L", "f_ring.02.L", "LeftHandRing2", "ring_02_l", "l_ring_02"],
    "l_ring3":    ["ring.03.L", "f_ring.03.L", "LeftHandRing3", "ring_03_l", "l_ring_03"],
    "l_pinky1":   ["pinky.01.L", "f_pinky.01.L", "LeftHandPinky1", "pinky_01_l", "l_pinky_01"],
    "l_pinky2":   ["pinky.02.L", "f_pinky.02.L", "LeftHandPinky2", "pinky_02_l", "l_pinky_02"],
    "l_pinky3":   ["pinky.03.L", "f_pinky.03.L", "LeftHandPinky3", "pinky_03_l", "l_pinky_03"],

    # Right fingers
    "r_thumb1":   ["thumb.01.R", "RightHandThumb1", "thumb_01_r", "f_thumb.01.R", "r_thumb_01"],
    "r_thumb2":   ["thumb.02.R", "RightHandThumb2", "thumb_02_r", "f_thumb.02.R", "r_thumb_02"],
    "r_thumb3":   ["thumb.03.R", "RightHandThumb3", "thumb_03_r", "f_thumb.03.R", "r_thumb_03"],
    "r_index1":   ["index.01.R", "f_index.01.R", "RightHandIndex1", "index_01_r", "r_index_01"],
    "r_index2":   ["index.02.R", "f_index.02.R", "RightHandIndex2", "index_02_r", "r_index_02"],
    "r_index3":   ["index.03.R", "f_index.03.R", "RightHandIndex3", "index_03_r", "r_index_03"],
    "r_middle1":  ["middle.01.R", "f_middle.01.R", "RightHandMiddle1", "middle_01_r", "r_middle_01"],
    "r_middle2":  ["middle.02.R", "f_middle.02.R", "RightHandMiddle2", "middle_02_r", "r_middle_02"],
    "r_middle3":  ["middle.03.R", "f_middle.03.R", "RightHandMiddle3", "middle_03_r", "r_middle_03"],
    "r_ring1":    ["ring.01.R", "f_ring.01.R", "RightHandRing1", "ring_01_r", "r_ring_01"],
    "r_ring2":    ["ring.02.R", "f_ring.02.R", "RightHandRing2", "ring_02_r", "r_ring_02"],
    "r_ring3":    ["ring.03.R", "f_ring.03.R", "RightHandRing3", "ring_03_r", "r_ring_03"],
    "r_pinky1":   ["pinky.01.R", "f_pinky.01.R", "RightHandPinky1", "pinky_01_r", "r_pinky_01"],
    "r_pinky2":   ["pinky.02.R", "f_pinky.02.R", "RightHandPinky2", "pinky_02_r", "r_pinky_02"],
    "r_pinky3":   ["pinky.03.R", "f_pinky.03.R", "RightHandPinky3", "pinky_03_r", "r_pinky_03"],
}


# Rigify Human metarig default mapping. Load it via the "Load Rigify preset"
# button to seed the mapping list, then edit rows to suit your rig.
# (source_bone, target_bone, mode). Sintel-style CloudRig control names —
# built from a rig generated from the CloudRig Sintel metarig sample.
#
# Mode selection is intentional per bone type:
#   DELTA    — for torso bones (spine, neck, head, jaw, eyes). DELTA uses a
#              fixed "character-forward" canonical that lines up with these
#              bones' natural Y-up rest orientation.
#   AIM_ROLL — for limb bones (clavicles, arms, legs, feet, FK-Hand). AIM_ROLL
#              aims the target's Y-axis at the source's Y direction and copies
#              twist around Y, so it works regardless of rest-axis conventions
#              between MHR and CloudRig. DELTA does NOT work for limbs because
#              their rest Y isn't the vertical canonical DELTA assumes.
#   FINGER   — for fingers. Dedicated palm-normal anatomical frame transfer
#              (same idea as hand-IK) — convention-invariant and carries
#              finger curl properly. Plain AIM/AIM_ROLL would only match Y
#              direction and miss the curl.
#   POS      — for IK end effectors (IK-Hand, IK-Foot) and the world root.
#              Position-only with chain-scaled proportions.
SINTEL_DEFAULT_MAPPING = [
    # Ground master + torso
    ("world",      "root",                  "POS"),
    ("root",       "TORSO-Spine",           "DELTA"),

    # Spine chain — Sintel FK is only 3-tier so we skip the middle MHR bones.
    # Bump the "Spine bend amplify" slider if target ends up straighter than
    # source, or manually target c_spine1/c_spine2 at intermediate helpers.
    ("c_spine0",   "FK-Spine",              "DELTA"),
    ("c_spine1",   "",                      "SKIP"),
    ("c_spine2",   "",                      "SKIP"),
    ("c_spine3",   "FK-Chest",              "DELTA"),

    # Neck / head / face
    # Plain AIM (not AIM_ROLL) for neck/head because the AIM_ROLL twist match
    # picks the wrong sign of rotation around Y when source/target X-axes are
    # on opposite sides of Y (which happens with Sintel FK-Head vs MHR c_head),
    # producing an upside-down + backward face. AIM carries head tilt/nod
    # correctly and skips the head-yaw twist. Enable AIM_ROLL manually if you
    # need head-turn transfer and know how to add a per-row rotation offset
    # to correct the twist sign.
    # c_neck targets -Y because Sintel's FK-Neck rest Y sits on the opposite
    # side of the aim direction MHR c_neck expects — plain +Y/+Y makes the
    # neck tilt away from the source's direction. -Y flips the aim to match.
    ("c_neck",     "FK-Neck",               "AIM", "+Y", "-Y"),
    ("c_head",     "FK-Head",               "AIM"),
    ("c_jaw",      "Jaw",                   "AIM"),
    ("l_eye",      "Eye.L",                 "AIM"),
    ("r_eye",      "Eye.R",                 "AIM"),

    # Left arm
    ("l_clavicle", "FK-Shoulder.L",         "AIM_ROLL"),
    ("l_uparm",    "FK-UpperArm.L",         "AIM_ROLL"),
    ("l_lowarm",   "FK-Forearm.L",          "AIM_ROLL"),
    ("l_wrist",    "IK-Hand.L",             "POS"),
    ("l_wrist",    "FK-Hand.L",             "AIM_ROLL"),

    # Right arm
    ("r_clavicle", "FK-Shoulder.R",         "AIM_ROLL"),
    ("r_uparm",    "FK-UpperArm.R",         "AIM_ROLL"),
    ("r_lowarm",   "FK-Forearm.R",          "AIM_ROLL"),
    ("r_wrist",    "IK-Hand.R",             "POS"),
    ("r_wrist",    "FK-Hand.R",             "AIM_ROLL"),

    # Left leg — subtalar (below ankle) drives IK for correct heel height.
    # FK-Foot uses l_ball as the aim source (not l_foot / l_subtalar) because
    # MHR l_foot's Y-axis is overridden to point at subtalar (down), which
    # doesn't match Sintel's forward-pointing FK-Foot Y convention. l_ball
    # gives a ball-to-toe-tip direction closer to Sintel's foot Y.
    #
    # Toes intentionally left on SKIP: MHR l_ball is a LEAF bone whose Y axis
    # is a synthetic extension of midfoot-to-ball rather than a real toe
    # direction, and SAM3D doesn't reconstruct meaningful toe articulation
    # from monocular images anyway — aim-based toe transfer just adds noise.
    # Enable manually if your source really has per-toe motion.
    ("l_upleg",    "FK-Thigh.L",            "AIM_ROLL"),
    ("l_lowleg",   "FK-Knee.L",             "AIM_ROLL"),
    ("l_subtalar", "IK-Foot.L",             "POS"),
    ("l_ball",     "FK-Foot.L",             "AIM_ROLL"),
    ("l_ball",     "FK-Toes.L",             "SKIP"),

    # Right leg
    ("r_upleg",    "FK-Thigh.R",            "AIM_ROLL"),
    ("r_lowleg",   "FK-Knee.R",             "AIM_ROLL"),
    ("r_subtalar", "IK-Foot.R",             "POS"),
    ("r_ball",     "FK-Foot.R",             "AIM_ROLL"),
    ("r_ball",     "FK-Toes.R",             "SKIP"),

    # Fingers — AIM (no _ROLL) because finger-twist transfer is jittery in
    # practice and rarely worth having. Sintel has 4 phalanges per finger;
    # MHR only tracks 3 for index/middle/ring so Finger_XN4 stays unmapped
    # for those. Thumb has 4 in both (thumb0..3 → Thumb1..4). Pinky has a
    # carpal (pinky0) that maps to Finger_Pinky_Carpal.
    ("l_thumb0",   "Finger_Thumb1.L",       "FINGER"),
    ("l_thumb1",   "Finger_Thumb2.L",       "FINGER"),
    ("l_thumb2",   "Finger_Thumb3.L",       "FINGER"),
    ("l_thumb3",   "Finger_Thumb4.L",       "FINGER"),
    ("l_index1",   "Finger_Index1.L",       "FINGER"),
    ("l_index2",   "Finger_Index2.L",       "FINGER"),
    ("l_index3",   "Finger_Index3.L",       "FINGER"),
    ("l_middle1",  "Finger_Middle1.L",      "FINGER"),
    ("l_middle2",  "Finger_Middle2.L",      "FINGER"),
    ("l_middle3",  "Finger_Middle3.L",      "FINGER"),
    ("l_ring1",    "Finger_Ring1.L",        "FINGER"),
    ("l_ring2",    "Finger_Ring2.L",        "FINGER"),
    ("l_ring3",    "Finger_Ring3.L",        "FINGER"),
    ("l_pinky0",   "Finger_Pinky_Carpal.L", "FINGER"),
    ("l_pinky1",   "Finger_Pinky1.L",       "FINGER"),
    ("l_pinky2",   "Finger_Pinky2.L",       "FINGER"),
    ("l_pinky3",   "Finger_Pinky3.L",       "FINGER"),

    ("r_thumb0",   "Finger_Thumb1.R",       "FINGER"),
    ("r_thumb1",   "Finger_Thumb2.R",       "FINGER"),
    ("r_thumb2",   "Finger_Thumb3.R",       "FINGER"),
    ("r_thumb3",   "Finger_Thumb4.R",       "FINGER"),
    ("r_index1",   "Finger_Index1.R",       "FINGER"),
    ("r_index2",   "Finger_Index2.R",       "FINGER"),
    ("r_index3",   "Finger_Index3.R",       "FINGER"),
    ("r_middle1",  "Finger_Middle1.R",      "FINGER"),
    ("r_middle2",  "Finger_Middle2.R",      "FINGER"),
    ("r_middle3",  "Finger_Middle3.R",      "FINGER"),
    ("r_ring1",    "Finger_Ring1.R",        "FINGER"),
    ("r_ring2",    "Finger_Ring2.R",        "FINGER"),
    ("r_ring3",    "Finger_Ring3.R",        "FINGER"),
    ("r_pinky0",   "Finger_Pinky_Carpal.R", "FINGER"),
    ("r_pinky1",   "Finger_Pinky1.R",       "FINGER"),
    ("r_pinky2",   "Finger_Pinky2.R",       "FINGER"),
    ("r_pinky3",   "Finger_Pinky3.R",       "FINGER"),
]


RIGIFY_EDIT_MAPPING = [
    # Ground master + pelvis
    ("world",      "root"),          # MHR world (ground) -> Rigify ground master
    ("root",       "spine_fk"),      # MHR "root" is the pelvis -> Rigify FK spine base
    ("c_spine0",   "spine_fk.001"),
    ("c_spine1",   "spine_fk.002"),
    ("c_spine2",   "spine_fk.003"),
    ("c_spine3",   "chest"),

    # Neck / head
    ("c_neck",     "neck"),
    ("c_head",     "head"),

    # Face
    ("c_jaw",      "jaw_master"),
    ("l_eye",      "eye.L"),
    ("r_eye",      "eye.R"),

    # Left arm — IK targets first (drive the chain in IK mode), FK as backup
    ("l_clavicle", "shoulder.L"),
    ("l_uparm",    "upper_arm_fk.L"),
    ("l_lowarm",   "forearm_fk.L"),
    ("l_wrist",    "hand_ik.L"),
    ("l_wrist",    "hand_fk.L"),

    # Right arm
    ("r_clavicle", "shoulder.R"),
    ("r_uparm",    "upper_arm_fk.R"),
    ("r_lowarm",   "forearm_fk.R"),
    ("r_wrist",    "hand_ik.R"),
    ("r_wrist",    "hand_fk.R"),

    # Left leg — subtalar (below ankle) drives IK for correct heel height,
    # l_foot (ankle) drives FK
    ("l_upleg",    "thigh_fk.L"),
    ("l_lowleg",   "shin_fk.L"),
    ("l_subtalar", "foot_ik.L"),
    ("l_foot",     "foot_fk.L"),
    ("l_ball",     "toe.L"),

    # Right leg
    ("r_upleg",    "thigh_fk.R"),
    ("r_lowleg",   "shin_fk.R"),
    ("r_subtalar", "foot_ik.R"),
    ("r_foot",     "foot_fk.R"),
    ("r_ball",     "toe.R"),

    # Fingers - left
    ("l_thumb1",   "thumb.01.L"),
    ("l_thumb2",   "thumb.02.L"),
    ("l_thumb3",   "thumb.03.L"),
    ("l_index1",   "f_index.01.L"),
    ("l_index2",   "f_index.02.L"),
    ("l_index3",   "f_index.03.L"),
    ("l_middle1",  "f_middle.01.L"),
    ("l_middle2",  "f_middle.02.L"),
    ("l_middle3",  "f_middle.03.L"),
    ("l_ring1",    "f_ring.01.L"),
    ("l_ring2",    "f_ring.02.L"),
    ("l_ring3",    "f_ring.03.L"),
    ("l_pinky1",   "f_pinky.01.L"),
    ("l_pinky2",   "f_pinky.02.L"),
    ("l_pinky3",   "f_pinky.03.L"),

    # Fingers - right
    ("r_thumb1",   "thumb.01.R"),
    ("r_thumb2",   "thumb.02.R"),
    ("r_thumb3",   "thumb.03.R"),
    ("r_index1",   "f_index.01.R"),
    ("r_index2",   "f_index.02.R"),
    ("r_index3",   "f_index.03.R"),
    ("r_middle1",  "f_middle.01.R"),
    ("r_middle2",  "f_middle.02.R"),
    ("r_middle3",  "f_middle.03.R"),
    ("r_ring1",    "f_ring.01.R"),
    ("r_ring2",    "f_ring.02.R"),
    ("r_ring3",    "f_ring.03.R"),
    ("r_pinky1",   "f_pinky.01.R"),
    ("r_pinky2",   "f_pinky.02.R"),
    ("r_pinky3",   "f_pinky.03.R"),
]


# (source_bone, target_bone, mode) — matches vanilla Blender Rigify Human rig
# (generated from Add → Armature → Human Meta-Rig → Rigify Buttons → Generate).
# Same per-body-part mode conventions as the Sintel preset:
#   DELTA    — torso (spine, chest, neck, head, jaw, eyes)
#   AIM_ROLL — limbs (clavicle, arms, thigh/shin/foot)
#   FINGER   — fingers (uses palm-normal anatomical transfer)
#   POS      — IK end effectors (hand_ik, foot_ik)
#
# Difference from the (edit) preset:
#   - Toe target is `toe_fk.{L,R}` (standard Rigify split-toe naming), not
#     `toe.{L,R}` which doesn't exist on a vanilla generated rig.
#   - Correct modes per body part instead of blanket FULL.
RIGIFY_STANDARD_MAPPING = [
    # Ground master + pelvis
    ("world",      "root",             "POS"),
    ("root",       "spine_fk",         "DELTA"),
    ("c_spine0",   "spine_fk.001",     "DELTA"),
    ("c_spine1",   "spine_fk.002",     "DELTA"),
    ("c_spine2",   "spine_fk.003",     "DELTA"),
    ("c_spine3",   "chest",            "DELTA"),

    # Neck / head / face
    ("c_neck",     "neck",             "DELTA"),
    ("c_head",     "head",             "DELTA"),
    ("c_jaw",      "jaw_master",       "DELTA"),
    ("l_eye",      "eye.L",            "DELTA"),
    ("r_eye",      "eye.R",            "DELTA"),

    # Left arm
    ("l_clavicle", "shoulder.L",       "AIM_ROLL"),
    ("l_uparm",    "upper_arm_fk.L",   "AIM_ROLL"),
    ("l_lowarm",   "forearm_fk.L",     "AIM_ROLL"),
    ("l_wrist",    "hand_ik.L",        "POS"),
    ("l_wrist",    "hand_fk.L",        "AIM_ROLL"),

    # Right arm
    ("r_clavicle", "shoulder.R",       "AIM_ROLL"),
    ("r_uparm",    "upper_arm_fk.R",   "AIM_ROLL"),
    ("r_lowarm",   "forearm_fk.R",     "AIM_ROLL"),
    ("r_wrist",    "hand_ik.R",        "POS"),
    ("r_wrist",    "hand_fk.R",        "AIM_ROLL"),

    # Left leg — subtalar (below ankle) drives IK for correct heel height,
    # l_ball (base of toes) drives FK-Foot (matches Rigify foot_fk Y convention
    # better than l_foot, which points at the subtalar via the tail override).
    ("l_upleg",    "thigh_fk.L",       "AIM_ROLL"),
    ("l_lowleg",   "shin_fk.L",        "AIM_ROLL"),
    ("l_subtalar", "foot_ik.L",        "POS"),
    ("l_ball",     "foot_fk.L",        "AIM_ROLL"),
    ("l_ball",     "toe_fk.L",         "SKIP"),

    # Right leg
    ("r_upleg",    "thigh_fk.R",       "AIM_ROLL"),
    ("r_lowleg",   "shin_fk.R",        "AIM_ROLL"),
    ("r_subtalar", "foot_ik.R",        "POS"),
    ("r_ball",     "foot_fk.R",        "AIM_ROLL"),
    ("r_ball",     "toe_fk.R",         "SKIP"),

    # Fingers — MHR only tracks 3 phalanges per non-thumb finger. Rigify's
    # `f_XXX.03.L` is the last phalanx (fingertip anchor); we don't map a 4th.
    # Pinky's carpal joint in MHR (l_pinky0) has no Rigify equivalent.
    ("l_thumb0",   "thumb.01.L",       "FINGER"),
    ("l_thumb1",   "thumb.02.L",       "FINGER"),
    ("l_thumb2",   "thumb.03.L",       "FINGER"),
    ("l_index1",   "f_index.01.L",     "FINGER"),
    ("l_index2",   "f_index.02.L",     "FINGER"),
    ("l_index3",   "f_index.03.L",     "FINGER"),
    ("l_middle1",  "f_middle.01.L",    "FINGER"),
    ("l_middle2",  "f_middle.02.L",    "FINGER"),
    ("l_middle3",  "f_middle.03.L",    "FINGER"),
    ("l_ring1",    "f_ring.01.L",      "FINGER"),
    ("l_ring2",    "f_ring.02.L",      "FINGER"),
    ("l_ring3",    "f_ring.03.L",      "FINGER"),
    ("l_pinky1",   "f_pinky.01.L",     "FINGER"),
    ("l_pinky2",   "f_pinky.02.L",     "FINGER"),
    ("l_pinky3",   "f_pinky.03.L",     "FINGER"),

    ("r_thumb0",   "thumb.01.R",       "FINGER"),
    ("r_thumb1",   "thumb.02.R",       "FINGER"),
    ("r_thumb2",   "thumb.03.R",       "FINGER"),
    ("r_index1",   "f_index.01.R",     "FINGER"),
    ("r_index2",   "f_index.02.R",     "FINGER"),
    ("r_index3",   "f_index.03.R",     "FINGER"),
    ("r_middle1",  "f_middle.01.R",    "FINGER"),
    ("r_middle2",  "f_middle.02.R",    "FINGER"),
    ("r_middle3",  "f_middle.03.R",    "FINGER"),
    ("r_ring1",    "f_ring.01.R",      "FINGER"),
    ("r_ring2",    "f_ring.02.R",      "FINGER"),
    ("r_ring3",    "f_ring.03.R",      "FINGER"),
    ("r_pinky1",   "f_pinky.01.R",     "FINGER"),
    ("r_pinky2",   "f_pinky.02.R",     "FINGER"),
    ("r_pinky3",   "f_pinky.03.R",     "FINGER"),
]


def _iter_pose_bones_top_down(armature_obj):
    """Root-first traversal so each parent's pose is committed before children."""
    stack = [b for b in armature_obj.pose.bones if b.parent is None]
    while stack:
        b = stack.pop(0)
        yield b
        stack.extend(b.children)


def _compute_pole_position(src, shoulder_name, elbow_name, wrist_name, distance):
    """Return world-space position for an IK pole, computed from the source
    arm/leg triplet (shoulder-elbow-wrist or hip-knee-ankle). The pole is
    placed in the elbow's plane, on the far side of the chord."""
    from mathutils import Vector
    s_pb = src.pose.bones.get(shoulder_name)
    e_pb = src.pose.bones.get(elbow_name)
    w_pb = src.pose.bones.get(wrist_name)
    if not (s_pb and e_pb and w_pb):
        return None
    S = src.matrix_world @ s_pb.head
    E = src.matrix_world @ e_pb.head
    W = src.matrix_world @ w_pb.head
    chord = W - S
    if chord.length < 1e-4:
        return None
    # Project elbow onto chord, then offset direction = elbow - projection
    t = (E - S).dot(chord) / chord.length_squared
    proj = S + t * chord
    to_elbow = E - proj
    if to_elbow.length < 1e-4:
        # Arm straight; fall back to a chord-perpendicular in the world-up plane
        up = Vector((0, 0, 1))
        to_elbow = chord.cross(up).cross(chord)
        if to_elbow.length < 1e-4:
            return E + Vector((0, -1, 0)) * distance
    to_elbow.normalize()
    return E + to_elbow * distance


def _set_pose_bone_world_position(dst, bone_name, world_pos):
    """Move target bone's head to the given world position, preserving its
    current rotation. Returns True if applied."""
    pb = dst.pose.bones.get(bone_name)
    if pb is None:
        return False
    cur_world = dst.matrix_world @ pb.matrix
    cur_world.translation = world_pos
    pb.matrix = dst.matrix_world.inverted() @ cur_world
    bpy.context.view_layer.update()
    return True


def _sort_mappings_by_source_order(props, src):
    """Rebuild _prof(props).mappings so entries appear in source-armature bone order.
    Rows whose source_bone isn't on the source armature go to the end, in
    their current relative order. Preserves target/mode/enabled per row.
    No-op if src is None."""
    if src is None or len(_prof(props).mappings) < 2:
        return
    bone_order = {b.name: i for i, b in enumerate(src.data.bones)}
    unknown_offset = len(bone_order) + 1

    def key(m):
        return (bone_order.get(m.source_bone, unknown_offset), m.target_bone)

    # Snapshot current data
    snapshot = [(m.source_bone, m.target_bone, m.enabled, m.mode, m.source_axis, m.target_axis) for m in _prof(props).mappings]
    snapshot.sort(key=lambda t: (bone_order.get(t[0], unknown_offset), t[1] or ""))

    # Rewrite in place
    with _bulk_mode():
        _prof(props).mappings.clear()
        for s, t, en, mode, sa, ta in snapshot:
            item = _prof(props).mappings.add()
            item.source_bone = s
            item.target_bone = t
            item.enabled = en
            item.mode = mode
            item.source_axis = sa
            item.target_axis = ta


# Source triplets used to compute each pole. Order: shoulder/hip, elbow/knee, wrist/ankle.
_POLE_TRIPLETS = {
    "l_arm_pole": ("l_uparm", "l_lowarm", "l_wrist"),
    "r_arm_pole": ("r_uparm", "r_lowarm", "r_wrist"),
    "l_leg_pole": ("l_upleg", "l_lowleg", "l_foot"),
    "r_leg_pole": ("r_upleg", "r_lowleg", "r_foot"),
}


# Reentrance / bulk-edit guards for the live-update mechanism.
_LIVE_COPY_LOCK = False
_BULK_EDIT = False


class _bulk_mode:
    """Context manager: suppress live-update callbacks during a batch operation
    like auto-match or preset load, otherwise every add() would trigger a copy."""
    def __enter__(self):
        global _BULK_EDIT
        _BULK_EDIT = True
        return self
    def __exit__(self, *args):
        global _BULK_EDIT
        _BULK_EDIT = False


def _row_updated(self, context):
    """Update callback on mapping-row fields. Re-runs the FULL copy (all
    enabled rows) so the interactive edit gives the same visual result as
    clicking Copy All. A single-row copy would use rest-pose parents for the
    row's ancestors, which produces a wrong-context intermediate result when
    other rows (torso, hip, hand) haven't been applied yet.

    Costs one full pose evaluation per field change — cheap enough at 60-100
    rows to feel instant. If it becomes a bottleneck, the user can turn off
    the "Live update" toggle."""
    global _LIVE_COPY_LOCK
    if _LIVE_COPY_LOCK or _BULK_EDIT:
        return
    props = context.scene.sam3d_pose_copy
    if not getattr(props, "live_update", True):
        return
    src = _prof(props).source_armature
    dst = _prof(props).target_armature
    if src is None or dst is None or src == dst:
        return
    _LIVE_COPY_LOCK = True
    try:
        entries = [
            (m.source_bone, m.target_bone, m.enabled, m.mode,
             m.source_axis, m.target_axis)
            for m in _prof(props).mappings
        ]
        _copy_pose(src, dst, entries,
                   props.position_only,
                   props.reset_before_copy,
                   lambda *a, **k: None,
                   compute_poles=True)
    finally:
        _LIVE_COPY_LOCK = False


_AXIS_INDEX = {'X': 0, 'Y': 1, 'Z': 2}


def _axis_vec_from_matrix(mat, axis_str):
    """Extract a signed local-axis vector from a 4x4 matrix's rotation
    columns. axis_str is one of '+X', '-X', '+Y', '-Y', '+Z', '-Z'.

    Returns a 3D Vector (not normalized)."""
    letter = axis_str[1] if len(axis_str) == 2 else axis_str[0]
    idx = _AXIS_INDEX.get(letter.upper(), 1)
    v = mat.col[idx].to_3d()
    if axis_str.startswith('-'):
        v = -v
    return v


def _perp_axis_letter(axis_str):
    """Pick a canonical axis LETTER perpendicular to the given one, used as
    the 'roll reference' for AIM_ROLL twist matching. Cyclic:
        aim Y → reference X
        aim X → reference Y
        aim Z → reference X (X and Z rows both use X as reference since Y
                             would collide with the natural along-bone axis)
    Sign is ignored — the twist calculation projects into a plane and doesn't
    care about the sign of the reference axis."""
    letter = axis_str[1] if len(axis_str) == 2 else axis_str[0]
    return {'X': 'Y', 'Y': 'X', 'Z': 'X'}.get(letter.upper(), 'X')


def _copy_pose(src, dst, entries, global_position_only, reset_first, report, compute_poles=True):
    """Copy pose from src to dst. `entries` is a list of 6-tuples describing
    each row: (source_bone, target_bone, enabled, mode, src_axis, tgt_axis).
    src_axis and tgt_axis are strings like '+Y', '-Y', '+X' etc — used by
    AIM / AIM_ROLL to pick which axes to align.
    mode is one of 'FULL', 'AIM', 'AIM_ROLL', 'DELTA', 'FINGER', 'POS', 'POS_RAW'.
    global_position_only forces POS mode for all rows.
    reset_first: if True, clears target pose (matrix_basis = identity) before applying."""
    if src is None or dst is None:
        report({'ERROR'}, "Set both source and target armatures.")
        return 0, [], []
    if src == dst:
        report({'ERROR'}, "Source and target must be different armatures.")
        return 0, [], []

    # Backwards-compat: entries may be 4-, 5-, 6-, or 7-tuples. Legacy 5/7
    # forms carried a flip_z bool that is now dropped (replaced by the axis
    # dropdowns), so discard it on load.
    def _unpack(e):
        if len(e) == 6:
            return e
        if len(e) == 7:
            s, t, en, m, _fz, sa, ta = e
            return (s, t, en, m, sa, ta)
        if len(e) == 5:
            s, t, en, m, _fz = e
            return (s, t, en, m, '+Y', '+Y')
        s, t, en, m = e
        return (s, t, en, m, '+Y', '+Y')
    entries = [_unpack(e) for e in entries]

    active = [(s, t, m, sa, ta) for (s, t, en, m, sa, ta) in entries
              if en and s and t and m != 'SKIP']
    if not active:
        report({'WARNING'}, "No enabled mappings.")
        return 0, [], []

    # Snapshot source world matrices; also remember per-target settings
    # (mode, source axis, target axis, source bone name, and source's
    # bone-local pose delta for POS-mode IK bone-local rotation fallback).
    src_world = src.matrix_world
    per_target = {}  # target_name -> (world_matrix, local_delta_q, mode, source_bone_name, src_axis, tgt_axis)
    missing_source = []
    for source_bone, target_bone, row_mode, row_src_axis, row_tgt_axis in active:
        pb = src.pose.bones.get(source_bone)
        if pb is None:
            missing_source.append(source_bone)
            continue
        per_target[target_bone] = (
            src_world @ pb.matrix,
            pb.matrix_basis.to_3x3().to_quaternion(),
            row_mode,
            source_bone,
            row_src_axis,
            row_tgt_axis,
        )

    # Fetch props once — needed by chain-scaled IK positioning inside the
    # per-bone loop AND by pole computation at the end.
    props = bpy.context.scene.sam3d_pose_copy

    # Prep target
    for obj in bpy.context.selected_objects:
        obj.select_set(False)
    dst.select_set(True)
    bpy.context.view_layer.objects.active = dst
    prev_mode = dst.mode
    if dst.mode != 'POSE':
        bpy.ops.object.mode_set(mode='POSE')

    # Optionally clear the target's pose so each Copy starts from rest.
    # Otherwise, toggling per-row `pos` after a previous full copy leaves
    # the target bone at the earlier rotation, and the toggle has no effect.
    # Bones on rows with mode SKIP are preserved so the user can manually
    # pose them and have it survive re-copies.
    if reset_first:
        skip_names = {t for (s, t, en, m, sa, ta) in entries if en and t and m == 'SKIP'}
        for pb in dst.pose.bones:
            if pb.name in skip_names:
                continue
            pb.matrix_basis = Matrix.Identity(4)
        bpy.context.view_layer.update()

    dst_world_inv = dst.matrix_world.inverted()

    # Compute a canonical source-rest frame (character right / up / forward)
    # from anatomical landmarks. Used by DELTA mode.
    canonical_rest_3x3 = None
    try:
        from mathutils import Vector as _V, Matrix as _M
        _l = src.matrix_world @ src.pose.bones['l_upleg'].head
        _r = src.matrix_world @ src.pose.bones['r_upleg'].head
        _root = src.matrix_world @ src.pose.bones['root'].head
        _head = src.matrix_world @ src.pose.bones['c_head'].head
        _right = (_r - _l)
        _up = (_head - _root)
        if _right.length > 1e-4 and _up.length > 1e-4:
            _right.normalize(); _up.normalize()
            _fwd = _right.cross(_up)
            if _fwd.length > 1e-4:
                _fwd.normalize()
                # Re-orthogonalize up so all three are perpendicular
                _up = _fwd.cross(_right).normalized()
                canonical_rest_3x3 = _M((
                    (_right.x, _up.x, _fwd.x),
                    (_right.y, _up.y, _fwd.y),
                    (_right.z, _up.z, _fwd.z),
                ))
    except (KeyError, AttributeError):
        pass

    # Re-snapshot after any reset.
    for source_bone, target_bone, row_mode, row_src_axis, row_tgt_axis in active:
        pb = src.pose.bones.get(source_bone)
        if pb is not None:
            per_target[target_bone] = (
                src_world @ pb.matrix,
                pb.matrix_basis.to_3x3().to_quaternion(),
                row_mode,
                source_bone,
                row_src_axis,
                row_tgt_axis,
            )

    from mathutils import Matrix as _M
    import math as _math

    # Per-row rotation offsets, indexed by target bone name for O(1) lookup
    # in the loop below.
    _rotation_offsets = {}
    for m in _prof(props).mappings:
        if m.enabled and m.target_bone:
            ro = m.rotation_offset
            if abs(ro[0]) > 1e-6 or abs(ro[1]) > 1e-6 or abs(ro[2]) > 1e-6:
                _rotation_offsets[m.target_bone] = tuple(ro)

    # PERF: precompute constants used every iteration and cache target-side
    # anatomical rest frames (one per side per body part) — these depend only
    # on target rest positions, not on the source pose.
    dst_world = dst.matrix_world
    dst_world_3x3 = dst_world.to_3x3()
    dst_world_3x3_inv = dst_world_3x3.inverted()
    _target_anat_cache = {}  # (side, is_hand) -> 3x3 or None
    def _cached_target_anat(side, is_hand):
        key = (side, is_hand)
        if key not in _target_anat_cache:
            frame = _target_anat_rest_frame_world(dst, side, is_hand)
            if frame is None:
                frame = _canonical_ik_frame_world(
                    canonical_rest_3x3, side, is_hand)
            _target_anat_cache[key] = frame
        return _target_anat_cache[key]

    _cached_rest = {}
    def _rest_local(bone):
        m = _cached_rest.get(bone.name)
        if m is None:
            m = bone.matrix_local.copy()
            _cached_rest[bone.name] = m
        return m

    # Ensure Blender's depsgraph is coherent before reading pose-bone worlds.
    bpy.context.view_layer.update()

    # Palm-normal caches keyed by side ('l' or 'r'). Computed lazily on first
    # use per copy — source palm normal from current pose, target from rest.
    _src_palm_normal_cache = {}
    _tgt_palm_normal_cache = {}
    def _src_palm(side):
        if side not in _src_palm_normal_cache:
            _src_palm_normal_cache[side] = _source_palm_normal_world(src, side)
        return _src_palm_normal_cache[side]
    def _tgt_palm_rest(side):
        if side not in _tgt_palm_normal_cache:
            _tgt_palm_normal_cache[side] = _target_palm_normal_world_rest(dst, side)
        return _tgt_palm_normal_cache[side]

    # Compute-once helper that produces the desired armature-local matrix for
    # a mapped bone in whatever mode it's in. Used by both copy passes so the
    # per-mode logic isn't duplicated.
    def _compute_new_matrix(pb, entry, parent_world):
        (target_world, src_local_delta_q, row_mode, source_bone,
         row_src_axis, row_tgt_axis) = entry
        if global_position_only:
            row_mode = 'POS'
        new_local = dst_world_inv @ target_world
        new_matrix = None

        def _attached_local_trans():
            """Armature-local translation keeping this bone attached at its
            rest offset from parent's CURRENT position (basis.translation = 0).
            Chain integrity survives when the parent has been moved (e.g.
            hip chain-scaled downward for kneeling)."""
            if pb.parent is None or parent_world is None:
                return _rest_local(pb.bone).translation
            return (parent_world
                    @ _rest_local(pb.parent.bone).inverted()
                    @ _rest_local(pb.bone).translation)

        if row_mode == 'POS':
            # For IK targets (hand_ik, foot_ik): position uses chain-scaled IK
            # (source direction, target chain length); rotation uses the source
            # hand/foot's ANATOMICAL frame delta computed from landmark bone
            # positions (wrist+finger bases for hands, ankle+ball+subtalar for
            # feet). Anatomical landmarks are chirality-consistent per side and
            # independent of any rig's bone-axis convention, so L and R sides
            # transfer symmetrically regardless of how MHR or Rigify orient
            # their IK bone rests. Falls back to bone-local delta (matrix_basis
            # transferred to target rest) if landmarks aren't available.
            proportion_pos = _chain_scaled_ik_position(
                pb, target_world, src, dst, dst_world_inv, props)
            if proportion_pos is not None:
                src_frame_now, is_hand, ik_side = (
                    _anatomical_ik_source_frame_world(pb, src))
                reference_frame = (_cached_target_anat(ik_side, is_hand)
                                    if is_hand is not None else None)
                if src_frame_now is not None and reference_frame is not None:
                    delta_world_3x3 = src_frame_now @ reference_frame.inverted()
                    tgt_rest_world_3x3 = (
                        dst_world @ _rest_local(pb.bone)).to_3x3()
                    final_world_3x3 = delta_world_3x3 @ tgt_rest_world_3x3
                    tgt_rot_3x3 = dst_world_3x3_inv @ final_world_3x3
                else:
                    tgt_rot_3x3 = (_rest_local(pb.bone).to_3x3()
                                    @ src_local_delta_q.to_matrix())
                proportion_pos_local = dst_world_inv @ proportion_pos
                new_matrix = Matrix.LocRotScale(
                    proportion_pos_local, tgt_rot_3x3.to_quaternion(),
                    (1.0, 1.0, 1.0))
            else:
                # Non-IK POS: keep target's rest rotation, use source position.
                # For hip / pelvis / torso, chain-scale the position so a
                # kneeling source lowers the target hips proportionally rather
                # than transferring raw source translation (which fails when
                # source and target rigs are different scales).
                tgt_rest_world = dst_world @ _rest_local(pb.bone)
                cur = dst_world_inv @ tgt_rest_world
                _hip_world = _chain_scaled_hip_world_position(
                    source_bone, src, dst, props, pb=pb)
                if _hip_world is not None:
                    cur.translation = dst_world_inv @ _hip_world
                else:
                    cur.translation = new_local.translation
                new_matrix = cur
        elif row_mode == 'DELTA':
            # All torso bones (root, spine chain, neck, head) go through the
            # same fixed world-space canonical:
            #     X = -X world (character right)
            #     Y = +Z world (spine up)
            #     Z = +Y world (character back — matches
            #                    _align_torso_rolls_from_landmarks)
            # Each bone gets its OWN delta from canonical applied to its
            # target rest — spine curvature, head tilt, neck bend all
            # transfer through the chain. Position: hip pinned to source
            # world position; other torso bones use parent-driven translation
            # so the chain integrity survives when hip moves/rotates.
            #
            # If target rest orientations partially cancel source's pose
            # deviation (e.g. Rigify natural spine curvature opposes source
            # forward-bend), you can amplify the delta via the row's
            # `rotation_offset` — but for typical rigs the default works.
            _canonical = Matrix((
                (-1.0, 0.0, 0.0),
                ( 0.0, 0.0, 1.0),
                ( 0.0, 1.0, 0.0),
            ))
            source_rot_3x3 = target_world.to_3x3()
            delta_3x3 = source_rot_3x3 @ _canonical.inverted()

            # For non-hip torso bones (spine chain, chest, neck, head, etc.),
            # optionally amplify the delta so target's own rest curvature
            # doesn't dampen the source's pose deviation. Scale rotation
            # angle via axis-angle: extract axis+angle, multiply angle by
            # the amplify factor, rebuild the quaternion. Cleanly supports
            # values > 1 (Blender's slerp clamps to [0,1]).
            _amp = float(getattr(_prof(props), "spine_bend_amplify", 1.0))
            if source_bone != 'root' and abs(_amp - 1.0) > 1e-4:
                _delta_q = delta_3x3.to_quaternion()
                _axis, _angle = _delta_q.to_axis_angle()
                delta_3x3 = Quaternion(_axis, _angle * _amp).to_matrix()

            tgt_rest_world = dst_world @ _rest_local(pb.bone)
            new_rot_3x3 = delta_3x3 @ tgt_rest_world.to_3x3()
            new_world = new_rot_3x3.to_4x4()

            if source_bone == 'root':
                _hip_world = _chain_scaled_hip_world_position(
                    source_bone, src, dst, props, pb=pb)
                if _hip_world is not None:
                    new_world.translation = _hip_world
                    new_matrix = dst_world_inv @ new_world
                else:
                    new_matrix = dst_world_inv @ new_world
                    new_matrix.translation = _attached_local_trans()
            else:
                new_matrix = dst_world_inv @ new_world
                new_matrix.translation = _attached_local_trans()
        elif row_mode in ('AIM', 'AIM_ROLL'):
            # Aim target's `row_tgt_axis` at source's `row_src_axis`. For
            # AIM_ROLL, also transfer the source's twist around the aim axis
            # so head/spine/pelvis facing is preserved.
            #
            # Axis dropdowns default to '+Y' / '+Y' (along-bone → along-bone),
            # which is the natural aim. Pick '-Y' on either side to flip the
            # aim direction (fixes cases like Sintel head where target's
            # natural aim ends up backward). Pick +X/+Z to use a different
            # local axis entirely (rare but supported).
            #
            # Position is preserved from target's REST (parent-driven) so
            # bones stay attached to their chain regardless of source/target
            # proportion differences.
            from mathutils import Vector, Matrix as _M
            src_aim = _axis_vec_from_matrix(target_world, row_src_axis)
            tgt_rest_world = dst.matrix_world @ pb.bone.matrix_local
            tgt_aim = _axis_vec_from_matrix(tgt_rest_world, row_tgt_axis)
            if src_aim.length < 1e-6 or tgt_aim.length < 1e-6:
                new_matrix = dst_world_inv @ tgt_rest_world
            else:
                src_aim.normalize(); tgt_aim.normalize()
                aim_quat = tgt_aim.rotation_difference(src_aim)
                after_aim_3x3 = aim_quat.to_matrix() @ tgt_rest_world.to_3x3()

                if row_mode == 'AIM_ROLL':
                    # Twist match: pick a "roll reference" axis perpendicular
                    # to the aim axis on both source and target. Historically
                    # aim was +Y and roll reference was +X; generalize by
                    # cycling: for aim Y use X, for aim X or Z use Y.
                    src_ref_letter = _perp_axis_letter(row_src_axis)
                    tgt_ref_letter = _perp_axis_letter(row_tgt_axis)
                    src_ref = _axis_vec_from_matrix(target_world, '+' + src_ref_letter)
                    aim_ref = _axis_vec_from_matrix(_M(after_aim_3x3).to_4x4(),
                                                     '+' + tgt_ref_letter)
                    # Project onto plane perpendicular to src_aim so twist is
                    # measured as a pure rotation around src_aim.
                    def _project(v, axis):
                        return v - axis * v.dot(axis)
                    src_ref_p = _project(src_ref, src_aim)
                    aim_ref_p = _project(aim_ref, src_aim)
                    if src_ref_p.length > 1e-6 and aim_ref_p.length > 1e-6:
                        src_ref_p.normalize(); aim_ref_p.normalize()
                        cos_t = max(-1.0, min(1.0, aim_ref_p.dot(src_ref_p)))
                        import math as _math
                        twist_angle = _math.acos(cos_t)
                        cross = aim_ref_p.cross(src_ref_p)
                        if cross.dot(src_aim) < 0:
                            twist_angle = -twist_angle
                        twist_quat = Quaternion(src_aim, twist_angle)
                        after_aim_3x3 = twist_quat.to_matrix() @ after_aim_3x3

                new_world = after_aim_3x3.to_4x4()
                new_matrix = dst_world_inv @ new_world
                # Parent-driven translation — follows parent chain when a
                # DELTA-mode ancestor (e.g. hip) has been moved.
                new_matrix.translation = _attached_local_trans()
        elif row_mode == 'FINGER':
            # Finger-specific: transfer using palm-normal anatomical frame.
            # Source and target rigs have subtly different bone-local X/Z
            # conventions per finger (and per side), so raw FULL rotation copy
            # gets some fingers right and inverts others (e.g. Rigify's middle
            # vs ring metacarpal roll differ). Anatomical-frame transfer
            # (Y along the bone plus palm-plane normal) is convention-invariant
            # and carries curl properly.
            #
            # Position must FOLLOW THE PARENT (basis.translation = 0) because
            # the finger's parent (hand chain) has already been moved by the
            # earlier IK/FK arm pose; pinning the finger to its armature-local
            # rest translation would leave it disconnected from the moved hand.
            if source_bone and _is_finger_source(source_bone):
                side = source_bone[0]  # 'l' or 'r'
                src_palm = _src_palm(side)
                tgt_palm = _tgt_palm_rest(side)
                src_pb = src.pose.bones.get(source_bone)
                if (src_palm is not None and tgt_palm is not None
                        and src_pb is not None):
                    src_bone_world = src.matrix_world @ src_pb.matrix
                    src_y = src_bone_world.col[1].to_3d()
                    src_frame = _frame_from_y_and_aux(src_y, src_palm)
                    tgt_rest_world = dst_world @ _rest_local(pb.bone)
                    tgt_y_rest = tgt_rest_world.col[1].to_3d()
                    tgt_frame_rest = _frame_from_y_and_aux(tgt_y_rest, tgt_palm)
                    if src_frame is not None and tgt_frame_rest is not None:
                        delta_world_3x3 = src_frame @ tgt_frame_rest.inverted()
                        new_world_rot = delta_world_3x3 @ tgt_rest_world.to_3x3()
                        new_local_rot = (dst_world_3x3_inv @ new_world_rot).to_quaternion()
                        new_matrix = Matrix.LocRotScale(
                            _attached_local_trans(), new_local_rot, (1.0, 1.0, 1.0))
            if new_matrix is None:
                # FINGER mode but source isn't a recognized MHR finger, or
                # palm-normal landmarks weren't found. Fall back to plain
                # FULL rotation copy so we at least produce something.
                _, rot, _ = new_local.decompose()
                new_matrix = Matrix.LocRotScale(
                    _attached_local_trans(), rot, (1.0, 1.0, 1.0))
        elif row_mode == 'POS_RAW':
            # Like POS but skip chain-scaling — use source's RAW world position
            # instead. Rotation still uses the same anatomical-frame transfer
            # (palm-normal for hands, foot landmarks for feet) so hand/foot
            # orientation carries over. Otherwise the IK target sits at the
            # right spot but with target-rest rotation, which looks twisted.
            src_pos_local = new_local.translation
            # Try anatomical rotation transfer first (works for IK-Hand /
            # IK-Foot). Falls back to source local delta applied to target
            # rest, matching POS's fallback behavior.
            src_frame_now, is_hand, ik_side = (
                _anatomical_ik_source_frame_world(pb, src))
            reference_frame = (_cached_target_anat(ik_side, is_hand)
                               if is_hand is not None else None)
            if src_frame_now is not None and reference_frame is not None:
                delta_world_3x3 = src_frame_now @ reference_frame.inverted()
                tgt_rest_world_3x3 = (
                    dst_world @ _rest_local(pb.bone)).to_3x3()
                final_world_3x3 = delta_world_3x3 @ tgt_rest_world_3x3
                tgt_rot_3x3 = dst_world_3x3_inv @ final_world_3x3
            else:
                tgt_rot_3x3 = (_rest_local(pb.bone).to_3x3()
                                @ src_local_delta_q.to_matrix())
            new_matrix = Matrix.LocRotScale(
                src_pos_local, tgt_rot_3x3.to_quaternion(), (1.0, 1.0, 1.0))
        else:  # FULL
            # Copy source's world rotation with PARENT-DRIVEN translation so
            # the bone stays attached to its chain as the parent moves.
            _, rot, _ = new_local.decompose()
            new_matrix = Matrix.LocRotScale(
                _attached_local_trans(), rot, (1.0, 1.0, 1.0))

        # Apply per-row rotation offset (rest-orientation correction).
        # Rotates in the bone's local frame around its origin, preserving the
        # bone's translation. Directly post-multiplying the 4x4 would rotate
        # the translation column and displace the bone.
        ro = _rotation_offsets.get(pb.name)
        if ro is not None:
            from mathutils import Euler as _E
            offset_3x3 = _E((ro[0], ro[1], ro[2]), 'XYZ').to_matrix()
            rot_only = new_matrix.to_3x3() @ offset_3x3
            new_matrix = Matrix.LocRotScale(
                new_matrix.to_translation(), rot_only.to_quaternion(), (1.0, 1.0, 1.0))

        return new_matrix

    # Small helper to commit a computed armature-local pose matrix as
    # matrix_basis, using the parent's known world. Avoids invoking the
    # pb.matrix setter (which needs a fresh depsgraph).
    def _set_basis_from_new_matrix(pb, new_matrix, parent_world):
        if pb.parent is None:
            basis = _rest_local(pb.bone).inverted() @ new_matrix
        else:
            basis = (_rest_local(pb.bone).inverted()
                     @ _rest_local(pb.parent.bone)
                     @ parent_world.inverted()
                     @ new_matrix)
        pb.matrix_basis = basis

    # TWO-PASS COPY.
    # Pass 1 sets POS-mode (IK controller) bones. Between passes we
    # view_layer.update() so Rigify's IK/copy-rotation constraints propagate
    # the new hand_ik/foot_ik state into the arm/leg deform chain — the
    # hand.fk.L (or DEF-hand.L) bone moves into position. Pass 2 then sets
    # the FK/finger bones with parent worlds read from Blender's now-correct
    # (post-IK) state; because fingers use parent-driven translation, they
    # attach to the moved hand instead of pinning at rest world position.
    applied = 0

    # --- Pass 1: POS (IK controller) bones ---
    for pb in _iter_pose_bones_top_down(dst):
        entry = per_target.get(pb.name)
        if entry is None or entry[2] != 'POS':
            continue
        parent_world = pb.parent.matrix.copy() if pb.parent is not None else None
        new_matrix = _compute_new_matrix(pb, entry, parent_world)
        _set_basis_from_new_matrix(pb, new_matrix, parent_world)
        applied += 1

    # Let IK / Copy-Rotation / etc. propagate.
    bpy.context.view_layer.update()

    # --- Pass 2: non-POS bones (FK / DELTA / AIM / fingers) ---
    computed_local = {}
    for pb in _iter_pose_bones_top_down(dst):
        entry = per_target.get(pb.name)
        if entry is None or entry[2] == 'POS':
            continue
        parent_pb = pb.parent
        if parent_pb is None:
            parent_world = None
        else:
            # Prefer Pass-2 tracked world (parent was touched earlier in
            # this pass); else read Blender's current pb.matrix, which for
            # POS / untouched bones already reflects the Pass-1 IK solve.
            parent_world = computed_local.get(parent_pb.name)
            if parent_world is None:
                parent_world = parent_pb.matrix.copy()
        new_matrix = _compute_new_matrix(pb, entry, parent_world)
        _set_basis_from_new_matrix(pb, new_matrix, parent_world)
        computed_local[pb.name] = new_matrix
        applied += 1

    bpy.context.view_layer.update()

    mapped_targets = set(per_target.keys())
    existing_targets = {pb.name for pb in dst.pose.bones}
    missing_target = sorted(mapped_targets - existing_targets)

    # Compute and apply IK pole targets from source arm/leg triplets.
    pole_applied = 0
    if compute_poles:
        for prop_name, (a, b, c) in _POLE_TRIPLETS.items():
            target_bone_name = getattr(_prof(props), prop_name, "")
            if not target_bone_name:
                continue
            pos = _compute_pole_position(src, a, b, c, _prof(props).pole_distance)
            if pos is None:
                continue
            if _set_pose_bone_world_position(dst, target_bone_name, pos):
                pole_applied += 1

    if dst.mode != prev_mode:
        bpy.ops.object.mode_set(mode=prev_mode)

    return applied + pole_applied, missing_source, missing_target


# -----------------------------------------------------------------------------
# Data model
# -----------------------------------------------------------------------------

class SAM3DMappingItem(PropertyGroup):
    source_bone: StringProperty(name="Source (MHR)", update=_row_updated)
    target_bone: StringProperty(name="Target", update=_row_updated)
    enabled: BoolProperty(name="", default=True, update=_row_updated)
    mode: bpy.props.EnumProperty(
        name="Mode",
        items=[
            ('FULL',     "Full",     "Copy source's rotation, preserve target's rest position. Bone stays attached to its chain regardless of proportion differences."),
            ('AIM',      "Aim",      "Aim target Y at source Y, preserve target rest position and rest X/Z. Best for limbs when only aim direction matters."),
            ('AIM_ROLL', "Aim+Roll", "Aim Y plus copy twist around Y. Preserves target rest position. Better than AIM when twist matters."),
            ('DELTA',    "Delta",    "Rotate target from its rest by the delta source rotated from an anatomical canonical (character axes). Preserves rest position. Best for spine/pelvis/head."),
            ('FINGER',   "Finger",   "Finger-specific: transfer using palm-normal anatomical frame. Convention-invariant across rigs — carries finger curl even when source/target bone rolls differ. Only meaningful when source is an MHR finger bone (l_index1, l_thumb0, etc.)."),
            ('POS',      "Pos",      "Position only; keep target rest rotation. For IK targets (foot_ik, hand_ik), position is scaled to target's chain length so proportions are respected."),
            ('POS_RAW',  "Pos (raw)","Like Pos but skip chain-scaling — pins the target IK at the source wrist/ankle raw world position. Rotation still uses the anatomical-frame transfer (palm-normal for hands, ankle+toe for feet) so hand/foot orientation carries over. Use when source and target have similar proportions and chain-scaled Pos is landing slightly off."),
            ('SKIP',     "Skip",     "Do not touch this bone. Use after manually adjusting a bone you want to preserve across re-copies."),
        ],
        default='FULL',
        update=_row_updated,
    )
    # Per-row axis mapping for AIM / AIM_ROLL — lets the user specify which
    # axis of the source bone should be "aimed" (default source Y = along the
    # bone) and which axis of the target should aim in that direction. Fixes
    # convention mismatches (e.g. Sintel head vs MHR head Z-sign) without
    # per-row rotation offsets.
    #
    # For AIM_ROLL the twist match uses a canonical perpendicular axis (bone
    # X for a Y aim, or bone Y for X/Z aims). For FULL / DELTA / FINGER / POS
    # the axis dropdowns are stored but currently have no effect — they only
    # matter for the AIM family.
    source_axis: bpy.props.EnumProperty(
        name="Src axis",
        description="Which axis of the SOURCE bone to aim / align. Default +Y "
                    "(along the bone, head-to-tail).",
        items=[
            ('+X', "+X", "Source's local +X axis"),
            ('+Y', "+Y", "Source's local +Y axis (default — along the bone)"),
            ('+Z', "+Z", "Source's local +Z axis"),
            ('-X', "-X", "Source's local -X axis"),
            ('-Y', "-Y", "Source's local -Y axis"),
            ('-Z', "-Z", "Source's local -Z axis"),
        ],
        default='+Y',
        update=_row_updated,
    )
    target_axis: bpy.props.EnumProperty(
        name="Tgt axis",
        description="Which axis of the TARGET bone should be aimed at the source axis. "
                    "Default +Y matches the source (natural aim). Pick -Y (or another axis) "
                    "to correct for convention mismatches between rigs (e.g. Sintel head).",
        items=[
            ('+X', "+X", "Target's local +X axis"),
            ('+Y', "+Y", "Target's local +Y axis (default)"),
            ('+Z', "+Z", "Target's local +Z axis"),
            ('-X', "-X", "Target's local -X axis"),
            ('-Y', "-Y", "Target's local -Y axis"),
            ('-Z', "-Z", "Target's local -Z axis"),
        ],
        default='+Y',
        update=_row_updated,
    )
    rotation_offset: bpy.props.FloatVectorProperty(
        name="Rotation offset",
        description="Additional rotation (Euler XYZ, degrees) applied to the target bone "
                    "after the source rotation is copied. Use to correct rest-orientation "
                    "mismatches (e.g. hand rig conventions where source is 'vertical hand' "
                    "and target is 'flat hand').",
        subtype='EULER',
        size=3,
        default=(0.0, 0.0, 0.0),
        unit='ROTATION',
        update=_row_updated,
    )
    # Legacy field, kept only so old JSON presets still load. New code uses `mode`.
    position_only: BoolProperty(default=False, options={'HIDDEN'})


class SAM3DProfile(PropertyGroup):
    """One character retarget profile: independent source/target, mapping list,
    poles, and per-character tuning. Multiple profiles allow retargeting several
    characters (e.g. from a two-character source image) in the same .blend."""
    name: StringProperty(name="Name", default="Character")
    source_armature: PointerProperty(
        name="MHR (source)",
        description="Armature to copy pose FROM (typically the SAM3D-Body FBX)",
        type=bpy.types.Object,
        poll=lambda self, obj: obj.type == 'ARMATURE',
    )
    target_armature: PointerProperty(
        name="Target",
        description="Armature to pose",
        type=bpy.types.Object,
        poll=lambda self, obj: obj.type == 'ARMATURE',
    )
    mappings: CollectionProperty(type=SAM3DMappingItem)
    active_mapping_index: IntProperty()
    source_fbx_path: StringProperty(
        name="Source FBX",
        description="Path to the SAM3D-Body FBX file. Remembered so 'Reimport source' can rebuild "
                    "the armature in place while keeping mappings/poles/modes.",
        subtype='FILE_PATH',
        default="",
    )
    l_arm_pole: StringProperty(name="L elbow pole", description="Target bone that acts as the left elbow IK pole")
    r_arm_pole: StringProperty(name="R elbow pole", description="Target bone that acts as the right elbow IK pole")
    l_leg_pole: StringProperty(name="L knee pole",  description="Target bone that acts as the left knee IK pole")
    r_leg_pole: StringProperty(name="R knee pole",  description="Target bone that acts as the right knee IK pole")
    pole_distance: bpy.props.FloatProperty(
        name="Pole distance",
        description="Distance from the elbow/knee to place the pole target, in scene units",
        default=0.3, min=0.02, max=5.0, soft_min=0.05, soft_max=1.0,
    )
    spine_bend_amplify: bpy.props.FloatProperty(
        name="Spine bend amplify",
        description=("Amplifier applied to each torso DELTA bone's pose delta "
                     "(spine chain, chest, neck, head — NOT the hip). Target "
                     "rig's natural rest curvature can partially cancel the "
                     "delta, leaving the spine straighter than the source. "
                     "Set > 1.0 to over-bend the target spine to compensate. "
                     "1.0 = no amplification. 0.0 = no rotation transferred "
                     "(target stays at rest orientation)."),
        default=1.0, min=0.0, max=5.0, soft_min=0.5, soft_max=2.5,
        update=_row_updated,
    )
    # Shape-key transfer: source is the SAM3D mesh imported from the exported
    # FBX (has 72 expr_XX shape keys plus Basis); target is any mesh on the
    # user's Rigify character (topology can differ — the operator uses
    # Surface Deform bake).
    face_shape_key_source: PointerProperty(
        name="Expr source mesh",
        description="Mesh imported from the SAM3D FBX (with 'expr_00'..'expr_71' shape keys) "
                    "that provides the face expression blendshapes",
        type=bpy.types.Object,
        poll=lambda self, obj: obj.type == 'MESH',
    )
    face_shape_key_target: PointerProperty(
        name="Expr target mesh",
        description="Mesh on the target character that should receive the transferred shape keys. "
                    "Topology can differ from the source — transfer uses Surface Deform to bake",
        type=bpy.types.Object,
        poll=lambda self, obj: obj.type == 'MESH',
    )


class SAM3DPoseCopyProps(PropertyGroup):
    """Scene-level props: list of character profiles + global copy behavior.
    Per-character data (source/target armatures, mappings, poles, etc.) lives
    on the active SAM3DProfile."""
    profiles: CollectionProperty(type=SAM3DProfile)
    active_profile_index: IntProperty(default=0)
    position_only: BoolProperty(
        name="Position only",
        description="Only copy the world position; keep the target bone's current rotation. Useful for IK targets when the source roll is off",
        default=False,
    )
    reset_before_copy: BoolProperty(
        name="Reset pose first",
        description="Clear all target pose bones (matrix_basis = identity) before applying the mapping. "
                    "Ensures each Copy starts from rest — required for toggling per-row 'pos' to take effect",
        default=True,
    )
    live_update: BoolProperty(
        name="Live update",
        description="Re-copy a single row automatically when its source, target, enable, or pos "
                    "field changes. Turn off if the auto-copy is getting in the way.",
        default=True,
    )
    auto_mode_threshold: bpy.props.FloatProperty(
        name="Auto-mode angle threshold (°)",
        description="In 'Auto-set modes from angles': rows with Y-axis mismatch <= this stay FULL; "
                    "greater becomes AIM (or AIM_ROLL for spine/head/pelvis-style bones)",
        default=30.0, min=0.0, max=180.0, soft_min=10.0, soft_max=90.0,
    )


# Module-level clipboard for copy/paste mappings across profiles.
_MAPPINGS_CLIPBOARD = []


def _prof(props):
    """Return the active SAM3DProfile, auto-creating a default one if the
    collection is empty. All per-character property access should go through
    this so operators work regardless of which profile is active.
    Draw context can't add collection items — silently returns the first
    existing profile if we can't add one there. Load handler + operator init
    ensures there's always at least one profile in normal use."""
    if len(props.profiles) == 0:
        try:
            p = props.profiles.add()
            p.name = "Character 1"
        except (RuntimeError, AttributeError):
            pass
    if len(props.profiles) == 0:
        return None
    idx = max(0, min(props.active_profile_index, len(props.profiles) - 1))
    return props.profiles[idx]


def _ensure_default_profile_for_all_scenes():
    """Guarantee every scene has at least one profile so draw + operators
    see a non-None active profile immediately.

    Blender restricts `bpy.data` during addon registration (bpy.data is a
    `_RestrictData` object at that point), so we swallow AttributeError and
    rely on the load_post / load_factory_startup_post handlers to fill in the
    default profile the next time a scene is actually loaded."""
    try:
        scenes = bpy.data.scenes
    except AttributeError:
        return
    for scene in scenes:
        props = getattr(scene, "sam3d_pose_copy", None)
        if props is not None and len(props.profiles) == 0:
            p = props.profiles.add()
            p.name = "Character 1"


@bpy.app.handlers.persistent
def _ensure_default_profile_on_load(_dummy):
    """Fires after opening a .blend or after loading factory startup (File → New)."""
    _ensure_default_profile_for_all_scenes()


# -----------------------------------------------------------------------------
# UI List
# -----------------------------------------------------------------------------

class SAM3D_UL_mappings(UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        props = context.scene.sam3d_pose_copy
        src = _prof(props).source_armature
        dst = _prof(props).target_armature

        row = layout.row(align=True)

        # Enable checkbox — kept narrow.
        en = row.row(align=True)
        en.scale_x = 0.3
        en.prop(item, "enabled", text="")

        # Source bone picker — a bit narrower so mode + axes get room.
        sub = row.row(align=True)
        sub.enabled = item.enabled
        sub.scale_x = 0.9
        if src is not None:
            valid_src = item.source_bone in src.pose.bones if item.source_bone else True
            icon_src = 'BONE_DATA' if valid_src else 'ERROR'
            sub.prop_search(item, "source_bone", src.pose, "bones", text="", icon=icon_src)
        else:
            sub.prop(item, "source_bone", text="", icon='BONE_DATA')

        arrow = row.row(align=True)
        arrow.scale_x = 0.2
        arrow.label(text="→")

        # Target bone picker.
        sub = row.row(align=True)
        sub.enabled = item.enabled
        sub.scale_x = 0.9
        if dst is not None:
            valid_dst = item.target_bone in dst.pose.bones if item.target_bone else True
            icon_dst = 'BONE_DATA' if valid_dst else 'ERROR'
            sub.prop_search(item, "target_bone", dst.pose, "bones", text="", icon=icon_dst)
        else:
            sub.prop(item, "target_bone", text="", icon='BONE_DATA')

        # Mode — enough width to show full names like "AIM_ROLL", "FINGER".
        mode = row.row(align=True)
        mode.scale_x = 0.7
        mode.prop(item, "mode", text="")

        # Source-axis and target-axis dropdowns. Only meaningful for
        # AIM / AIM_ROLL, but always shown so the dropdown is discoverable.
        axis_active = item.mode in ('AIM', 'AIM_ROLL')
        sa = row.row(align=True)
        sa.enabled = item.enabled and axis_active
        sa.scale_x = 0.55
        sa.prop(item, "source_axis", text="")
        ta = row.row(align=True)
        ta.enabled = item.enabled and axis_active
        ta.scale_x = 0.55
        ta.prop(item, "target_axis", text="")

        # Per-row test button
        op = row.operator("sam3d.copy_row", text="", icon='PLAY')
        op.index = index


# -----------------------------------------------------------------------------
# Operators
# -----------------------------------------------------------------------------

class SAM3D_OT_add_profile(Operator):
    bl_idname = "sam3d.add_profile"
    bl_label = "Add character profile"
    bl_description = "Create a new empty character profile and switch to it"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        props = context.scene.sam3d_pose_copy
        p = props.profiles.add()
        p.name = f"Character {len(props.profiles)}"
        props.active_profile_index = len(props.profiles) - 1
        return {'FINISHED'}


class SAM3D_OT_remove_profile(Operator):
    bl_idname = "sam3d.remove_profile"
    bl_label = "Remove character profile"
    bl_description = "Delete the active character profile"
    bl_options = {'REGISTER', 'UNDO'}

    def invoke(self, context, event):
        return context.window_manager.invoke_confirm(self, event)

    def execute(self, context):
        props = context.scene.sam3d_pose_copy
        if len(props.profiles) <= 1:
            self.report({'WARNING'}, "Can't remove the last profile.")
            return {'CANCELLED'}
        idx = props.active_profile_index
        if 0 <= idx < len(props.profiles):
            props.profiles.remove(idx)
            props.active_profile_index = max(0, idx - 1)
        return {'FINISHED'}


class SAM3D_OT_rename_profile(Operator):
    bl_idname = "sam3d.rename_profile"
    bl_label = "Rename character profile"
    bl_description = "Rename the active character profile"
    bl_options = {'REGISTER', 'UNDO'}

    new_name: StringProperty(name="Name", default="Character")

    def invoke(self, context, event):
        props = context.scene.sam3d_pose_copy
        active = _prof(props)
        self.new_name = active.name
        return context.window_manager.invoke_props_dialog(self)

    def execute(self, context):
        props = context.scene.sam3d_pose_copy
        active = _prof(props)
        active.name = self.new_name.strip() or f"Character {props.active_profile_index + 1}"
        return {'FINISHED'}


class SAM3D_OT_switch_profile(Operator):
    bl_idname = "sam3d.switch_profile"
    bl_label = "Switch profile"
    bl_description = "Make this character profile the active one"
    bl_options = {'REGISTER', 'UNDO'}

    index: IntProperty(default=0)

    def execute(self, context):
        props = context.scene.sam3d_pose_copy
        if 0 <= self.index < len(props.profiles):
            props.active_profile_index = self.index
        return {'FINISHED'}


class SAM3D_OT_copy_mappings(Operator):
    bl_idname = "sam3d.copy_mappings"
    bl_label = "Copy mappings"
    bl_description = ("Copy all mapping rows from the active profile to an "
                      "internal clipboard. Use Paste on another profile to "
                      "duplicate them (useful when a second character uses "
                      "the same target rig conventions).")
    bl_options = {'REGISTER'}

    def execute(self, context):
        global _MAPPINGS_CLIPBOARD
        props = context.scene.sam3d_pose_copy
        active = _prof(props)
        _MAPPINGS_CLIPBOARD = [
            {
                'source_bone': m.source_bone,
                'target_bone': m.target_bone,
                'enabled': m.enabled,
                'mode': m.mode,
                'source_axis': m.source_axis,
                'target_axis': m.target_axis,
                'rotation_offset': tuple(m.rotation_offset),
            }
            for m in active.mappings
        ]
        self.report({'INFO'}, f"Copied {len(_MAPPINGS_CLIPBOARD)} mapping rows.")
        return {'FINISHED'}


class SAM3D_OT_paste_mappings(Operator):
    bl_idname = "sam3d.paste_mappings"
    bl_label = "Paste mappings"
    bl_description = ("Replace the active profile's mapping rows with the "
                      "clipboard from Copy mappings (preserves source/target "
                      "names, modes, axes, and rotation offsets).")
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        props = context.scene.sam3d_pose_copy
        if not _MAPPINGS_CLIPBOARD:
            self.report({'WARNING'}, "Clipboard is empty — Copy mappings first.")
            return {'CANCELLED'}
        active = _prof(props)
        # Suppress per-field update callbacks — otherwise every one of the
        # ~127 rows × 6 fields would trigger _row_updated → a full pose
        # re-copy (thousands of copies for a big preset).
        with _bulk_mode():
            active.mappings.clear()
            for d in _MAPPINGS_CLIPBOARD:
                item = active.mappings.add()
                item.source_bone = d['source_bone']
                item.target_bone = d['target_bone']
                item.enabled = d['enabled']
                item.mode = d['mode']
                item.source_axis = d.get('source_axis', '+Y')
                item.target_axis = d.get('target_axis', '+Y')
                item.rotation_offset = d['rotation_offset']
            active.active_mapping_index = 0
        self.report({'INFO'}, f"Pasted {len(_MAPPINGS_CLIPBOARD)} mapping rows.")
        return {'FINISHED'}


class SAM3D_OT_add_mapping(Operator):
    bl_idname = "sam3d.add_mapping"
    bl_label = "Add mapping"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        props = context.scene.sam3d_pose_copy
        item = _prof(props).mappings.add()
        # Prefill from currently selected bones if available
        src = _prof(props).source_armature
        dst = _prof(props).target_armature
        if src is not None and context.mode == 'POSE' and src.mode == 'POSE':
            act = src.data.bones.active
            if act:
                item.source_bone = act.name
        if dst is not None and context.mode == 'POSE' and dst.mode == 'POSE':
            act = dst.data.bones.active
            if act:
                item.target_bone = act.name
        _prof(props).active_mapping_index = len(_prof(props).mappings) - 1
        return {'FINISHED'}


class SAM3D_OT_remove_mapping(Operator):
    bl_idname = "sam3d.remove_mapping"
    bl_label = "Remove mapping"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        props = context.scene.sam3d_pose_copy
        idx = _prof(props).active_mapping_index
        if 0 <= idx < len(_prof(props).mappings):
            _prof(props).mappings.remove(idx)
            _prof(props).active_mapping_index = min(idx, len(_prof(props).mappings) - 1)
        return {'FINISHED'}


class SAM3D_OT_clear_mappings(Operator):
    bl_idname = "sam3d.clear_mappings"
    bl_label = "Clear all mappings"
    bl_options = {'REGISTER', 'UNDO'}

    def invoke(self, context, event):
        return context.window_manager.invoke_confirm(self, event)

    def execute(self, context):
        props = context.scene.sam3d_pose_copy
        _prof(props).mappings.clear()
        _prof(props).active_mapping_index = 0
        return {'FINISHED'}


def _mapped_target_bone(props, mhr_name):
    """Find the target bone the given MHR source bone is mapped to (in the
    current mapping list). Returns None if not present or disabled."""
    for m in _prof(props).mappings:
        if m.source_bone == mhr_name and m.target_bone and m.enabled:
            return m.target_bone
    return None


class SAM3D_OT_flip_source_180(Operator):
    bl_idname = "sam3d.flip_source_180"
    bl_label = "Flip source 180°"
    bl_description = ("Rotate the source armature 180° around Z. Use this if Align source facing "
                      "pointed the character the wrong way — often happens when the target rig "
                      "has mirror-named bones (thigh.L on the character's right side).")
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        import math
        props = context.scene.sam3d_pose_copy
        src = _prof(props).source_armature
        if src is None:
            self.report({'ERROR'}, "Set the source armature first.")
            return {'CANCELLED'}
        src.rotation_mode = 'XYZ'
        src.rotation_euler.z += math.pi
        bpy.context.view_layer.update()

        # Re-copy so user sees the effect immediately
        if getattr(props, "live_update", True) and _prof(props).mappings:
            dst = _prof(props).target_armature
            if dst is not None:
                entries = [(m.source_bone, m.target_bone, m.enabled, m.mode, m.source_axis, m.target_axis) for m in _prof(props).mappings]
                global _LIVE_COPY_LOCK
                _LIVE_COPY_LOCK = True
                try:
                    _copy_pose(src, dst, entries, props.position_only,
                               props.reset_before_copy, lambda *a, **k: None,
                               compute_poles=True)
                finally:
                    _LIVE_COPY_LOCK = False

        self.report({'INFO'}, "Flipped source 180° around Z.")
        return {'FINISHED'}


class SAM3D_OT_scale_source_to_target(Operator):
    bl_idname = "sam3d.scale_source_to_target"
    bl_label = "Scale source to target"
    bl_description = (
        "Uniformly scale the source armature object so its LEG length (hip-to-foot in rest) "
        "matches the target's. Fixes crouching that happens when a shorter source's hip position "
        "is copied to a taller target and the legs bend to compensate. "
        "Uses REST bone positions on both, ignoring any current pose."
    )
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        props = context.scene.sam3d_pose_copy
        src = _prof(props).source_armature
        dst = _prof(props).target_armature
        if src is None or dst is None:
            self.report({'ERROR'}, "Set both source and target armatures.")
            return {'CANCELLED'}

        tgt_upleg = _mapped_target_bone(props, "l_upleg")
        # Use l_subtalar's mapped target (that's the source of foot IK in the
        # current auto-match). Fall back to l_foot's mapping if subtalar isn't
        # mapped. Aligning with a different bone than we copy causes the feet
        # to float / crouch.
        tgt_foot = (_mapped_target_bone(props, "l_subtalar")
                    or _mapped_target_bone(props, "l_foot"))
        if not tgt_upleg or not tgt_foot:
            self.report({'ERROR'}, "Need l_upleg and (l_subtalar or l_foot) mapped "
                                    "(run Auto-match, or add them manually).")
            return {'CANCELLED'}

        # Rest-pose bone positions in world space (unaffected by current pose)
        def rest_head_world(arm, name):
            b = arm.data.bones.get(name)
            if b is None:
                return None
            return arm.matrix_world @ b.head_local

        src_hip = rest_head_world(src, "l_upleg")
        # Use l_subtalar (below the ankle, closer to the sole) as the source
        # foot reference. l_foot is at the ankle joint which sits ~5cm above
        # the actual foot contact plane; aligning at the ankle leaves the
        # target's IK foot floating above its rest position by that gap.
        src_foot = rest_head_world(src, "l_subtalar") or rest_head_world(src, "l_foot")
        dst_hip = rest_head_world(dst, tgt_upleg)
        dst_foot = rest_head_world(dst, tgt_foot)
        if any(v is None for v in (src_hip, src_foot, dst_hip, dst_foot)):
            self.report({'ERROR'}, "One of the reference bones was not found on its armature.")
            return {'CANCELLED'}

        # Sum bone-segment lengths (thigh + shin + [foot-arch]) using rest
        # positions between successive joint heads. This is pose-invariant,
        # unlike the naive "hip Z minus foot Z" which shortens when knees
        # are bent in the source.
        def _chain_length(arm, bone_names):
            """Sum of distances between consecutive rest heads of the given bones."""
            heads = []
            for n in bone_names:
                b = arm.data.bones.get(n)
                if b is not None:
                    heads.append(arm.matrix_world @ b.head_local)
            if len(heads) < 2:
                return 0.0
            total = 0.0
            for i in range(len(heads) - 1):
                total += (heads[i+1] - heads[i]).length
            return total

        # For source: hip -> knee -> ankle -> subtalar (matches copy target).
        src_leg = _chain_length(src, ["l_upleg", "l_lowleg", "l_foot", "l_subtalar"])
        if src_leg < 1e-4:
            src_leg = abs(src_hip.z - src_foot.z)
        # For target: use its own thigh/shin/foot chain via the mapped bones
        # if we can find them, else fall back to hip->foot vertical.
        tgt_lowleg = _mapped_target_bone(props, "l_lowleg")
        tgt_ankle = _mapped_target_bone(props, "l_foot")
        dst_leg = _chain_length(dst, [n for n in (tgt_upleg, tgt_lowleg, tgt_ankle, tgt_foot) if n])
        if dst_leg < 1e-4:
            dst_leg = abs(dst_hip.z - dst_foot.z)
        if src_leg < 1e-4 or dst_leg < 1e-4:
            self.report({'ERROR'}, "Couldn't measure leg length on one of the rigs.")
            return {'CANCELLED'}

        # Compensate for source's current scale so we set an absolute scale
        # that lands on the desired ratio (idempotent).
        current_scale = src.scale.x if abs(src.scale.x) > 1e-6 else 1.0
        ratio = dst_leg / src_leg
        new_scale = current_scale * ratio
        src.scale = (new_scale, new_scale, new_scale)
        bpy.context.view_layer.update()

        # After scaling, translate the source armature so its l_foot rest world
        # position lands on target's mapped l_foot rest world position. That way
        # any IK foot targets copied from source land where target's foot IK is
        # supposed to be, instead of floating and forcing the legs to bend.
        # Use the same reference bone as we used for measurement.
        src_foot_bone = src.data.bones.get('l_subtalar') or src.data.bones.get('l_foot')
        if src_foot_bone is not None:
            src_foot_after = src.matrix_world @ src_foot_bone.head_local
            foot_offset = dst_foot - src_foot_after
            src.location = src.location + foot_offset
            bpy.context.view_layer.update()
        else:
            foot_offset = None

        # Re-copy so the pose lands at the new proportions
        if getattr(props, "live_update", True) and _prof(props).mappings:
            entries = [(m.source_bone, m.target_bone, m.enabled, m.mode, m.source_axis, m.target_axis) for m in _prof(props).mappings]
            global _LIVE_COPY_LOCK
            _LIVE_COPY_LOCK = True
            try:
                _copy_pose(src, dst, entries, props.position_only,
                           props.reset_before_copy, lambda *a, **k: None,
                           compute_poles=True)
            finally:
                _LIVE_COPY_LOCK = False

        offset_msg = f", foot offset {foot_offset.length:.3f}m" if foot_offset else ""
        self.report({'INFO'},
                    f"Scaled source ×{ratio:.3f} (leg {src_leg:.2f}m -> {dst_leg:.2f}m). "
                    f"Source scale: {new_scale:.3f}{offset_msg}")
        return {'FINISHED'}


def _detect_side_letter(name_lower):
    import re
    if 'left' in name_lower: return 'l'
    if 'right' in name_lower: return 'r'
    if re.search(r'(?:^|[._-])l(?:$|[._-])', name_lower): return 'l'
    if re.search(r'(?:^|[._-])r(?:$|[._-])', name_lower): return 'r'
    return None


def _frame_from_y_and_aux(y_vec, aux_vec):
    """Build a right-handed orthonormal 3x3 rotation with column 1 (Y) along
    y_vec, and column 2 (Z) in the plane of y_vec and aux_vec (projecting aux
    perpendicular to Y). Column 0 (X) = Y × Z. Returns None on degenerate inputs.
    """
    from mathutils import Matrix
    if y_vec.length < 1e-6:
        return None
    y = y_vec.normalized()
    aux_perp = aux_vec - y * aux_vec.dot(y)
    if aux_perp.length < 1e-6:
        return None
    z = aux_perp.normalized()
    x = y.cross(z)
    if x.length < 1e-6:
        return None
    x = x.normalized()
    z = x.cross(y)  # re-orthogonalize
    return Matrix((
        (x.x, y.x, z.x),
        (x.y, y.y, z.y),
        (x.z, y.z, z.z),
    ))


_FINGER_PREFIXES = ("l_thumb", "r_thumb", "l_index", "r_index",
                     "l_middle", "r_middle", "l_ring", "r_ring",
                     "l_pinky", "r_pinky")


def _is_finger_source(name):
    return name.startswith(_FINGER_PREFIXES)


def _source_palm_normal_world(src, side):
    """Palm-plane normal for the source hand, in world coords, at current pose.
    Sign is chiral per side but consistent between source and target since both
    use the same construction — chirality cancels in the delta."""
    src_world = src.matrix_world
    def _h(name):
        pb = src.pose.bones.get(name)
        return src_world @ pb.head if pb is not None else None
    w = _h(f"{side}_wrist")
    i = _h(f"{side}_index1")
    p = _h(f"{side}_pinky1")
    if w is None or i is None or p is None:
        return None
    n = (p - w).cross(i - w)
    if n.length < 1e-6:
        return None
    return n.normalized()


def _target_palm_normal_world_rest(dst, side):
    """Palm-plane normal for the target hand at REST, world coords. Uses the
    same landmark set (wrist proxy + index + pinky metacarpal bases) as the
    source construction, so chirality signs match."""
    S = side.upper()
    def _rest_head(names):
        for n in names:
            b = dst.data.bones.get(n)
            if b is not None:
                return dst.matrix_world @ b.head_local
        return None
    w = _rest_head([f"hand_ik.{S}", f"hand.{S}", f"hand_fk.{S}",
                    f"DEF-hand.{S}", f"ORG-hand.{S}"])
    i = _rest_head([f"f_index.01.{S}", f"f_index_01_{side}", f"index.01.{S}",
                    f"IndexFinger1_{side}"])
    p = _rest_head([f"f_pinky.01.{S}", f"f_pinky_01_{side}", f"pinky.01.{S}",
                    f"PinkyFinger1_{side}"])
    if w is None or i is None or p is None:
        return None
    n = (p - w).cross(i - w)
    if n.length < 1e-6:
        return None
    return n.normalized()


def _target_anat_rest_frame_world(dst, side, is_hand):
    """Compute the TARGET rig's anatomical frame at REST for the given side and
    body part. Uses actual target landmarks (finger metacarpal-base bones for
    hands, foot+toe bones for feet). Common Rigify naming patterns are tried
    first, then generic variants. Returns 3x3 Matrix or None if landmarks
    can't be located on the target armature.
    """
    from mathutils import Vector
    S = side.upper()

    def _head_rest_world(names):
        for n in names:
            b = dst.data.bones.get(n)
            if b is not None:
                return dst.matrix_world @ b.head_local
        return None

    if is_hand:
        # Wrist proxy — the "hand IK" or "hand" bone's HEAD is at the wrist joint
        # in Rigify. Falls back to plain hand names.
        # CloudRig / Sintel patterns are UPPERCASE with dashes (`IK-Hand.L`,
        # `FK-Hand.L`, `Hand.L`, `DEF-Hand.L`), Rigify uses lowercase with
        # underscores (`hand_ik.L`, `hand.L`).
        wrist_h = _head_rest_world([
            f"hand_ik.{S}", f"hand.{S}", f"hand_fk.{S}", f"DEF-hand.{S}",
            f"MCH-hand_ik.{S}", f"ORG-hand.{S}",
            f"IK-Hand.{S}", f"FK-Hand.{S}", f"Hand.{S}", f"DEF-Hand.{S}",
            f"Hand_{side}", f"hand_{side}", f"mixamorig:{S}Hand",
        ])
        # Metacarpal-base bones (f_middle.01.L etc.) — anatomically equivalent
        # to MHR's l_middle1 / l_index1 / l_pinky1. CloudRig / Sintel style
        # uses `Finger_Middle1.L` etc.
        middle_h = _head_rest_world([
            f"f_middle.01.{S}", f"f_middle_01_{side}", f"middle.01.{S}",
            f"Finger_Middle1.{S}", f"DEF-Finger_Middle1.{S}",
            f"MiddleFinger1_{side}", f"mixamorig:{S}HandMiddle1",
        ])
        index_h = _head_rest_world([
            f"f_index.01.{S}", f"f_index_01_{side}", f"index.01.{S}",
            f"Finger_Index1.{S}", f"DEF-Finger_Index1.{S}",
            f"IndexFinger1_{side}", f"mixamorig:{S}HandIndex1",
        ])
        pinky_h = _head_rest_world([
            f"f_pinky.01.{S}", f"f_pinky_01_{side}", f"pinky.01.{S}",
            f"Finger_Pinky1.{S}", f"DEF-Finger_Pinky1.{S}",
            f"PinkyFinger1_{side}", f"mixamorig:{S}HandPinky1",
        ])
        if any(v is None for v in (wrist_h, middle_h, index_h, pinky_h)):
            return None
        aux = (pinky_h - wrist_h).cross(index_h - wrist_h)
        return _frame_from_y_and_aux(middle_h - wrist_h, aux)
    else:
        # Ankle proxy — foot IK controller head is typically at the ankle/heel.
        # CloudRig/Sintel uses uppercase with dashes.
        ankle_h = _head_rest_world([
            f"foot_ik.{S}", f"foot.{S}", f"foot_fk.{S}", f"DEF-foot.{S}",
            f"MCH-foot_ik.{S}", f"ORG-foot.{S}",
            f"IK-Foot.{S}", f"FK-Foot.{S}", f"Foot.{S}", f"DEF-Foot.{S}",
            f"Foot_{side}", f"foot_{side}", f"mixamorig:{S}Foot",
        ])
        toe_h = _head_rest_world([
            f"toe.{S}", f"toe_ik.{S}", f"toe_fk.{S}", f"DEF-toe.{S}",
            f"toes.{S}", f"ORG-toe.{S}",
            f"Toes.{S}", f"FK-Toes.{S}", f"DEF-Toes.{S}",
            f"Toe_{side}", f"toe_{side}", f"mixamorig:{S}ToeBase",
        ])
        if ankle_h is None or toe_h is None:
            return None
        # Foot at rest is upright and flat on ground → up is world +Z.
        # Using world up avoids depending on a fourth foot landmark on target.
        aux = Vector((0.0, 0.0, 1.0))
        return _frame_from_y_and_aux(toe_h - ankle_h, aux)


def _canonical_ik_frame_world(canonical_rest_3x3, side, is_hand):
    """Build a canonical world-space anatomical frame for hand or foot at
    T-pose, from the character's own axes. Matches the sign convention of the
    landmark-based source frame in `_anatomical_ik_source_frame_world`, so
    delta = source_now @ canonical.inverted() is a proper world-space rotation
    representing "how this body part is rotated compared to a rest T-pose".

    canonical_rest_3x3 has columns (right, up, forward) — character axes in
    world coords. side is 'l' or 'r'. is_hand True for hand, False for foot.
    Returns a 3x3 Matrix, or None if canonical is missing.
    """
    if canonical_rest_3x3 is None:
        return None
    right = canonical_rest_3x3.col[0].to_3d()
    up = canonical_rest_3x3.col[1].to_3d()
    forward = canonical_rest_3x3.col[2].to_3d()

    if is_hand:
        # T-pose palm-forward, thumb-up. Left arm extends in -right; right arm
        # in +right. Cross(pinky-wrist, index-wrist) is dorsal (-forward) for
        # left, palmar (+forward) for right — matches source-frame chirality.
        if side == 'l':
            y = -right
            z = -forward
        else:
            y = right
            z = forward
    else:
        # Foot at rest: both point forward and stand upright. No chirality.
        y = forward
        z = up

    x = y.cross(z)
    if x.length < 1e-6:
        return None
    from mathutils import Matrix
    return Matrix((
        (x.x, y.x, z.x),
        (x.y, y.y, z.y),
        (x.z, y.z, z.z),
    ))


def _anatomical_ik_source_frame_world(target_pb, src):
    """Compute the source's CURRENT anatomical hand/foot frame in world coords
    from landmark bone HEAD positions. Independent of any rig's local
    X/Z convention. Fingers use METACARPAL BASE (l_index1 etc.), stable
    relative to the palm even when fingers curl.

    Returns (3x3 Matrix, is_hand: bool, side: str) or (None, ..., ...).
    """
    tname = target_pb.name.lower()
    if 'ik' not in tname:
        return None, None, None
    is_foot = any(w in tname for w in ('foot', 'ankle'))
    is_hand = any(w in tname for w in ('hand', 'wrist'))
    if not (is_foot or is_hand):
        return None, None, None
    side = _detect_side_letter(tname)
    if side is None:
        return None, None, None

    src_world = src.matrix_world

    def _now(name):
        pb = src.pose.bones.get(name)
        return src_world @ pb.head if pb is not None else None

    if is_hand:
        # Y = wrist → middle-finger base. aux = palm-plane normal from
        # cross(pinky→wrist, index→wrist). Chirality varies per side but
        # matches canonical construction, so delta cancels sign.
        n_anchor = f"{side}_wrist"
        n_forward = f"{side}_middle1"
        n_a = f"{side}_pinky1"
        n_b = f"{side}_index1"
    else:
        n_anchor = f"{side}_foot"
        n_forward = f"{side}_ball"
        n_a = f"{side}_subtalar"
        n_b = f"{side}_transversetarsal"

    a_now, f_now, x_now, y_now = (_now(n_anchor), _now(n_forward),
                                   _now(n_a), _now(n_b))
    if any(v is None for v in (a_now, f_now, x_now, y_now)):
        return None, is_hand, side

    if is_hand:
        aux_now = (x_now - a_now).cross(y_now - a_now)
    else:
        # Up = ankle-above-subtalar (subtalar is below foot at rest).
        aux_now = a_now - x_now

    frame = _frame_from_y_and_aux(f_now - a_now, aux_now)
    return frame, is_hand, side


def _target_hip_move_delta(src, dst, props):
    """Return the (world-space) translation the target hip bone is about to
    receive from _chain_scaled_hip_world_position, minus its rest position.
    Kept for callers that only care about translation delta."""
    hip_new_matrix = _target_hip_new_world_matrix(src, dst, props)
    if hip_new_matrix is None:
        return None
    tgt_hip_name = _mapped_target_bone(props, 'root')
    tgt_hip_bone = dst.data.bones.get(tgt_hip_name)
    hip_rest_world = dst.matrix_world @ tgt_hip_bone.head_local
    return hip_new_matrix.translation - hip_rest_world


def _target_hip_new_world_matrix(src, dst, props):
    """Return the target hip bone's ANTICIPATED world matrix after the copy
    (chain-scaled position + DELTA-rotated orientation using the fixed hip
    canonical). Used by chain-scaled foot/hand IK so they can predict where
    l_upleg / l_uparm end up when the hip both translates and rotates.

    Returns a mathutils.Matrix (4x4 world) or None if no hip mapping / chain
    scaling applies.
    """
    tgt_hip_name = _mapped_target_bone(props, 'root')
    if not tgt_hip_name:
        return None
    tgt_hip_bone = dst.data.bones.get(tgt_hip_name)
    if tgt_hip_bone is None:
        return None
    hip_new_pos = _chain_scaled_hip_world_position('root', src, dst, props)
    if hip_new_pos is None:
        return None

    # Rotation: DELTA against the fixed hip canonical (same as the DELTA
    # branch uses for source 'root').
    src_root_pb = src.pose.bones.get('root')
    if src_root_pb is None:
        return None
    canonical = Matrix((
        (-1.0, 0.0, 0.0),
        ( 0.0, 0.0, 1.0),
        ( 0.0, 1.0, 0.0),
    ))
    source_rot = (src.matrix_world @ src_root_pb.matrix).to_3x3()
    delta_3x3 = source_rot @ canonical.inverted()
    tgt_hip_rest_rot = (dst.matrix_world @ tgt_hip_bone.matrix_local).to_3x3()
    new_rot_3x3 = delta_3x3 @ tgt_hip_rest_rot

    m = new_rot_3x3.to_4x4()
    m.translation = hip_new_pos
    return m


def _chain_scaled_hip_world_position(source_bone, src, dst, props, pb=None):
    """For the source pelvis (`root` in MHR) → target hip mapping, compute
    the target bone's WORLD position by scaling source's hip elevation:

        target_hip = target_ground + source_dir *
                     (elevation_ratio × target_hip_rest_distance)

    where elevation_ratio = distance(source ground → source pelvis) divided
    by source's total leg length (upleg + lowleg + foot). A kneeling source
    with root at 50 % of its leg length lands the target's hip at 50 % of
    its own rest hip height, so the target visibly kneels regardless of
    scale differences between the two rigs.

    Detection is by SOURCE name (`root`, unambiguous MHR pelvis) rather than
    the target name, so it works no matter what the target hip bone is
    called (`torso`, `spine`, `spine_fk`, `hips`, `Pelvis`, etc.).

    Returns a world-space mathutils.Vector, or None if the mapping isn't
    the pelvis or source landmarks are missing.
    """
    if source_bone != 'root':
        return None
    # Prefer pb.bone (set when called from DELTA/POS on the hip row itself).
    # Fall back to mapping lookup for callers that don't have it (e.g. the
    # foot/hand IK compensation path via _target_hip_move_delta).
    if pb is not None:
        tgt_hip_bone = pb.bone
    else:
        tgt_hip_name = _mapped_target_bone(props, 'root')
        if not tgt_hip_name:
            print("[SAM3D hip-scale] skip: root has no target-bone mapping")
            return None
        tgt_hip_bone = dst.data.bones.get(tgt_hip_name)
        if tgt_hip_bone is None:
            print(f"[SAM3D hip-scale] skip: dst has no bone '{tgt_hip_name}'")
            return None
    src_world_pb = src.pose.bones.get('world')
    src_root_pb = src.pose.bones.get('root')
    if src_world_pb is None or src_root_pb is None:
        print("[SAM3D hip-scale] skip: source has no 'world' or 'root' pose bone")
        return None

    src_ground = src.matrix_world @ src_world_pb.head
    src_hip = src.matrix_world @ src_root_pb.head
    src_offset = src_hip - src_ground
    src_distance = src_offset.length
    if src_distance < 1e-4:
        print(f"[SAM3D hip-scale] skip: source root sits on world bone "
              f"(distance {src_distance:.4f}m)")
        return None
    src_dir = src_offset / src_distance

    # Standing hip height ≈ (root → hip socket) + full leg chain (hip
    # socket to sole). MHR has several small bones in the foot subchain
    # (l_foot → l_talocrural → l_subtalar → l_transversetarsal → l_ball)
    # that together account for the ankle-to-sole depth; leaving them
    # out inflates the elevation_ratio and floats the target hip up.
    _upleg = src.data.bones.get('l_upleg')
    _root_bone = src.data.bones.get('root')
    _leg_chain = ['l_upleg', 'l_lowleg', 'l_foot', 'l_talocrural',
                   'l_subtalar', 'l_transversetarsal', 'l_ball']
    src_leg_length = 0.0
    for _n in _leg_chain:
        _b = src.data.bones.get(_n)
        if _b is not None:
            src_leg_length += _b.length
    if _root_bone is not None and _upleg is not None:
        src_leg_length += (
            _upleg.head_local - _root_bone.head_local).length
    if src_leg_length < 1e-4:
        src_leg_length = src_distance
    # Clamp at 1.0 — target hip should never exceed rest height.
    elevation_ratio = min(1.0, src_distance / src_leg_length)

    tgt_ground = None
    tgt_world_name = _mapped_target_bone(props, 'world')
    if tgt_world_name:
        _b = dst.data.bones.get(tgt_world_name)
        if _b is not None:
            tgt_ground = dst.matrix_world @ _b.head_local
    if tgt_ground is None:
        tgt_ground = dst.matrix_world.translation.copy()

    tgt_hip_rest_world = dst.matrix_world @ tgt_hip_bone.head_local
    tgt_rest_distance = (tgt_hip_rest_world - tgt_ground).length
    if tgt_rest_distance < 1e-4:
        print(f"[SAM3D hip-scale] skip: target hip '{tgt_hip_bone.name}' "
              f"sits on target ground (distance {tgt_rest_distance:.4f}m)")
        return None

    # Use SOURCE's actual world hip position directly — this is accurate when
    # source and target rigs are at matching scale (via the addon's
    # "3. Scale to target" operator). The chain-scaling formula
    # (elevation_ratio × tgt_rest_distance) is available as a fallback but
    # tends to place the target hip a bit higher than the source's actual
    # position because sum-of-bone-lengths overestimates the vertical
    # standing hip height (the leg / foot chain isn't purely vertical).
    #
    # If rigs are DIFFERENT scales and you want proportion-preserving
    # kneeling, use "3. Scale to target" first to bring source to target's
    # scale — then direct transfer gives both correct absolute position AND
    # correct proportions.
    result = src_hip.copy()
    print(f"[SAM3D hip-scale] target hip '{tgt_hip_bone.name}': "
          f"src_hip_world={tuple(round(x,3) for x in src_hip)}, "
          f"src_dist={src_distance:.3f}m, src_leg_len={src_leg_length:.3f}m, "
          f"ratio={elevation_ratio:.3f}, "
          f"tgt_rest_dist={tgt_rest_distance:.3f}m, "
          f"rest_world={tuple(round(x,3) for x in tgt_hip_rest_world)}, "
          f"new_world={tuple(round(x,3) for x in result)}")
    return result


def _chain_scaled_ik_position(pb, target_world, src, dst, dst_world_inv, props):
    """For IK-target bones (hand_ik.*, foot_ik.*), compute a position that
    preserves TARGET's own chain length while matching source's chain
    DIRECTION. Returns a world Vector, or None if the bone isn't an IK
    target we recognize or the mapping is incomplete.

    This is the fix for cross-rig proportion mismatches: with straight
    absolute copy, a shorter source's foot ends up too close to a taller
    target's hip and legs bend. Chain-scaling places the foot at the
    same direction × target's leg length, so legs stay straight when
    source's legs are straight.
    """
    tname = pb.name.lower()
    if 'ik' not in tname:
        return None
    # Foot IK: chain root = l_upleg/r_upleg (hip).
    # Hand IK: chain root = l_clavicle/r_clavicle (shoulder).
    is_foot = any(w in tname for w in ('foot', 'ankle'))
    is_hand = any(w in tname for w in ('hand', 'wrist'))
    if not (is_foot or is_hand):
        return None

    side = _detect_side_letter(tname)
    if side is None:
        return None

    if is_foot:
        src_root_name = f"{side}_upleg"
    else:
        # Arm chain pivot is the shoulder joint = uparm head, not clavicle.
        # Using clavicle would measure arm reach from the chest, giving wrong
        # positions.
        src_root_name = f"{side}_uparm"

    src_root_pb = src.pose.bones.get(src_root_name)
    tgt_root_name = _mapped_target_bone(props, src_root_name)
    if src_root_pb is None or not tgt_root_name:
        return None
    tgt_root_bone = dst.data.bones.get(tgt_root_name)
    if tgt_root_bone is None:
        return None

    from mathutils import Vector
    # Source chain direction and current-pose distance
    src_root_world = src.matrix_world @ src_root_pb.head
    src_end_world = target_world.translation
    src_offset = src_end_world - src_root_world
    src_distance = src_offset.length
    if src_distance < 1e-4:
        return None
    src_dir = src_offset.normalized()

    # Source chain FULL length (rest) — sum of segment bone lengths so we
    # can compute how bent the source is (distance / chain_length ratio).
    if is_foot:
        src_chain_names = [f"{side}_upleg", f"{side}_lowleg", f"{side}_foot", f"{side}_subtalar"]
    else:
        src_chain_names = [f"{side}_uparm", f"{side}_lowarm", f"{side}_wrist"]
    src_heads = []
    for n in src_chain_names:
        b = src.data.bones.get(n)
        if b is not None:
            src_heads.append(src.matrix_world @ b.head_local)
    src_chain_length = 0.0
    for i in range(len(src_heads) - 1):
        src_chain_length += (src_heads[i+1] - src_heads[i]).length
    if src_chain_length < 1e-4:
        src_chain_length = src_distance  # fallback

    # Target chain length = rest distance from chain root to IK bone.
    tgt_root_world = dst.matrix_world @ tgt_root_bone.head_local
    tgt_end_rest_world = dst.matrix_world @ pb.bone.matrix_local.translation
    tgt_chain_length = (tgt_end_rest_world - tgt_root_world).length
    if tgt_chain_length < 1e-4:
        return None

    # Anchor at the SOURCE chain root's actual current world position.
    # Source's pose already encodes every torso bone's rotation (hip DELTA,
    # spine bend, chest / neck tilt, etc.) so `l_uparm.head` in world for the
    # source is exactly where the shoulder ends up. Copying that as the
    # target's chain-root anchor gives the correct starting point for IK
    # placement regardless of how the spine has been amplified — no need to
    # reconstruct the chain propagation from hip only.
    #
    # (Falls back to the target's REST chain-root when we can't get source's,
    # which is the original naive behavior.)
    tgt_root_world = src.matrix_world @ src_root_pb.head

    # Preserve bend proportion: how bent source is (as fraction of its own
    # chain length) becomes how bent target is (of its own chain length).
    bend_ratio = min(1.0, src_distance / src_chain_length)
    target_distance = bend_ratio * tgt_chain_length

    return tgt_root_world + src_dir * target_distance


def _realign_source_foot_to_target(src, dst, tgt_foot_name):
    """After changing source scale, retranslate the source armature so source's
    foot rest world position stays on target's foot rest world position."""
    src_foot_bone = src.data.bones.get('l_subtalar') or src.data.bones.get('l_foot')
    tgt_bone = dst.data.bones.get(tgt_foot_name) if tgt_foot_name else None
    if src_foot_bone is None or tgt_bone is None:
        return None
    src_foot_after = src.matrix_world @ src_foot_bone.head_local
    dst_foot = dst.matrix_world @ tgt_bone.head_local
    offset = dst_foot - src_foot_after
    src.location = src.location + offset
    bpy.context.view_layer.update()
    return offset


class SAM3D_OT_apply_master_yaw(Operator):
    bl_idname = "sam3d.apply_master_yaw"
    bl_label = "Apply source facing to master"
    bl_description = (
        "Compute source character's yaw (rotation around vertical) from hip anatomy "
        "and store it as the rotation offset on the row that targets a master/root bone. "
        "Result: target master rotates to face the same direction as source, while the "
        "master control keeps its rest orientation (flat ring)."
    )
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        import math
        from mathutils import Vector
        props = context.scene.sam3d_pose_copy
        src = _prof(props).source_armature
        dst = _prof(props).target_armature
        if src is None or dst is None:
            self.report({'ERROR'}, "Set both source and target armatures.")
            return {'CANCELLED'}

        # Source facing: from l_upleg → r_upleg vector projected to horizontal.
        def w(arm, name):
            pb = arm.pose.bones.get(name)
            return arm.matrix_world @ pb.head if pb else None
        src_l = w(src, "l_upleg"); src_r = w(src, "r_upleg")
        if src_l is None or src_r is None:
            self.report({'ERROR'}, "Source rig missing l_upleg or r_upleg.")
            return {'CANCELLED'}
        src_right = Vector((src_r.x - src_l.x, src_r.y - src_l.y, 0.0))
        if src_right.length < 1e-4:
            self.report({'ERROR'}, "Source hip axis degenerate on horizontal plane.")
            return {'CANCELLED'}
        src_right.normalize()
        # Target facing at rest: use mapped hip bones, same projection.
        tgt_l_name = _mapped_target_bone(props, "l_upleg")
        tgt_r_name = _mapped_target_bone(props, "r_upleg")
        if not tgt_l_name or not tgt_r_name:
            self.report({'ERROR'}, "l_upleg/r_upleg not mapped on target.")
            return {'CANCELLED'}
        tl = dst.data.bones.get(tgt_l_name); tr = dst.data.bones.get(tgt_r_name)
        if tl is None or tr is None:
            self.report({'ERROR'}, "Target hip bones not found.")
            return {'CANCELLED'}
        tgt_l_world = dst.matrix_world @ tl.head_local
        tgt_r_world = dst.matrix_world @ tr.head_local
        tgt_right = Vector((tgt_r_world.x - tgt_l_world.x, tgt_r_world.y - tgt_l_world.y, 0.0))
        if tgt_right.length < 1e-4:
            self.report({'ERROR'}, "Target hip axis degenerate.")
            return {'CANCELLED'}
        tgt_right.normalize()

        # Signed yaw angle from tgt_right to src_right (around +Z).
        cos_a = max(-1.0, min(1.0, tgt_right.dot(src_right)))
        yaw = math.acos(cos_a)
        cross_z = tgt_right.x * src_right.y - tgt_right.y * src_right.x
        if cross_z < 0:
            yaw = -yaw

        # Find the world→master row. Prefer source == "world"; fall back to
        # any row with a master/root-style target.
        master_names = ("root", "root.001", "master", "world")
        target_row = None
        for m in _prof(props).mappings:
            if m.enabled and m.source_bone == "world":
                target_row = m
                break
        if target_row is None:
            for m in _prof(props).mappings:
                if m.enabled and m.target_bone and m.target_bone.lower() in master_names:
                    target_row = m
                    break
        if target_row is None:
            self.report({'ERROR'}, "No enabled row targeting a master bone (world→root/master).")
            return {'CANCELLED'}

        # Set the rotation offset around Z (target's world up).
        target_row.rotation_offset = (0.0, 0.0, yaw)
        self.report({'INFO'}, f"Master row '{target_row.source_bone}→{target_row.target_bone}' "
                              f"rotation offset set to yaw {math.degrees(yaw):+.1f}° around Z.")
        return {'FINISHED'}


class SAM3D_OT_nudge_scale(Operator):
    bl_idname = "sam3d.nudge_scale"
    bl_label = "Nudge source scale"
    bl_description = ("Multiply source armature scale by a factor (default ×1.05), then re-align "
                      "source's foot to target's foot rest position. Use if the auto-scale is close "
                      "but target legs still bend — some rigs have IK bones offset from where the "
                      "mesh sole sits, so bone-chain measurement can't get it exactly right.")
    bl_options = {'REGISTER', 'UNDO'}

    factor: bpy.props.FloatProperty(default=1.05)

    def execute(self, context):
        props = context.scene.sam3d_pose_copy
        src = _prof(props).source_armature
        dst = _prof(props).target_armature
        if src is None:
            self.report({'ERROR'}, "Set the source armature first.")
            return {'CANCELLED'}
        new_scale = src.scale.x * self.factor
        src.scale = (new_scale, new_scale, new_scale)
        bpy.context.view_layer.update()

        # Re-align foot after scaling so nudge only changes the character's
        # proportions relative to the foot anchor, not the character position.
        if dst is not None:
            tgt_foot = (_mapped_target_bone(props, "l_subtalar")
                        or _mapped_target_bone(props, "l_foot"))
            if tgt_foot:
                _realign_source_foot_to_target(src, dst, tgt_foot)

        if getattr(props, "live_update", True) and dst is not None and _prof(props).mappings:
            entries = [(m.source_bone, m.target_bone, m.enabled, m.mode, m.source_axis, m.target_axis) for m in _prof(props).mappings]
            global _LIVE_COPY_LOCK
            _LIVE_COPY_LOCK = True
            try:
                _copy_pose(src, dst, entries, props.position_only,
                           props.reset_before_copy, lambda *a, **k: None,
                           compute_poles=True)
            finally:
                _LIVE_COPY_LOCK = False

        self.report({'INFO'}, f"Source scale × {self.factor:.3f} → new scale {new_scale:.3f}, "
                              f"foot re-aligned to target rest")
        return {'FINISHED'}


class SAM3D_OT_align_source_facing(Operator):
    bl_idname = "sam3d.align_source_facing"
    bl_label = "Align source facing to target"
    bl_description = (
        "Rotate the source armature object around Z so its hip axis (l_upleg -> r_upleg) "
        "aligns with the target's mapped hip bones. Run before copying if the source "
        "character is facing a different direction than the target."
    )
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        import math
        from mathutils import Vector

        props = context.scene.sam3d_pose_copy
        src = _prof(props).source_armature
        dst = _prof(props).target_armature
        if src is None or dst is None:
            self.report({'ERROR'}, "Set both source and target armatures.")
            return {'CANCELLED'}

        tgt_l = _mapped_target_bone(props, "l_upleg")
        tgt_r = _mapped_target_bone(props, "r_upleg")
        if not tgt_l or not tgt_r:
            self.report({'ERROR'}, "Need l_upleg and r_upleg entries in the mapping "
                                    "to compute facing (add them or run Auto-match first).")
            return {'CANCELLED'}

        def world_head(arm, name):
            pb = arm.pose.bones.get(name)
            if pb is None:
                return None
            return arm.matrix_world @ pb.head

        src_l = world_head(src, "l_upleg")
        src_r = world_head(src, "r_upleg")
        dst_l = world_head(dst, tgt_l)
        dst_r = world_head(dst, tgt_r)
        if any(v is None for v in (src_l, src_r, dst_l, dst_r)):
            missing = [n for n, v in [
                (f"source '{src_l is None and 'l_upleg' or ''}'", src_l),
                (f"source '{src_r is None and 'r_upleg' or ''}'", src_r),
                (f"target '{tgt_l}'", dst_l),
                (f"target '{tgt_r}'", dst_r),
            ] if v is None]
            self.report({'ERROR'}, "Missing bones: " + ", ".join(missing))
            return {'CANCELLED'}

        # Right vector on the horizontal plane
        src_right = Vector((src_r.x - src_l.x, src_r.y - src_l.y, 0.0))
        dst_right = Vector((dst_r.x - dst_l.x, dst_r.y - dst_l.y, 0.0))
        if src_right.length < 1e-4 or dst_right.length < 1e-4:
            self.report({'ERROR'}, "Hip axis is degenerate on the horizontal plane "
                                    "(character lying flat?). Manual alignment needed.")
            return {'CANCELLED'}
        src_right.normalize()
        dst_right.normalize()

        # Signed yaw angle from src_right to dst_right (around +Z)
        cos_a = max(-1.0, min(1.0, src_right.dot(dst_right)))
        angle = math.acos(cos_a)
        cross_z = src_right.x * dst_right.y - src_right.y * dst_right.x
        if cross_z < 0:
            angle = -angle

        if abs(angle) < 1e-4:
            self.report({'INFO'}, "Source already aligned (delta < 0.01°).")
            return {'FINISHED'}

        # Rotate source armature object around its own origin, on the Z axis.
        src.rotation_mode = 'XYZ'
        src.rotation_euler.z += angle

        # Force the depsgraph to re-evaluate so any subsequent copy reads the
        # source's world matrices with the new rotation applied.
        bpy.context.view_layer.update()

        # If live update is on, re-copy the whole pose so the user immediately
        # sees the pose in the new facing direction. Without this, the target
        # still shows the pose computed before the rotation.
        if getattr(props, "live_update", True) and _prof(props).mappings:
            entries = [(m.source_bone, m.target_bone, m.enabled, m.mode, m.source_axis, m.target_axis) for m in _prof(props).mappings]
            global _LIVE_COPY_LOCK
            _LIVE_COPY_LOCK = True
            try:
                _copy_pose(src, dst, entries, props.position_only,
                           props.reset_before_copy, lambda *a, **k: None,
                           compute_poles=True)
            finally:
                _LIVE_COPY_LOCK = False

        deg = math.degrees(angle)
        warn = ""
        if abs(deg) > 135:
            warn = (f" — rotation was {deg:+.0f}°, close to 180°. If the character now "
                    f"faces backward, your target rig may use mirror-named bones "
                    f"(l_/r_ swapped). Use 'Flip source 180°' to correct.")
        self.report({'WARNING' if warn else 'INFO'},
                    f"Rotated source by {deg:+.1f}° around Z"
                    + (" and re-copied pose." if getattr(props, "live_update", True) and _prof(props).mappings else "")
                    + warn)
        return {'FINISHED'}


class SAM3D_OT_add_all_source_bones(Operator):
    bl_idname = "sam3d.add_all_source_bones"
    bl_label = "Add all source bones"
    bl_description = ("Add a mapping row for every bone on the source armature "
                      "that isn't already in the list. Empty target — fill in manually.")
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        props = context.scene.sam3d_pose_copy
        src = _prof(props).source_armature
        if src is None:
            self.report({'ERROR'}, "Set the source armature first.")
            return {'CANCELLED'}

        # Existing sources (across all mappings — a source can appear more than
        # once when it's wired to both IK and FK targets, that's fine to leave alone).
        existing_sources = {m.source_bone for m in _prof(props).mappings if m.source_bone}

        added = 0
        with _bulk_mode():
            for b in src.data.bones:
                if b.name in existing_sources:
                    continue
                item = _prof(props).mappings.add()
                item.source_bone = b.name
                item.target_bone = ""
                item.enabled = False
                item.mode = 'FULL'
                added += 1

        # Reorder everything so newly-added rows land in anatomical order,
        # not appended at the end.
        _sort_mappings_by_source_order(props, src)

        self.report({'INFO'}, f"Added {added} source bones (disabled — enable and set target per row).")
        return {'FINISHED'}


class SAM3D_OT_auto_match(Operator):
    bl_idname = "sam3d.auto_match"
    bl_label = "Auto-match to target"
    bl_description = ("Scan the target armature and build a mapping by trying common bone-name variants "
                      "(plain names first, then Rigify FK/IK, then Mixamo, then Unreal-style). "
                      "Replaces the current list.")
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        import re
        props = context.scene.sam3d_pose_copy
        dst = _prof(props).target_armature
        src = _prof(props).source_armature
        if dst is None:
            self.report({'ERROR'}, "Set the target armature first.")
            return {'CANCELLED'}

        target_bones = {pb.name for pb in dst.pose.bones}
        target_bones_lower = {pb.name.lower(): pb.name for pb in dst.pose.bones}

        position_only_targets = {"root", "root.001", "master", "world"}

        # For each hand/foot MHR bone, describe body-part keywords and expected side.
        # We use this to find IK variants regardless of the exact separator convention
        # ("hand_ik.L", "hand.ik.L", "hand_IK.L", "IK_hand.L", "LeftHandIK" all match).
        ik_scan_specs = {
            "l_wrist":    (["hand", "wrist"], "l"),
            "r_wrist":    (["hand", "wrist"], "r"),
            # Send l_subtalar to foot IK (subtalar sits closer to the target IK
            # bone height than l_foot which is at the ankle). l_foot is still
            # available for FK via the synonym list below.
            "l_subtalar": (["foot", "ankle"], "l"),
            "r_subtalar": (["foot", "ankle"], "r"),
        }

        def _detect_side(name_lower):
            if "left" in name_lower: return "l"
            if "right" in name_lower: return "r"
            if re.search(r"(?:^|[._-])l(?:$|[._-])", name_lower): return "l"
            if re.search(r"(?:^|[._-])r(?:$|[._-])", name_lower): return "r"
            return None

        # Rigify (and similar) internal-only bones we should never map to.
        internal_prefixes = ("DEF-", "MCH-", "ORG-", "WGT-", "VIS-")
        sorted_target_bones = sorted(target_bones)  # deterministic iteration order

        def _scan_for_variant(body_words, wanted_side, require_token, exclude_token=None):
            """Find any target bone name containing require_token, a body word, and the wanted side.
            Higher score wins; ties broken by shorter name (cleaner)."""
            candidates = []
            for actual in sorted_target_bones:
                if any(actual.startswith(p) for p in internal_prefixes):
                    continue
                low = actual.lower()
                if require_token and require_token not in low:
                    continue
                if exclude_token and exclude_token in low:
                    continue
                if not any(w in low for w in body_words):
                    continue
                if _detect_side(low) != wanted_side:
                    continue
                if require_token == "ik":
                    # IK scan: prefer shorter names (e.g. "hand_ik.L" over "MCH-hand_ik_target.L")
                    score = 100 - len(actual)
                elif "fk" in low:
                    score = 30
                elif not any(k in low for k in ("ik", "def", "mch", "org", "ctrl", "wgt")):
                    score = 20  # plain-named bone (deform-style if no fk/ik split exists)
                else:
                    score = 10
                candidates.append((score, len(actual), actual))
            if not candidates:
                return None
            candidates.sort(key=lambda x: (-x[0], x[1]))  # highest score, then shortest
            return candidates[0][2]

        def _resolve(name):
            if name in target_bones:
                return name
            return target_bones_lower.get(name.lower())

        matched = 0
        skipped = []
        added_targets = set()

        def _add(mhr, tgt):
            nonlocal matched
            if not tgt or tgt in added_targets:
                return
            added_targets.add(tgt)
            item = _prof(props).mappings.add()
            item.source_bone = mhr
            item.target_bone = tgt
            item.enabled = True
            # Master/root-style targets: default to DELTA so the master stays
            # at its own rest position (POS would drag it to source's world
            # position, which pushes it off the floor when we translate source
            # for foot alignment).
            item.mode = 'DELTA' if tgt.lower() in position_only_targets else 'FULL'
            matched += 1

        with _bulk_mode():
            _prof(props).mappings.clear()
            for mhr_name, candidates in MHR_TARGET_SYNONYMS.items():
                # Hands/feet: prefer any target bone containing "ik" + body part + side.
                # This wins over the synonym list, so custom IK naming (hand.ik.L,
                # IK_hand.L, LeftHandIK, ...) is picked up even if we didn't list it.
                if mhr_name in ik_scan_specs:
                    body_words, wanted_side = ik_scan_specs[mhr_name]
                    ik_hit = _scan_for_variant(body_words, wanted_side, require_token="ik")
                    _add(mhr_name, ik_hit)
                    fk_hit = _scan_for_variant(body_words, wanted_side, require_token=None, exclude_token="ik")
                    _add(mhr_name, fk_hit)
                    if ik_hit or fk_hit:
                        continue
                    # If neither scan found anything, fall through to the synonym list.

                # Fallback: first candidate from the synonym list that exists on target.
                chosen = None
                for cand in candidates:
                    actual = _resolve(cand)
                    if actual is not None:
                        chosen = actual
                        break
                if chosen is None:
                    skipped.append(mhr_name)
                    continue
                _add(mhr_name, chosen)

            # Detect Rigify-style IK pole targets. Only overwrite if we find a
            # match — preserves any manually-picked pole a user set previously.
            pole_candidates = {
                "l_arm_pole": ("upper_arm_ik_target.L", "elbow_pole.L", "elbow_target.L", "pole_arm.L"),
                "r_arm_pole": ("upper_arm_ik_target.R", "elbow_pole.R", "elbow_target.R", "pole_arm.R"),
                "l_leg_pole": ("thigh_ik_target.L", "knee_pole.L", "knee_target.L", "pole_leg.L"),
                "r_leg_pole": ("thigh_ik_target.R", "knee_pole.R", "knee_target.R", "pole_leg.R"),
            }
            pole_hits = 0
            for prop_name, cands in pole_candidates.items():
                for c in cands:
                    r = _resolve(c)
                    if r is not None:
                        setattr(props, prop_name, r)
                        pole_hits += 1
                        break
                # Else: leave the current value alone (may be user-set).

            # Always add rows for the core anatomical MHR bones even if unmatched,
            # so the user can see them and manually pick a target.
            core_bones_always_visible = [
                "world", "root",
                "c_spine0", "c_spine1", "c_spine2", "c_spine3",
                "c_neck", "c_head",
                "l_clavicle", "l_uparm", "l_lowarm", "l_wrist",
                "r_clavicle", "r_uparm", "r_lowarm", "r_wrist",
                "l_upleg", "l_lowleg", "l_foot", "l_subtalar", "l_ball",
                "r_upleg", "r_lowleg", "r_foot", "r_subtalar", "r_ball",
            ]
            existing_sources = {m.source_bone for m in _prof(props).mappings if m.source_bone}
            for core in core_bones_always_visible:
                if core in existing_sources:
                    continue
                item = _prof(props).mappings.add()
                item.source_bone = core
                item.target_bone = ""
                item.enabled = False
                item.mode = 'POS' if core.lower() in position_only_targets else 'FULL'
            # Re-sort mappings into source-armature bone order (anatomical order for MHR).
            _sort_mappings_by_source_order(props, src)

            _prof(props).active_mapping_index = 0

        msg = f"Auto-matched {matched}/{len(MHR_TARGET_SYNONYMS)} bones"
        if pole_hits:
            msg += f", {pole_hits}/4 IK poles"
        if skipped:
            print("[SAM3D Pose Copy] No target match found for:", skipped)
            msg += f" ({len(skipped)} unmatched — see console)"
        self.report({'INFO'}, msg)
        return {'FINISHED'}


class SAM3D_OT_load_rigify_edit_preset(Operator):
    bl_idname = "sam3d.load_rigify_edit_preset"
    bl_label = "Load Rigify (edit) preset"
    bl_description = ("Populate the mapping with the legacy Rigify defaults — "
                      "kept for compatibility with the older mode assignments "
                      "(FULL for most rows, DELTA for master/root). For a "
                      "vanilla Blender Rigify Human rig, prefer the (standard) "
                      "preset instead.")
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        props = context.scene.sam3d_pose_copy
        position_only_targets = {"root", "root.001", "master", "world"}
        with _bulk_mode():
            _prof(props).mappings.clear()
            _prof(props).l_arm_pole = "upper_arm_ik_target.L"
            _prof(props).r_arm_pole = "upper_arm_ik_target.R"
            _prof(props).l_leg_pole = "thigh_ik_target.L"
            _prof(props).r_leg_pole = "thigh_ik_target.R"
            for source_bone, target_bone in RIGIFY_EDIT_MAPPING:
                item = _prof(props).mappings.add()
                item.source_bone = source_bone
                item.target_bone = target_bone
                item.enabled = True
                item.mode = 'DELTA' if target_bone.lower() in position_only_targets else 'FULL'
            _prof(props).active_mapping_index = 0
        self.report({'INFO'}, f"Loaded {len(RIGIFY_EDIT_MAPPING)} Rigify (edit) mappings.")
        return {'FINISHED'}


class SAM3D_OT_load_rigify_standard_preset(Operator):
    bl_idname = "sam3d.load_rigify_standard_preset"
    bl_label = "Load Rigify (standard) preset"
    bl_description = ("Populate the mapping to match a vanilla Blender Rigify "
                      "Human rig (generated from Add → Armature → Human Meta-Rig, "
                      "then Rigify Buttons → Generate). Uses the per-body-part "
                      "mode conventions: DELTA for torso, AIM_ROLL for limbs, "
                      "FINGER for fingers, POS for IK end effectors.")
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        props = context.scene.sam3d_pose_copy
        with _bulk_mode():
            _prof(props).mappings.clear()
            _prof(props).l_arm_pole = "upper_arm_ik_target.L"
            _prof(props).r_arm_pole = "upper_arm_ik_target.R"
            _prof(props).l_leg_pole = "thigh_ik_target.L"
            _prof(props).r_leg_pole = "thigh_ik_target.R"
            for entry in RIGIFY_STANDARD_MAPPING:
                if len(entry) == 5:
                    source_bone, target_bone, mode, src_axis, tgt_axis = entry
                else:
                    source_bone, target_bone, mode = entry
                    src_axis, tgt_axis = '+Y', '+Y'
                item = _prof(props).mappings.add()
                item.source_bone = source_bone
                item.target_bone = target_bone
                item.enabled = (mode != 'SKIP')
                item.mode = mode
                item.source_axis = src_axis
                item.target_axis = tgt_axis
            _prof(props).active_mapping_index = 0
        self.report({'INFO'}, f"Loaded {len(RIGIFY_STANDARD_MAPPING)} Rigify (standard) mappings.")
        return {'FINISHED'}


class SAM3D_OT_load_sintel_preset(Operator):
    bl_idname = "sam3d.load_sintel_preset"
    bl_label = "Load Sintel (CloudRig) preset"
    bl_description = ("Populate the mapping with CloudRig Sintel-metarig defaults "
                      "(replaces current list). Assumes the target rig was generated "
                      "from the CloudRig Sintel sample or uses the same FK-/IK-* "
                      "naming conventions.")
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        props = context.scene.sam3d_pose_copy
        with _bulk_mode():
            _prof(props).mappings.clear()
            _prof(props).l_arm_pole = "POLE-Arm.L"
            _prof(props).r_arm_pole = "POLE-Arm.R"
            _prof(props).l_leg_pole = "POLE-Leg.L"
            _prof(props).r_leg_pole = "POLE-Leg.R"
            for entry in SINTEL_DEFAULT_MAPPING:
                # Rows may be 3-tuple (source, target, mode) or 5-tuple
                # (source, target, mode, source_axis, target_axis).
                if len(entry) == 5:
                    source_bone, target_bone, mode, src_axis, tgt_axis = entry
                else:
                    source_bone, target_bone, mode = entry
                    src_axis, tgt_axis = '+Y', '+Y'
                item = _prof(props).mappings.add()
                item.source_bone = source_bone
                item.target_bone = target_bone
                item.enabled = (mode != 'SKIP')
                item.mode = mode
                item.source_axis = src_axis
                item.target_axis = tgt_axis
            _prof(props).active_mapping_index = 0
        self.report({'INFO'}, f"Loaded {len(SINTEL_DEFAULT_MAPPING)} Sintel default mappings.")
        return {'FINISHED'}


class SAM3D_OT_save_preset(Operator, ExportHelper):
    bl_idname = "sam3d.save_preset"
    bl_label = "Save mapping to file"
    filename_ext = ".json"
    filter_glob: StringProperty(default="*.json", options={'HIDDEN'})

    def execute(self, context):
        props = context.scene.sam3d_pose_copy
        data = [
            {"source": m.source_bone, "target": m.target_bone,
             "enabled": m.enabled, "mode": m.mode,
             "source_axis": m.source_axis, "target_axis": m.target_axis,
             "rotation_offset": list(m.rotation_offset)}
            for m in _prof(props).mappings
        ]
        with open(self.filepath, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
        self.report({'INFO'}, f"Saved {len(data)} mappings to {os.path.basename(self.filepath)}")
        return {'FINISHED'}


class SAM3D_OT_load_preset(Operator, ImportHelper):
    bl_idname = "sam3d.load_preset"
    bl_label = "Load mapping from file"
    filename_ext = ".json"
    filter_glob: StringProperty(default="*.json", options={'HIDDEN'})

    def execute(self, context):
        with open(self.filepath, "r", encoding="utf-8") as f:
            data = json.load(f)
        props = context.scene.sam3d_pose_copy
        with _bulk_mode():
            _prof(props).mappings.clear()
            for entry in data:
                item = _prof(props).mappings.add()
                item.source_bone = entry.get("source", "")
                item.target_bone = entry.get("target", "")
                item.enabled = bool(entry.get("enabled", True))
                # Migrate legacy 'position_only' bool to new 'mode' enum.
                if "mode" in entry:
                    valid_modes = {'FULL', 'AIM', 'AIM_ROLL', 'DELTA', 'FINGER', 'POS', 'POS_RAW'}
                    item.mode = entry["mode"] if entry["mode"] in valid_modes else 'FULL'
                else:
                    item.mode = 'POS' if bool(entry.get("position_only", False)) else 'FULL'
                item.source_axis = entry.get("source_axis", "+Y")
                item.target_axis = entry.get("target_axis", "+Y")
                ro = entry.get("rotation_offset")
                if ro and len(ro) == 3:
                    item.rotation_offset = (float(ro[0]), float(ro[1]), float(ro[2]))
            _prof(props).active_mapping_index = 0
        self.report({'INFO'}, f"Loaded {len(data)} mappings from {os.path.basename(self.filepath)}")
        return {'FINISHED'}


class SAM3D_OT_copy_row(Operator):
    bl_idname = "sam3d.copy_row"
    bl_label = "Copy this row only"
    bl_options = {'REGISTER', 'UNDO'}
    index: IntProperty()

    def execute(self, context):
        props = context.scene.sam3d_pose_copy
        if not (0 <= self.index < len(_prof(props).mappings)):
            return {'CANCELLED'}
        m = _prof(props).mappings[self.index]
        # Per-row "test" copy — never reset (would wipe the pose in progress).
        applied, ms, mt = _copy_pose(
            _prof(props).source_armature,
            _prof(props).target_armature,
            [(m.source_bone, m.target_bone, True, m.mode, m.source_axis, m.target_axis)],
            props.position_only,
            False,
            self.report,
        )
        if applied:
            self.report({'INFO'}, f"Applied: {m.source_bone} → {m.target_bone}")
        else:
            missing = []
            if ms: missing.append(f"source '{ms[0]}'")
            if mt: missing.append(f"target '{mt[0]}'")
            self.report({'WARNING'}, f"Nothing applied: {', '.join(missing) if missing else 'no valid mapping'}")
        return {'FINISHED'}


class SAM3D_OT_copy_all(Operator):
    bl_idname = "sam3d.copy_all"
    bl_label = "Copy all enabled"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        props = context.scene.sam3d_pose_copy
        entries = [(m.source_bone, m.target_bone, m.enabled, m.mode, m.source_axis, m.target_axis) for m in _prof(props).mappings]
        applied, ms, mt = _copy_pose(
            _prof(props).source_armature,
            _prof(props).target_armature,
            entries,
            props.position_only,
            props.reset_before_copy,
            self.report,
        )
        parts = [f"Copied {applied}"]
        if ms:
            parts.append(f"source-missing {len(ms)}")
        if mt:
            parts.append(f"target-missing {len(mt)}")
        if ms:
            print("[SAM3D Pose Copy] source bones not found:", ms)
        if mt:
            print("[SAM3D Pose Copy] target bones not found:", mt)
        self.report({'INFO'}, ", ".join(parts))
        return {'FINISHED'}


class SAM3D_OT_diagnose(Operator):
    bl_idname = "sam3d.diagnose"
    bl_label = "Diagnose rest-pose alignment"
    bl_description = ("Per mapping row: rest-frame Y and Z axis angles between source and target "
                      "(explains twist / FULL misbehaviour), plus current world-space head "
                      "position drift (explains 'target didn't reach source pose'). Prints a "
                      "sorted table and a per-row recommendation to the system console.")

    def execute(self, context):
        import math
        from mathutils import Vector
        props = context.scene.sam3d_pose_copy
        src = _prof(props).source_armature
        dst = _prof(props).target_armature
        if src is None or dst is None:
            self.report({'ERROR'}, "Set both armatures first.")
            return {'CANCELLED'}

        def _axis_angle_deg(a, b):
            an, bn = a.copy(), b.copy()
            if an.length < 1e-6 or bn.length < 1e-6:
                return None
            an.normalize(); bn.normalize()
            return math.degrees(math.acos(max(-1.0, min(1.0, an.dot(bn)))))

        def _suggest(y_ang, z_ang, pos_drift_cm, mode, src_pos_only_target,
                     is_finger, is_torso):
            """Turn measurements into a short human-readable recommendation.

            Note on why Y/Z angles don't matter for every mode:
              - DELTA on a torso bone uses a fixed world-space canonical, so
                the source bone's rest orientation doesn't need to match the
                target's — big Y/Z° is expected and irrelevant.
              - AIM / AIM_ROLL compute a fresh rotation from world Y directions,
                so rest-frame mismatch doesn't affect the result.
              - FULL is the one mode that DOES need rest axes to agree, so
                that's where Y/Z° angles become actionable.
            """
            if src_pos_only_target and mode != 'POS':
                return "IK / end-effector — switch to POS"
            if y_ang is None:
                return "degenerate axis; skip"
            # DELTA outside the torso set is a design mismatch — it needs the
            # canonical Y-up torso rest to make sense.
            if mode == 'DELTA' and not is_torso:
                return "DELTA is torso-only — switch to AIM_ROLL (limbs) or AIM (fingers)"
            # Suggest FINGER for finger sources on any non-FINGER mode.
            if is_finger and mode not in ('FINGER', 'SKIP'):
                return "switch to FINGER (palm-normal transfer preserves curl)"
            # FULL only needs rest axes to agree for the plain-copy path
            # (i.e. non-finger sources; FINGER now owns the special handling).
            if mode == 'FULL':
                if y_ang > 60:
                    return f"Y axes {y_ang:.0f}° apart — switch to AIM_ROLL"
                if z_ang is not None and z_ang > 30:
                    target_mode = 'DELTA' if is_torso else 'AIM_ROLL'
                    return f"roll ~{z_ang:.0f}° off — switch to {target_mode}"
            # POS drift for IK end effectors: some drift is EXPECTED when
            # source and target have different arm-to-leg proportions, because
            # chain-scaled IK places the target at target's arm length in the
            # source's direction, not at source's raw wrist position. Only
            # flag really large drifts (> 25cm) as likely-actionable.
            if pos_drift_cm is not None and pos_drift_cm > 25.0 and mode == 'POS':
                return (f"end-effector drifted {pos_drift_cm:.1f}cm — check src "
                        f"armature scale / facing, or try POS_RAW to skip chain scaling")
            # Modes that are convention-agnostic (DELTA-on-torso, AIM, AIM_ROLL, POS-with-small-drift):
            # numeric angles look ugly but don't imply a mode change.
            return "OK"

        # Rough bone-type classifiers used only for suggestion text.
        _torso_srcs = {"world", "root", "c_spine0", "c_spine1", "c_spine2",
                       "c_spine3", "c_neck", "c_head", "c_jaw",
                       "l_eye", "r_eye"}
        def _is_finger(src_name):
            return any(k in src_name for k in
                       ("thumb", "index", "middle", "ring", "pinky"))
        def _is_torso(src_name):
            return src_name in _torso_srcs

        pos_only_targets = {"root", "root.001", "master", "world"}

        rows = []
        for m in _prof(props).mappings:
            if not (m.enabled and m.source_bone and m.target_bone):
                continue
            src_pb = src.pose.bones.get(m.source_bone)
            dst_pb = dst.pose.bones.get(m.target_bone)
            if src_pb is None or dst_pb is None:
                continue

            # Rest frames in world space — these tell you what CONVENTIONS the
            # two bones live in, independent of any pose. Post-mismatch here is
            # a mode-selection issue.
            src_rest_world = src.matrix_world @ src_pb.bone.matrix_local
            dst_rest_world = dst.matrix_world @ dst_pb.bone.matrix_local
            y_ang = _axis_angle_deg(src_rest_world.col[1].to_3d(),
                                    dst_rest_world.col[1].to_3d())
            z_ang = _axis_angle_deg(src_rest_world.col[2].to_3d(),
                                    dst_rest_world.col[2].to_3d())

            # Post-pose world head positions — where each bone actually is right
            # NOW. Drift here after a copy = position/chain-scaling issue.
            src_head_w = src.matrix_world @ src_pb.head
            dst_head_w = dst.matrix_world @ dst_pb.head
            pos_drift_cm = (src_head_w - dst_head_w).length * 100.0

            tgt_is_end_effector = (
                'ik' in m.target_bone.lower() or 'foot' in m.target_bone.lower()
                and m.target_bone.lower().startswith(('ik', 'foot_ik', 'hand_ik'))
            )
            suggestion = _suggest(
                y_ang, z_ang, pos_drift_cm, m.mode,
                m.target_bone.lower() in pos_only_targets or tgt_is_end_effector,
                _is_finger(m.source_bone),
                _is_torso(m.source_bone),
            )
            rows.append((y_ang or 0.0, z_ang or 0.0, pos_drift_cm,
                         m.source_bone, m.target_bone, m.mode, suggestion))

        # Keep rows in UI order (mapping list order) so the console lines up
        # with what the user sees in the panel — makes correlating a specific
        # row to its diagnosis much easier than a worst-first sort.

        print()
        print("[SAM3D Pose Copy] Diagnose ─ per-row rest & pose analysis")
        print("=" * 108)
        print(f"  {'Y°':>4}  {'Z°':>4}  {'drift':>6}  {'src':<16}  {'tgt':<26}  {'mode':<6}  suggestion")
        print("  " + "-" * 104)
        for y, z, d, s, t, mo, sug in rows:
            drift = f"{d:5.1f}cm" if d is not None else "  n/a "
            print(f"  {y:4.0f}  {z:4.0f}  {drift}  {s:<16}  {t:<26}  {mo:<6}  {sug}")

        # Summary rollup
        bad_y = sum(1 for r in rows if r[0] > 45)
        bad_z = sum(1 for r in rows if r[1] > 30)
        bad_d = sum(1 for r in rows if (r[2] or 0) > 5)
        needs_change = sum(1 for r in rows if r[6] != "OK")
        print("  " + "-" * 104)
        print(f"  Summary: {len(rows)} enabled rows, "
              f"{bad_y} Y>45°, {bad_z} Z>30°, {bad_d} drift>5cm, "
              f"{needs_change} rows with a suggested change.")
        print()
        self.report({'INFO'}, f"Diagnosed {len(rows)} rows; {needs_change} have a suggested fix. "
                              f"See console (Window → Toggle System Console).")
        return {'FINISHED'}


class SAM3D_OT_auto_set_modes(Operator):
    bl_idname = "sam3d.auto_set_modes"
    bl_label = "Auto-set modes from angles"
    bl_description = ("Set each row's mode based on the rest-pose Y-axis mismatch angle. "
                      "Angle <= 30° → FULL (rest-compatible copy). "
                      "30–90° → AIM (aim Y-axis, preserve target rest for other axes). "
                      "90–180° → AIM (large mismatch; AIM is still the right handling). "
                      "Master/root-style targets are forced to POS regardless.")
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        import math
        props = context.scene.sam3d_pose_copy
        src = _prof(props).source_armature
        dst = _prof(props).target_armature
        if src is None or dst is None:
            self.report({'ERROR'}, "Set both armatures first.")
            return {'CANCELLED'}

        pos_names = {"root", "root.001", "master", "world"}
        # Sources that map to torso controls — use DELTA (rotate relative to
        # anatomical canonical rest, respect target's rest orientation).
        delta_sources = {
            "root", "c_spine0", "c_spine1", "c_spine2", "c_spine3",
            "c_neck", "c_head",
        }
        # Finger source bones — regardless of Y-axis angle (curled fingers
        # produce huge angles that aren't convention mismatch), use FULL.
        finger_prefixes = ("l_thumb", "r_thumb", "l_index", "r_index",
                           "l_middle", "r_middle", "l_ring", "r_ring",
                           "l_pinky", "r_pinky")
        # IK target substrings — for these, POS mode (position only) is safer
        # because AIM/FULL rotate the target and can flip hands/feet upside
        # down when source bone Y direction doesn't match target IK Y.
        ik_target_hints = ("_ik.", ".ik.", "_ik_", "ik.l", "ik.r")

        def _is_ik_target(name):
            n = name.lower()
            return any(h in n for h in ik_target_hints)
        threshold = props.auto_mode_threshold
        counts = {'FULL': 0, 'AIM': 0, 'AIM_ROLL': 0, 'DELTA': 0, 'POS': 0}
        z_flipped_count = 0  # rows where auto-detect set target_axis to -Y
        with _bulk_mode():
            for m in _prof(props).mappings:
                if not (m.enabled and m.source_bone and m.target_bone):
                    continue
                src_pb = src.pose.bones.get(m.source_bone)
                dst_pb = dst.pose.bones.get(m.target_bone)
                if src_pb is None or dst_pb is None:
                    continue
                if m.target_bone.lower() in pos_names:
                    m.mode = 'POS'
                    counts['POS'] += 1
                    continue
                if _is_ik_target(m.target_bone):
                    # IK controls: position drives the chain; rotation often
                    # comes out wrong (feet upside down, hand twisted) when
                    # copied from source. Position-only + IK solver is safer.
                    m.mode = 'POS'
                    counts['POS'] += 1
                    continue
                if m.source_bone in ("l_eye", "r_eye"):
                    # Eyes: SKIP so they stay entirely at rest (in the sockets)
                    # and follow the head bone via chain.
                    m.mode = 'SKIP'
                    counts.setdefault('SKIP', 0)
                    counts['SKIP'] += 1
                    continue
                if m.source_bone in ("l_thumb1", "r_thumb1"):
                    # Thumb base (metacarpal): MHR's l_thumb1 sits at CMC joint
                    # which is anatomically different from most rigs' thumb.01
                    # (at MCP joint). Copying rotation looks off; SKIP so the
                    # thumb base stays at rest and thumb.02/.03 do the posing.
                    m.mode = 'SKIP'
                    counts.setdefault('SKIP', 0)
                    counts['SKIP'] += 1
                    continue
                if m.source_bone in delta_sources:
                    # Torso: always use DELTA — Y-axis angle isn't meaningful
                    # here (torso rest orientation differs across rigs).
                    m.mode = 'DELTA'
                    counts['DELTA'] += 1
                    continue
                if m.source_bone.startswith(finger_prefixes):
                    # Fingers: FULL — big angles are from curled pose, not rest mismatch.
                    m.mode = 'FULL'
                    counts['FULL'] += 1
                    continue
                src_world = src.matrix_world @ src_pb.matrix
                dst_rest_world = dst.matrix_world @ dst_pb.bone.matrix_local
                src_y = src_world.col[1].to_3d()
                dst_y = dst_rest_world.col[1].to_3d()
                if src_y.length < 1e-6 or dst_y.length < 1e-6:
                    continue
                src_y.normalize(); dst_y.normalize()
                cos_a = max(-1.0, min(1.0, src_y.dot(dst_y)))
                angle = math.degrees(math.acos(cos_a))
                if angle <= threshold:
                    m.mode = 'FULL'
                else:
                    m.mode = 'AIM'
                counts[m.mode] += 1

                # Detect Z-axis convention mismatch. If source rest Z and target
                # rest Z point roughly opposite (dot < 0), the two rigs use
                # opposite Z conventions — flip the target-axis dropdown to
                # -Y so the AIM path takes the correction into account.
                src_z = src_world.col[2].to_3d()
                dst_z = dst_rest_world.col[2].to_3d()
                if src_z.length > 1e-6 and dst_z.length > 1e-6:
                    src_z.normalize(); dst_z.normalize()
                    if src_z.dot(dst_z) < 0:
                        m.target_axis = '-Y'
                        z_flipped_count += 1
                    else:
                        m.target_axis = '+Y'
        self.report(
            {'INFO'},
            f"Set modes: FULL={counts['FULL']}, AIM={counts['AIM']}, "
            f"AIM_ROLL={counts['AIM_ROLL']}, DELTA={counts['DELTA']}, "
            f"POS={counts['POS']}; target-axis flipped to -Y on {z_flipped_count} "
            f"rows (threshold {threshold:.0f}°)"
        )
        return {'FINISHED'}


class SAM3D_OT_auto_align_ik(Operator):
    bl_idname = "sam3d.auto_align_ik"
    bl_label = "Clear IK rotation offsets"
    bl_description = (
        "Reset the rotation_offset column to zero for every POS-mode row. "
        "POS mode now transfers the source's bone-LOCAL rotation delta onto "
        "target's rest, which handles axis-convention mismatches automatically "
        "— per-row offsets are no longer needed for hand_ik/foot_ik. Use this "
        "to clear leftover values from prior auto-align runs."
    )
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        props = context.scene.sam3d_pose_copy
        count = 0
        with _bulk_mode():
            for m in _prof(props).mappings:
                if not (m.enabled and m.source_bone and m.target_bone):
                    continue
                if m.mode != 'POS':
                    continue
                ro = m.rotation_offset
                if abs(ro[0]) > 1e-6 or abs(ro[1]) > 1e-6 or abs(ro[2]) > 1e-6:
                    m.rotation_offset = (0.0, 0.0, 0.0)
                    count += 1

        self.report({'INFO'}, f"Cleared rotation offset on {count} POS row(s).")
        return {'FINISHED'}


def _delete_armature_and_children(arm_obj):
    """Remove an armature object plus every mesh/etc parented under it."""
    if arm_obj is None:
        return
    for child in list(arm_obj.children_recursive):
        try:
            bpy.data.objects.remove(child, do_unlink=True)
        except Exception:
            pass
    try:
        bpy.data.objects.remove(arm_obj, do_unlink=True)
    except Exception:
        pass


def _import_source_fbx(filepath, target_name=None):
    """Import an FBX and return the new armature object.
    If target_name is provided, the imported armature is renamed to it."""
    try:
        bpy.ops.preferences.addon_enable(module="io_scene_fbx")
    except Exception:
        pass
    before = set(bpy.data.objects)
    bpy.ops.import_scene.fbx(
        filepath=filepath,
        ignore_leaf_bones=False,
        automatic_bone_orientation=False,
    )
    after = set(bpy.data.objects) - before
    armatures = [o for o in after if o.type == 'ARMATURE']
    if not armatures:
        return None
    new_arm = armatures[0]
    if target_name:
        new_arm.name = target_name
        new_arm.data.name = target_name
    return new_arm


class SAM3D_OT_reimport_source(Operator):
    bl_idname = "sam3d.reimport_source"
    bl_label = "Reimport source FBX"
    bl_description = ("Delete the current source armature (and mesh children) and re-import from the "
                      "stored FBX path. Mapping rows and pole targets survive because bone names match.")
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        props = context.scene.sam3d_pose_copy
        path = bpy.path.abspath(_prof(props).source_fbx_path)
        if not path or not os.path.exists(path):
            self.report({'ERROR'}, "Source FBX path is empty or the file doesn't exist.")
            return {'CANCELLED'}

        old_arm = _prof(props).source_armature
        old_name = old_arm.name if old_arm is not None else None
        # Remember the armature's world transform so alignment survives
        old_world = old_arm.matrix_world.copy() if old_arm is not None else None

        _delete_armature_and_children(old_arm)

        new_arm = _import_source_fbx(path, target_name=old_name)
        if new_arm is None:
            self.report({'ERROR'}, "Import produced no armature.")
            return {'CANCELLED'}

        if old_world is not None:
            new_arm.matrix_world = old_world

        # Restore pointer property — Blender should keep it since name matches,
        # but we set it explicitly in case old_arm was already gone.
        _prof(props).source_armature = new_arm
        bpy.context.view_layer.update()
        self.report({'INFO'}, f"Reimported '{os.path.basename(path)}' as '{new_arm.name}'.")
        return {'FINISHED'}


class SAM3D_OT_import_source_fbx(Operator, ImportHelper):
    bl_idname = "sam3d.import_source_fbx"
    bl_label = "Import source FBX..."
    bl_description = "Pick an FBX file, import it, and set it as the source armature (remembers path for Reimport)."
    filename_ext = ".fbx"
    filter_glob: StringProperty(default="*.fbx", options={'HIDDEN'})

    def execute(self, context):
        props = context.scene.sam3d_pose_copy
        new_arm = _import_source_fbx(self.filepath)
        if new_arm is None:
            self.report({'ERROR'}, "Import produced no armature.")
            return {'CANCELLED'}
        _prof(props).source_armature = new_arm
        _prof(props).source_fbx_path = self.filepath
        self.report({'INFO'}, f"Imported '{os.path.basename(self.filepath)}' as source.")
        return {'FINISHED'}


class SAM3D_OT_repose_source_fbx(Operator, ImportHelper):
    bl_idname = "sam3d.repose_source_fbx"
    bl_label = "Repose source (pick FBX)..."
    bl_description = (
        "Pick a new SAM3D FBX with a different pose and swap the current source "
        "armature's bone data for it — no new armature is created in the blend, "
        "so all mapping rows, pole targets, and the armature's world transform "
        "carry over. Bones must have matching MHR names (they will if both FBXs "
        "came from the SAM3D export node)."
    )
    filename_ext = ".fbx"
    filter_glob: StringProperty(default="*.fbx", options={'HIDDEN'})

    def execute(self, context):
        props = context.scene.sam3d_pose_copy
        path = self.filepath
        if not path or not os.path.exists(path):
            self.report({'ERROR'}, "Pick a valid .fbx file.")
            return {'CANCELLED'}

        old_arm = _prof(props).source_armature
        if old_arm is None:
            # Nothing to replace — just do a plain import.
            new_arm = _import_source_fbx(path)
            if new_arm is None:
                self.report({'ERROR'}, "Import produced no armature.")
                return {'CANCELLED'}
            _prof(props).source_armature = new_arm
            _prof(props).source_fbx_path = path
            self.report({'INFO'},
                        f"Imported '{os.path.basename(path)}' as source.")
            return {'FINISHED'}

        old_name = old_arm.name
        old_world = old_arm.matrix_world.copy()

        # Delete the old source armature + its mesh children, then bring in
        # the new FBX under the SAME NAME. Blender's pointer properties on
        # mapping rows / pole targets survive because they're by-name.
        _delete_armature_and_children(old_arm)

        new_arm = _import_source_fbx(path, target_name=old_name)
        if new_arm is None:
            self.report({'ERROR'}, "Import produced no armature.")
            return {'CANCELLED'}

        new_arm.matrix_world = old_world
        _prof(props).source_armature = new_arm
        _prof(props).source_fbx_path = path
        bpy.context.view_layer.update()
        self.report({'INFO'},
                    f"Reposed with '{os.path.basename(path)}' as '{new_arm.name}'.")
        return {'FINISHED'}


class SAM3D_OT_dump_bones(Operator):
    bl_idname = "sam3d.dump_target_bones"
    bl_label = "Print target bones to console"

    def execute(self, context):
        props = context.scene.sam3d_pose_copy
        dst = _prof(props).target_armature
        if dst is None:
            self.report({'ERROR'}, "No target armature set.")
            return {'CANCELLED'}
        names = sorted(pb.name for pb in dst.pose.bones)
        print(f"[SAM3D Pose Copy] {len(names)} bones on '{dst.name}':")
        for n in names:
            print(f"  {n}")
        self.report({'INFO'}, f"Dumped {len(names)} bone names to system console.")
        return {'FINISHED'}


class SAM3D_OT_pick_from_selected(Operator):
    bl_idname = "sam3d.pick_from_selected"
    bl_label = "Fill row from selected pose bones"
    bl_description = "Fill the active row's source from the source armature's active pose bone, and target from the target armature's active pose bone"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        props = context.scene.sam3d_pose_copy
        if not _prof(props).mappings:
            self.report({'WARNING'}, "No mapping row selected; add one first.")
            return {'CANCELLED'}
        idx = _prof(props).active_mapping_index
        if not (0 <= idx < len(_prof(props).mappings)):
            return {'CANCELLED'}
        item = _prof(props).mappings[idx]
        src, dst = _prof(props).source_armature, _prof(props).target_armature
        if src is not None and src.data.bones.active is not None:
            item.source_bone = src.data.bones.active.name
        if dst is not None and dst.data.bones.active is not None:
            item.target_bone = dst.data.bones.active.name
        return {'FINISHED'}


# -----------------------------------------------------------------------------
# Panels
# -----------------------------------------------------------------------------

class SAM3D_OT_transfer_face_shape_keys(Operator):
    """Copy all MHR face-expression shape keys from the profile's source mesh
    onto the target mesh via a Surface Deform bake.

    Works across different topologies (Rigify head ≠ SAM3D head), so the user
    can sculpt each blendshape (jaw, brow, lip corners, cheeks, etc.) directly
    on their target character. Existing shape keys on the target with the same
    name are removed first so re-running gives a clean result."""

    bl_idname = "sam3d.transfer_face_shape_keys"
    bl_label = "Transfer face shape keys → target mesh"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        props = context.scene.sam3d_pose_copy
        prof = _prof(props)
        if prof is None:
            self.report({'ERROR'}, "No active profile.")
            return {'CANCELLED'}

        src = prof.face_shape_key_source
        tgt = prof.face_shape_key_target
        if src is None or tgt is None:
            self.report({'ERROR'}, "Pick both Expr source and Expr target meshes on this profile.")
            return {'CANCELLED'}
        if src.type != 'MESH' or tgt.type != 'MESH':
            self.report({'ERROR'}, "Expr source and target must both be meshes.")
            return {'CANCELLED'}
        if src.data.shape_keys is None or len(src.data.shape_keys.key_blocks) < 2:
            self.report({'ERROR'}, "Source mesh has no expression shape keys — export the "
                                    "FBX with 'bake_face_shape_keys' enabled.")
            return {'CANCELLED'}

        src_keys = src.data.shape_keys
        # Names to transfer: everything except Basis. Includes the "expr_XX"
        # keys the exporter writes; also any custom-named keys the user added.
        source_key_names = [kb.name for kb in src_keys.key_blocks if kb.name != "Basis"]
        if not source_key_names:
            self.report({'ERROR'}, "Source mesh only has a Basis key — nothing to transfer.")
            return {'CANCELLED'}

        # Snapshot source shape-key values so we can restore after transfer.
        original_values = {kb.name: float(kb.value) for kb in src_keys.key_blocks}

        # Ensure target has a Basis shape key. from_mix=False so it's the
        # true rest mesh.
        if tgt.data.shape_keys is None:
            tgt.shape_key_add(name="Basis", from_mix=False)

        # Add Surface Deform modifier + bind. Bind fails silently on some
        # meshes (n-gons, non-manifold), so verify the bind succeeded.
        mod_name = "_SAM3D_ShapeKeyTransfer"
        # Remove any leftover from a previous attempt.
        for m in list(tgt.modifiers):
            if m.name == mod_name:
                tgt.modifiers.remove(m)

        sd = tgt.modifiers.new(name=mod_name, type='SURFACE_DEFORM')
        sd.target = src

        # Bind requires the modifier's owner to be the active object.
        prev_active = context.view_layer.objects.active
        prev_selected = [o for o in context.selected_objects]
        try:
            for o in prev_selected:
                o.select_set(False)
            tgt.select_set(True)
            context.view_layer.objects.active = tgt

            # Zero all source shape keys BEFORE binding so the bind captures
            # the rest (Basis) geometry — otherwise the "unactivated" state
            # already carries the source's baked expression.
            for kb in src_keys.key_blocks:
                kb.value = 0.0
            context.view_layer.update()

            bpy.ops.object.surfacedeform_bind(modifier=mod_name)
            if not sd.is_bound:
                self.report({'ERROR'}, "Surface Deform bind failed — try increasing 'Interpolation "
                                        "Falloff' or check the target mesh for non-manifold geometry.")
                return {'CANCELLED'}

            depsgraph = context.evaluated_depsgraph_get()

            transferred = 0
            for name in source_key_names:
                # Activate this expression on the source ONLY.
                for kb in src_keys.key_blocks:
                    kb.value = 1.0 if kb.name == name else 0.0
                context.view_layer.update()
                depsgraph.update()

                # Evaluate the target mesh with the Surface Deform modifier
                # applied — this gives us where each vertex ends up under
                # the current source deformation.
                eval_obj = tgt.evaluated_get(depsgraph)
                eval_mesh = eval_obj.to_mesh()

                # Remove any existing shape key with the same name so re-runs
                # don't accumulate duplicates.
                if name in tgt.data.shape_keys.key_blocks:
                    old = tgt.data.shape_keys.key_blocks[name]
                    tgt.shape_key_remove(old)

                new_key = tgt.shape_key_add(name=name, from_mix=False)
                new_key.slider_min = -2.0
                new_key.slider_max = 2.0
                # Copy evaluated vertex positions into the new key.
                # to_mesh() returns a mesh whose vertex count == original,
                # since Surface Deform doesn't add/remove verts.
                for vi, v in enumerate(eval_mesh.vertices):
                    new_key.data[vi].co = v.co

                eval_obj.to_mesh_clear()
                transferred += 1

        finally:
            # Restore source shape-key values (default to original from
            # snapshot, else 0).
            for kb in src_keys.key_blocks:
                kb.value = original_values.get(kb.name, 0.0)
            # Remove the transfer modifier — it was scaffolding only.
            if mod_name in tgt.modifiers:
                tgt.modifiers.remove(tgt.modifiers[mod_name])

            # Restore selection and active object.
            for o in context.selected_objects:
                o.select_set(False)
            for o in prev_selected:
                try:
                    o.select_set(True)
                except ReferenceError:
                    pass
            if prev_active is not None:
                try:
                    context.view_layer.objects.active = prev_active
                except ReferenceError:
                    pass

        self.report({'INFO'}, f"Transferred {transferred} shape keys from '{src.name}' to '{tgt.name}'.")
        return {'FINISHED'}


class SAM3D_PT_pose_panel(Panel):
    bl_label = "SAM3D Pose Copy"
    bl_idname = "SAM3D_PT_pose_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'SAM3D'

    def draw(self, context):
        layout = self.layout
        props = context.scene.sam3d_pose_copy

        # Profile tabs — one per character. Active one is highlighted; each
        # profile has its own source/target/mappings/poles/etc.
        active = _prof(props)
        prof_row = layout.row(align=True)
        for i, p in enumerate(props.profiles):
            sub = prof_row.row(align=True)
            sub.alert = (i == props.active_profile_index)
            op = sub.operator("sam3d.switch_profile",
                              text=(p.name or f"Ch {i+1}"),
                              depress=(i == props.active_profile_index))
            op.index = i
        prof_row.operator("sam3d.add_profile", icon='ADD', text="")
        prof_row.operator("sam3d.remove_profile", icon='REMOVE', text="")
        prof_row.operator("sam3d.rename_profile", icon='GREASEPENCIL', text="")
        prof_row.separator()
        prof_row.operator("sam3d.copy_mappings", icon='COPYDOWN', text="Copy")
        prof_row.operator("sam3d.paste_mappings", icon='PASTEDOWN', text="Paste")

        if active is None:
            # No profile yet — draw context couldn't auto-create one. Ask
            # user to click Add. All subsequent draw sections would crash on
            # a None active, so bail early.
            layout.label(text="No profiles yet — click ➕ to add one.",
                          icon='ERROR')
            return

        # Armature pickers
        col = layout.column(align=True)
        col.prop(active, "source_armature")
        col.prop(active, "target_armature")

        # Source FBX path + reimport
        box = layout.box()
        box.label(text="Source FBX (for Reimport)", icon='FILE_3D')
        box.prop(active, "source_fbx_path", text="")
        row = box.row(align=True)
        row.operator("sam3d.import_source_fbx", icon='IMPORT')
        row.operator("sam3d.reimport_source", icon='FILE_REFRESH')
        row.operator("sam3d.repose_source_fbx", icon='POSE_HLT', text="Repose (pick)")

        # Mapping list
        row = layout.row()
        row.template_list(
            "SAM3D_UL_mappings", "",
            active, "mappings",
            active, "active_mapping_index",
            rows=8,
        )
        col = row.column(align=True)
        col.operator("sam3d.add_mapping", icon='ADD', text="")
        col.operator("sam3d.remove_mapping", icon='REMOVE', text="")
        col.separator()
        col.operator("sam3d.pick_from_selected", icon='EYEDROPPER', text="")
        col.separator()
        col.operator("sam3d.clear_mappings", icon='TRASH', text="")

        # Per-row rotation offset editor (for the active row). Axis dropdowns
        # live in the row itself now.
        idx = _prof(props).active_mapping_index
        if 0 <= idx < len(_prof(props).mappings):
            active_row = _prof(props).mappings[idx]
            box = layout.box()
            box.scale_y = 0.9
            box.label(text=f"Rotation offset: {active_row.source_bone} → {active_row.target_bone}",
                      icon='DRIVER_ROTATIONAL_DIFFERENCE')
            box.prop(active_row, "rotation_offset", text="")

        # Workflow steps in the order you'd normally run them.
        box = layout.box()
        box.label(text="Workflow (top to bottom):", icon='INFO')

        # 1. Populate mapping
        row = box.row(align=True)
        row.operator("sam3d.auto_match", icon='ZOOM_SELECTED', text="1. Auto-match")
        row.operator("sam3d.add_all_source_bones", icon='OUTLINER_OB_ARMATURE', text="Fill from source")

        # 2. Align source
        row = box.row(align=True)
        row.operator("sam3d.align_source_facing", icon='ORIENTATION_GIMBAL', text="2. Align facing")
        row.operator("sam3d.flip_source_180", icon='FILE_REFRESH', text="Flip 180°")
        row = box.row(align=True)
        row.operator("sam3d.apply_master_yaw", icon='EMPTY_AXIS')

        # 3. Scale source (auto) + manual nudge buttons for fine-tuning
        row = box.row(align=True)
        row.operator("sam3d.scale_source_to_target", icon='FULLSCREEN_ENTER', text="3. Scale to target")
        op = row.operator("sam3d.nudge_scale", text="+5%")
        op.factor = 1.05
        op = row.operator("sam3d.nudge_scale", text="-5%")
        op.factor = 1.0 / 1.05

        # 4. Set per-row modes based on rest-pose analysis
        row = box.row(align=True)
        row.operator("sam3d.auto_set_modes", icon='SORTBYEXT', text="4. Auto-set modes")
        row.prop(props, "auto_mode_threshold", text="threshold")

        # 4b. Zero out rotation_offset on POS rows (they use bone-local delta now)
        row = box.row(align=True)
        row.operator("sam3d.auto_align_ik", icon='X', text="4b. Clear IK rotation offsets")

        # 5. Apply the copy
        row = box.row(align=True)
        row.operator("sam3d.copy_all", icon='POSE_HLT', text="5. Copy all enabled")

        # Presets + preset save/load
        row = layout.row(align=True)
        row.operator("sam3d.load_rigify_standard_preset", icon='ARMATURE_DATA',
                     text="Rigify (standard)")
        row.operator("sam3d.load_rigify_edit_preset", icon='ARMATURE_DATA',
                     text="Rigify (edit)")
        row = layout.row(align=True)
        row.operator("sam3d.load_sintel_preset", icon='OUTLINER_OB_ARMATURE',
                     text="Sintel (CloudRig)")
        row = layout.row(align=True)
        row.operator("sam3d.save_preset", icon='FILE_TICK')
        row.operator("sam3d.load_preset", icon='FILEBROWSER')

        # Face expression shape-key transfer
        box = layout.box()
        box.label(text="Face expression shape keys", icon='SHAPEKEY_DATA')
        col = box.column(align=True)
        col.prop(_prof(props), "face_shape_key_source")
        col.prop(_prof(props), "face_shape_key_target")
        row = box.row(align=True)
        row.operator("sam3d.transfer_face_shape_keys",
                     icon='SHAPEKEY_DATA',
                     text="Transfer to target mesh")

        # IK pole targets (populated by auto-match or manually)
        box = layout.box()
        box.label(text="IK pole targets", icon='EMPTY_SINGLE_ARROW')
        col = box.column(align=True)
        target_arm = _prof(props).target_armature
        _p = _prof(props)
        if target_arm is not None:
            col.prop_search(_p, "l_arm_pole", target_arm.pose, "bones", text="L elbow", icon='BONE_DATA')
            col.prop_search(_p, "r_arm_pole", target_arm.pose, "bones", text="R elbow", icon='BONE_DATA')
            col.prop_search(_p, "l_leg_pole", target_arm.pose, "bones", text="L knee",  icon='BONE_DATA')
            col.prop_search(_p, "r_leg_pole", target_arm.pose, "bones", text="R knee",  icon='BONE_DATA')
        else:
            col.prop(_p, "l_arm_pole", text="L elbow")
            col.prop(_p, "r_arm_pole", text="R elbow")
            col.prop(_p, "l_leg_pole", text="L knee")
            col.prop(_p, "r_leg_pole", text="R knee")
        box.prop(_p, "pole_distance")

        # Copy options
        col = layout.column(align=True)
        col.prop(props, "reset_before_copy")
        col.prop(props, "position_only")
        col.prop(props, "live_update")
        col.prop(_prof(props), "spine_bend_amplify")

        # Diagnostics
        row = layout.row(align=True)
        row.operator("sam3d.diagnose", icon='OUTLINER_OB_LIGHT')
        row.operator("sam3d.dump_target_bones", icon='CONSOLE')

        box = layout.box()
        box.scale_y = 0.85
        box.label(text="Tips:", icon='INFO')
        box.label(text="• Use the ▶ button per row to test one mapping.")
        box.label(text="• Red bone icon = name not found on that armature.")
        box.label(text="• For arm/leg IK, switch the rig to IK before copying.")
        box.label(text="• To view side-by-side: move the target ARMATURE OBJECT")
        box.label(text="  (Object mode G), not the master pose bone.")
        box.label(text="• Row set to Skip = not touched by copy. Use to preserve")
        box.label(text="  a bone you've manually adjusted.")


# -----------------------------------------------------------------------------
# Registration
# -----------------------------------------------------------------------------

classes = (
    SAM3DMappingItem,
    SAM3DProfile,
    SAM3DPoseCopyProps,
    SAM3D_UL_mappings,
    SAM3D_OT_add_profile,
    SAM3D_OT_remove_profile,
    SAM3D_OT_rename_profile,
    SAM3D_OT_switch_profile,
    SAM3D_OT_copy_mappings,
    SAM3D_OT_paste_mappings,
    SAM3D_OT_add_mapping,
    SAM3D_OT_remove_mapping,
    SAM3D_OT_clear_mappings,
    SAM3D_OT_add_all_source_bones,
    SAM3D_OT_auto_match,
    SAM3D_OT_align_source_facing,
    SAM3D_OT_flip_source_180,
    SAM3D_OT_apply_master_yaw,
    SAM3D_OT_scale_source_to_target,
    SAM3D_OT_nudge_scale,
    SAM3D_OT_diagnose,
    SAM3D_OT_auto_set_modes,
    SAM3D_OT_auto_align_ik,
    SAM3D_OT_reimport_source,
    SAM3D_OT_import_source_fbx,
    SAM3D_OT_repose_source_fbx,
    SAM3D_OT_load_rigify_edit_preset,
    SAM3D_OT_load_rigify_standard_preset,
    SAM3D_OT_load_sintel_preset,
    SAM3D_OT_save_preset,
    SAM3D_OT_load_preset,
    SAM3D_OT_copy_row,
    SAM3D_OT_copy_all,
    SAM3D_OT_dump_bones,
    SAM3D_OT_pick_from_selected,
    SAM3D_OT_transfer_face_shape_keys,
    SAM3D_PT_pose_panel,
)


def register():
    for c in classes:
        bpy.utils.register_class(c)
    bpy.types.Scene.sam3d_pose_copy = PointerProperty(type=SAM3DPoseCopyProps)
    # Add default profile to whatever scenes already exist (the currently-
    # open one on addon enable / Blender startup — load_post doesn't fire
    # for these).
    _ensure_default_profile_for_all_scenes()
    # Also register for load / New-File events so profile appears there too.
    if _ensure_default_profile_on_load not in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.append(_ensure_default_profile_on_load)
    if hasattr(bpy.app.handlers, "load_factory_startup_post"):
        if _ensure_default_profile_on_load not in bpy.app.handlers.load_factory_startup_post:
            bpy.app.handlers.load_factory_startup_post.append(_ensure_default_profile_on_load)


def unregister():
    if _ensure_default_profile_on_load in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.remove(_ensure_default_profile_on_load)
    if hasattr(bpy.app.handlers, "load_factory_startup_post"):
        if _ensure_default_profile_on_load in bpy.app.handlers.load_factory_startup_post:
            bpy.app.handlers.load_factory_startup_post.remove(_ensure_default_profile_on_load)
    del bpy.types.Scene.sam3d_pose_copy
    for c in reversed(classes):
        bpy.utils.unregister_class(c)


if __name__ == "__main__":
    register()
