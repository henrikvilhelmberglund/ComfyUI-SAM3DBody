# Semantic names for the 72 MHR v1.x face-expression blendshapes.
#
# Source: https://github.com/facebookresearch/MHR/blob/main/docs/face-expressions.md
# The shipped TorchScript model exposes only the coefficient count, not names,
# but the ordering is stable across LODs 0-6 and identical to `mhr.FACE_EXPRESSION_NAMES`
# in the upstream package. Index i in this list is the column used by
# `face_expr_coeffs[:, i]`, and also maps to `shape_{45+i}` in FBX exports.
#
# Suffix convention: `_L` / `_R` = character's left / right; lip-quadrant
# controls additionally use `T` / `B` for top / bottom.

MHR_FACE_EXPRESSION_NAMES = [
    "browLowerer_L",           # 0
    "browLowerer_R",           # 1
    "cheekPuff_L",             # 2
    "cheekPuff_R",             # 3
    "cheekRaiser_L",           # 4
    "cheekRaiser_R",           # 5
    "cheekSuck_L",             # 6
    "cheekSuck_R",             # 7
    "chinRaiser_B",            # 8
    "chinRaiser_T",            # 9
    "dimpler_L",               # 10
    "dimpler_R",               # 11
    "eyesClosed_L",            # 12
    "eyesClosed_R",            # 13
    "eyesLookDown_L",          # 14
    "eyesLookDown_R",          # 15
    "eyesLookLeft_L",          # 16
    "eyesLookLeft_R",          # 17
    "eyesLookRight_L",         # 18
    "eyesLookRight_R",         # 19
    "eyesLookUp_L",            # 20
    "eyesLookUp_R",            # 21
    "innerBrowRaiser_L",       # 22
    "innerBrowRaiser_R",       # 23
    "jawDrop",                 # 24
    "jawSidewaysLeft",         # 25
    "jawSidewaysRight",        # 26
    "jawThrust",               # 27
    "lidTightener_L",          # 28
    "lidTightener_R",          # 29
    "lipCornerDepressor_L",    # 30
    "lipCornerDepressor_R",    # 31
    "lipCornerPuller_L",       # 32
    "lipCornerPuller_R",       # 33
    "lipFunneler_LB",          # 34
    "lipFunneler_LT",          # 35
    "lipFunneler_RB",          # 36
    "lipFunneler_RT",          # 37
    "lipPressor_L",            # 38
    "lipPressor_R",            # 39
    "lipPucker_L",             # 40
    "lipPucker_R",             # 41
    "lipStretcher_L",          # 42
    "lipStretcher_R",          # 43
    "lipSuck_LB",              # 44
    "lipSuck_LT",              # 45
    "lipSuck_RB",              # 46
    "lipSuck_RT",              # 47
    "lipTightener_L",          # 48
    "lipTightener_R",          # 49
    "lipsToward_LB",           # 50
    "lipsToward_LT",           # 51
    "lipsToward_RB",           # 52
    "lipsToward_RT",           # 53
    "lowerLipDepressor_L",     # 54
    "lowerLipDepressor_R",     # 55
    "mouthLeft",               # 56
    "mouthRight",              # 57
    "nasolabialFurrow_L",      # 58
    "nasolabialFurrow_R",      # 59
    "noseWrinkler_L",          # 60
    "noseWrinkler_R",          # 61
    "nostrilCompressor_L",     # 62
    "nostrilCompressor_R",     # 63
    "nostrilDilator_L",        # 64
    "nostrilDilator_R",        # 65
    "outerBrowRaiser_L",       # 66
    "outerBrowRaiser_R",       # 67
    "upperLidRaiser_L",        # 68
    "upperLidRaiser_R",        # 69
    "upperLipRaiser_L",        # 70
    "upperLipRaiser_R",        # 71
]

assert len(MHR_FACE_EXPRESSION_NAMES) == 72
