import logging

log = logging.getLogger("sam3dbody")


def sam3d_attention(q, k, v, heads, mask=None, skip_reshape=False):
    """Dispatch attention using ComfyUI's device-appropriate backend."""
    from comfy.ldm.modules.attention import optimized_attention_for_device
    fn = optimized_attention_for_device(q.device, mask=mask is not None)
    # SAM 3D Body regresses continuous 3D vertex/rotation coordinates, so
    # INT8-quantised attention (SageAttention) is not an acceptable
    # approximation here: measured ~1.3e-2 rel-L2 vs ~3.5e-3 for SDPA/flash,
    # which propagates to ~7 deg of joint-angle error through the 6D->SO(3)
    # head. ComfyUI core sets this same flag for every geometry model
    # (ldm/sam3/sam.py, ldm/triposplat/model.py, ldm/hunyuan3dv2_1/...).
    # Every backend takes **kwargs, so this is safe on all of them; only
    # attention_sage reads it. Belt-and-braces: it holds even if
    # sageattention arrives in the env by some other route.
    return fn(q, k, v, heads=heads, mask=mask, skip_reshape=skip_reshape,
              low_precision_attention=False)
