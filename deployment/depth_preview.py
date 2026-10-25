"""Diagnostic surface points in the color optical frame, never grasp poses."""
import numpy as np


def measure(depth, scale, u, v, intrinsics, deproject):
    result = dict(depth_valid=False, position_optical_m=None, execution_ready=False)
    h, w = depth.shape
    if not np.isfinite([scale, u, v]).all() or scale <= 0 or not (0 <= u < w and 0 <= v < h):
        return dict(result, depth_reason='Invalid pixel or depth scale')
    x, y = int(u), int(v)
    patch = depth[max(0,y-3):min(h,y+4), max(0,x-3):min(w,x+4)].astype(float)*scale
    valid = patch[np.isfinite(patch) & (patch >= .15) & (patch <= 3.0)]
    result['depth_valid_fraction'] = float(valid.size/patch.size)
    if valid.size < 5 or valid.size/patch.size < .5:
        return dict(result, depth_reason='Insufficient valid depth')
    z = float(np.median(valid))
    spread = float(np.percentile(valid,90)-np.percentile(valid,10))
    result.update(depth_m=z, depth_spread_m=spread)
    if spread > .04:
        return dict(result, depth_reason='Depth edge or unstable surface')
    point = [float(p) for p in deproject(intrinsics, [float(u),float(v)], z)]
    if not np.isfinite(point).all():
        return dict(result, depth_reason='Invalid deprojection')
    return dict(result, depth_valid=True, position_optical_m=point, depth_reason='')
