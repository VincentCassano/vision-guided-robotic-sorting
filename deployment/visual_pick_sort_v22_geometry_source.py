"""Verify v22 support evidence; retain original handling for saved bindings."""
import math

from visual_pick_sort_v20_source import (target_candidate as legacy_target,
    same_measurement)
from visual_pick_sort_v14_source import accepted_anchor
from visual_pick_sort_v16_source import target_candidate as measured_level_target
from visual_batch_workspace_v20 import polygon,contains,foot
from visual_pick_sort_v22_auto_height import infer_support,METHOD
from visual_pick_sort_v22_fix3_auto_height import infer_support as legacy_infer_support, METHOD as LEGACY_METHOD


def target_candidate(point,anchor):
    point_method=point.get('auto_support_decision',{}).get('method')
    if point_method not in (METHOD,LEGACY_METHOD):
        # Old near/far visual bindings are historical evidence. Never relabel
        # their saved result as if the new estimator had run on those images.
        return legacy_target(point,anchor)
    accepted_anchor(anchor)
    if point.get('automatic_target_selection') is True:
        points=polygon(point['automatic_workspace'],anchor['camera_context'])
        box=point['measurement_bbox_px']
        if (not isinstance(box,list) or len(box)!=4
                or any(type(v) not in (int,float) or not math.isfinite(v) for v in box)
                or min(box[2:])<=0 or not contains(foot(box),points)):
            raise ValueError('Automatic target base is outside current image workspace')
    solver=legacy_infer_support if point_method==LEGACY_METHOD else infer_support
    decision=solver(point['auto_support_height_evidence'],anchor['equal_height_plane'],
        point['burst_summary']['elapsed_seconds'])
    if (decision.get('qualified') is not True
            or not same_measurement(decision,point.get('auto_support_decision'))
            or point.get('support_surface')!=decision['selected_surface']
            or point.get('operator_height_selection') is not False):
        raise ValueError('Current all-frame support hypotheses are ambiguous, unavailable or altered')
    result=measured_level_target(point,anchor)
    result.update(scope='live_visual_two_pixel_support_hypotheses_from_actual044001_pickup_v22',
        automatic_support_selection=True,operator_height_selection=False,
        observed_height_median_mm=decision['observed_rise_median_mm'],
        observed_height_repeatability_mm=decision['height_repeatability_max_mm'],
        continuous_height_spread_is_diagnostic=True,
        support_constrained_center_repeatability_max_mm=decision['support_constrained_center_repeatability_max_mm'],
        support_method=point_method,original_height_result_retained=True,
        arbitrary_unknown_height_supported=False)
    return result
