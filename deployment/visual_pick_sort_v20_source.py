"""Qualify an automatic visual support decision before any arm motion."""
import copy
import hashlib
import math
from visual_batch_workspace_v20 import polygon,contains,foot

from visual_pick_sort_v14_source import (ROOT,SUCCESS_SHA,accepted_success,accepted_anchor,
    make_anchor,bin_rows)
from visual_pick_sort_v16_source import target_candidate as measured_level_target
from visual_pick_sort_v18_auto_height import infer_support

MANIFEST=ROOT/'deployment/visual-pick-sort-v20-files.sha256'


def verify_delivery():
    inventory={}
    for line in MANIFEST.read_text(encoding='ascii').splitlines():
        digest,name=line.split('  ',1);path=(ROOT/name).resolve()
        if ROOT not in path.parents or name in inventory or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
            raise ValueError('Delivery file differs: '+name)
        inventory[name]=digest
    if not inventory:raise ValueError('Empty delivery manifest')
    return inventory


def target_candidate(point,anchor):
    accepted_anchor(anchor)
    if point.get('automatic_target_selection') is True:
        points=polygon(point['automatic_workspace'],anchor['camera_context'])
        box=point['measurement_bbox_px']
        if (not isinstance(box,list) or len(box)!=4
                or any(type(v) not in (int,float) or not math.isfinite(v) for v in box)
                or min(box[2:])<=0 or not contains(foot(box),points)):
            raise ValueError('Automatic selected cylinder base is outside the defined image workspace')
    decision=infer_support(point['auto_support_height_evidence'],anchor['equal_height_plane'],
        point['burst_summary']['elapsed_seconds'])
    if (decision.get('qualified') is not True or not same_measurement(decision,point.get('auto_support_decision'))
            or point.get('support_surface')!=decision['selected_surface']
            or point.get('operator_height_selection') is not False):
        raise ValueError('Automatic current visual height decision is absent, ambiguous or altered')
    result=measured_level_target(point,anchor)
    result.update(scope='live_visual_auto_support_translation_from_actual044001_pickup',
        automatic_support_selection=True,operator_height_selection=False,
        observed_height_median_mm=decision['observed_rise_median_mm'],
        observed_height_repeatability_mm=decision['height_repeatability_max_mm'],
        arbitrary_unknown_height_supported=False)
    return result


def same_measurement(current,saved):
    # A repeated numerical solver may differ in floating point roundoff.
    # This comparison does not change a geometric, motion or support threshold.
    if type(current) is not type(saved):return False
    if isinstance(current,dict):
        return current.keys()==saved.keys() and all(same_measurement(v,saved[k]) for k,v in current.items())
    if isinstance(current,list):
        return len(current)==len(saved) and all(same_measurement(a,b) for a,b in zip(current,saved))
    if isinstance(current,float):
        return math.isfinite(current) and math.isfinite(saved) and math.isclose(current,saved,rel_tol=1e-12,abs_tol=1e-9)
    return current==saved


def accepted_request(request):
    success=accepted_success();mode=request.get('mode')
    if request.get('success_source_sha256')!=SUCCESS_SHA:
        raise ValueError('Current request must use actual044001 successful pickup')
    if mode not in ('deliver_held','park_empty','visual_cycle'):
        raise ValueError('Unsupported current cycle mode')
    required=['exclusive','onsite_stop','inert_shell','fixed_head_board_mount','whole_route_and_sweep_clear']
    if mode=='deliver_held':
        required+=['same_current_held_shell','no_release_move_or_restart_since_success']
    else:
        required+=['fresh_actual_open_posture_required','initial_stationary_group_open_authorized',
            'initial_opening_and_inert_shell_drop_zone_clear']
    if mode=='visual_cycle':required+=['target_unchanged_since_capture','continuous_pick_sort_authorized']
    permission=request.get('scene_permission',{})
    if request.get('return_to_scan_upper_after_drop') is True:
        if mode!='visual_cycle' or permission.get('batch_route_and_scan_return_authorized') is not True:
            raise ValueError('Current batch return route permission missing')
    for key in required:
        if permission.get(key) is not True:raise ValueError('Current permission missing: '+key)
    candidate=None
    if mode=='visual_cycle':
        candidate=target_candidate(request['point_record'],request['anchor'])
        if request.get('candidate')!=candidate:raise ValueError('Automatic visual target candidate was altered')
    return success,candidate
