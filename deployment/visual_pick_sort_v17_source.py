"""Qualify an automatic visual support decision before any arm motion."""
import copy
import hashlib

from visual_pick_sort_v14_source import (ROOT,SUCCESS_SHA,accepted_success,accepted_anchor,
    make_anchor,bin_rows,accepted_request as base_request)
from visual_pick_sort_v16_source import target_candidate as measured_level_target
from visual_pick_sort_v17_auto_height import infer_support

MANIFEST=ROOT/'deployment/visual-pick-sort-v17-files.sha256'


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
    decision=infer_support(point['auto_support_height_evidence'],anchor['equal_height_plane'],
        point['burst_summary']['elapsed_seconds'])
    if (decision.get('qualified') is not True or decision!=point.get('auto_support_decision')
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


def accepted_request(request):
    if request.get('mode')!='visual_cycle':return base_request(request)
    copied=copy.deepcopy(request);copied['mode']='park_empty';success,_=base_request(copied)
    for key in ('target_unchanged_since_capture','continuous_pick_sort_authorized'):
        if request.get('scene_permission',{}).get(key) is not True:
            raise ValueError('Current permission missing: '+key)
    candidate=target_candidate(request['point_record'],request['anchor'])
    if request.get('candidate')!=candidate:raise ValueError('Automatic visual target candidate was altered')
    return success,candidate
