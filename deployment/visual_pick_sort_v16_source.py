"""Keep the accepted wrist binding and qualify the selected support plane."""
import copy
import hashlib

from visual_pick_sort_v14_source import (ROOT,SUCCESS_SHA,accepted_success,accepted_anchor,
    make_anchor,point_checked,bin_rows,accepted_request as base_request)
from visual_pick_sort_v16_plane import support_plane

MANIFEST=ROOT/'deployment/visual-pick-sort-v16-files.sha256'


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
    accepted_anchor(anchor);top=point_checked(point)
    surface=point.get('support_surface')
    expected_plane,rise,digest=support_plane(anchor['equal_height_plane'],surface)
    if (point['binding_camera_context']!=anchor['camera_context']
            or point['equal_height_plane']!=expected_plane
            or point.get('measured_support_rise_mm')!=rise
            or point.get('support_height_model_sha256')!=digest):
        raise ValueError('Target support height/camera/plane differs from its retained image evidence')
    old=anchor['reference_pregrasp_tcp_pose_m_rad'];origin=anchor['reference_top_center_world_m']
    delta=[top[i]-origin[i] for i in range(3)]
    pose=[old[i]+delta[i] for i in range(3)]+old[3:];upper=pose[:];upper[2]+=.180
    return dict(scope='live_visual_translation_from_actual044001_pickup_measured_support',
        pregrasp_tcp_candidate_m_rad=pose,hover_tcp_candidate_m_rad=upper,
        delta_from_reference_world_mm=[v*1000 for v in delta],fixed_wrist_rpy_rad=old[3:],
        prepare_recipe=anchor['prepare_targets'][:],grip_recipe=anchor['grip_targets'][:],lift_mm=180,
        support_surface=surface,measured_support_rise_mm=rise,support_height_model_sha256=digest,
        every_live_ray_reintersected=True,z_only_TCP_offset_used=False,
        slot_lookup_used=False,grid_snap_used=False,old_offsets_applied=False,
        actual_finger_arrival_required=False,motion_authorized=False,execution_ready=False)


def accepted_request(request):
    if request.get('mode')!='visual_cycle':return base_request(request)
    # Base qualification retains source identity and the actual empty-route
    # permission. This branch additionally binds its fresh image/height below.
    copied=copy.deepcopy(request);copied['mode']='park_empty'
    success,_=base_request(copied)
    for key in ('target_unchanged_since_capture','continuous_pick_sort_authorized'):
        if request.get('scene_permission',{}).get(key) is not True:
            raise ValueError('Current permission missing: '+key)
    candidate=target_candidate(request['point_record'],request['anchor'])
    if request.get('candidate')!=candidate:raise ValueError('Current measured-support image candidate was altered')
    return success,candidate
