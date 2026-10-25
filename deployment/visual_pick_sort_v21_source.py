"""Two actual pickup wrists; image regions choose wrist, never target XYZ."""
import copy
import hashlib
import json
import math

from visual_pick_sort_v14_source import (ROOT,SUCCESS_SHA,accepted_success as far_success,
    accepted_anchor,bin_rows)
from visual_pick_sort_v20_source import target_candidate as geometry_candidate
from visual_pick_sort_v20_source import same_measurement
from visual_batch_workspace_v20 import polygon,foot

MANIFEST=ROOT/'deployment/visual-pick-sort-v21-files.sha256'
NEAR_FILE=ROOT/'deployment/grasp_near_success_140429/success_record.json'
NEAR_SHA='4d4ea59a9f188651029c2f87d11928234452429d5debd7ed7560f7f8b12b55ce'
NEAR_CELLS=[[0,0],[0,1],[0,2],[1,1],[1,2]]


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_delivery():
    checked={}
    for line in MANIFEST.read_text(encoding='ascii').splitlines():
        digest,name=line.split('  ',1);path=(ROOT/name).resolve()
        if ROOT not in path.parents or name in checked or sha(path)!=digest:
            raise ValueError('Delivery file differs: '+name)
        checked[name]=digest
    if not checked:raise ValueError('Empty delivery manifest')
    return checked


def near_success():
    from grasp_rebuild_records_v1 import assemble_success
    if sha(NEAR_FILE)!=NEAR_SHA:raise ValueError('Actual140429 near success record differs')
    value=json.loads(NEAR_FILE.read_bytes())
    if value.get('scope')!='continued_original_near_wrist_actual180mm_pickup_record':
        raise ValueError('Near pickup source is not the actual accepted continuation')
    expected=assemble_success(value['start_record'],value['grip_shape_record'],value['prepare_shape_record'],
        value['wrist_record'],value['lift_result'],value.get('operator_observed_pickup_success'))
    for key,item in expected.items():
        if key!='scope' and value.get(key)!=item:raise ValueError('Near pickup provenance differs: '+key)
    return value


def success_for(profile):
    if profile=='near':return near_success(),NEAR_SHA
    if profile=='far':return far_success(),SUCCESS_SHA
    raise ValueError('Unknown wrist profile')


def make_near_binding(point,anchor,original_place_confirmed):
    if original_place_confirmed is not True:raise ValueError('Near shell must be at its actual successful original place')
    geometry_candidate(point,anchor)
    near=near_success();pose=near['reference_pregrasp_tcp_pose_m_rad']
    return dict(schema_version=21,scope='actual140429_near_wrist_bound_to_current_live_geometry',
        near_success_sha256=NEAR_SHA,far_geometry_anchor=copy.deepcopy(anchor),
        reference_point=copy.deepcopy(point),reference_top_center_world_m=point['top_center_world_m'][:],
        reference_pregrasp_tcp_pose_m_rad=pose[:],original_place_confirmed=True,
        camera_context=copy.deepcopy(point['binding_camera_context']),
        original_far_grasp_mapping_used_for_near=False,slot_XYZ_lookup_used=False,
        motion_authorized=False,execution_ready=False)


def accepted_near_binding(value,anchor):
    if not isinstance(value,dict):raise ValueError('Near visual binding is missing; use b once')
    expected=make_near_binding(value.get('reference_point'),anchor,value.get('original_place_confirmed'))
    if value!=expected:raise ValueError('Near binding differs from actual pickup and retained image')
    return expected


def grid_matrix(record,context):
    import numpy as np
    points=polygon(record,context)
    if (record.get('role')!='wrist_selection_only' or record.get('near_cells')!=NEAR_CELLS
            or record.get('corner_order')!=['near_left','near_right','far_right','far_left']):
        raise ValueError('Required near-row3 and middle-right2 wrist policy differs')
    a=[];b=[]
    for (x,y),(u,v) in zip(points,[(0.,0.),(1.,0.),(1.,1.),(0.,1.)]):
        a.extend([[x,y,1.,0.,0.,0.,-u*x,-u*y],[0.,0.,0.,x,y,1.,-v*x,-v*y]])
        b.extend([u,v])
    h=np.linalg.solve(np.asarray(a,float),np.asarray(b,float))
    return np.asarray(list(h)+[1.],float).reshape(3,3)


def select_profile(point,grid):
    h=grid_matrix(grid,point['binding_camera_context'])
    x,y=foot(point['measurement_bbox_px'])
    denominator=float(h[2,0]*x+h[2,1]*y+h[2,2])
    if abs(denominator)<1e-12:raise ValueError('Image region projection is singular')
    u=float((h[0,0]*x+h[0,1]*y+h[0,2])/denominator)
    v=float((h[1,0]*x+h[1,1]*y+h[1,2])/denominator)
    if not all(math.isfinite(q) for q in (u,v)):raise ValueError('Invalid image region coordinates')
    row=0 if v<.25 else 1 if v<.75 else 2
    col=0 if u<.25 else 1 if u<.75 else 2
    profile='near' if [row,col] in NEAR_CELLS else 'far'
    return profile,dict(row_from_robot_near= row+1,column_from_robot_image_left=col+1,
        image_region_uv=[u,v],profile=profile,XYZ_snapped_to_grid=False)


def target_candidate(point,anchor,near_binding,grid):
    far=geometry_candidate(point,anchor)
    profile,region=select_profile(point,grid)
    if profile=='far':result=copy.deepcopy(far)
    else:
        bound=accepted_near_binding(near_binding,anchor)
        old=bound['reference_pregrasp_tcp_pose_m_rad'];origin=bound['reference_top_center_world_m']
        delta=[point['top_center_world_m'][i]-origin[i] for i in range(3)]
        pose=[old[i]+delta[i] for i in range(3)]+old[3:]
        hover=pose[:];hover[2]+=.180
        success=near_success();result=copy.deepcopy(far)
        result.update(scope='live_visual_translation_from_actual140429_near_pickup',
            pregrasp_tcp_candidate_m_rad=pose,hover_tcp_candidate_m_rad=hover,
            delta_from_reference_world_mm=[d*1000 for d in delta],fixed_wrist_rpy_rad=old[3:],
            prepare_recipe=success['prepare_targets'][:],grip_recipe=success['grip_targets'][:])
    result.update(wrist_profile=profile,wrist_region=region,region_only_selects_wrist=True,
        target_XYZ_from_current_vision=True,slot_lookup_used=False,grid_snap_used=False)
    return result


def accepted_request(request):
    if request.get('mode') in ('resume_transfer','deliver_current_held'):
        from visual_transfer_resume_v21 import load_completed_lift
        proof=request.get('transfer_resume_provenance',{})
        original,expected=load_completed_lift(ROOT,proof.get('original_result_file'))
        if proof!=expected:raise ValueError('Completed-lift continuation provenance differs')
        takeover=request.get('mode')=='deliver_current_held'
        if takeover:
            from visual_transfer_resume_v21 import load_current_held_takeover
            context=request.get('current_held_takeover_context',{})
            source,source_proof,expected_context=load_current_held_takeover(ROOT,context.get('rejected_result_file'))
            if source!=original or source_proof!=proof or context!=expected_context:
                raise ValueError('Current-held takeover source chain differs')
        for key in ('wrist_profile','success_source_sha256','anchor','near_binding','wrist_regions','point_record','candidate'):
            if request.get(key)!=original.get(key):raise ValueError('Original visual target/wrist changed: '+key)
        success,candidate=accepted_request(original)
        required=['exclusive','onsite_stop','inert_shell','fixed_head_board_mount',
                'whole_route_and_sweep_clear','same_current_held_shell',
                'profile_carry_common_upper_authorized','batch_route_and_scan_return_authorized']
        required+=(['operator_confirms_current_shell_is_stably_held',
                    'current_pose_takeover_not_original_endpoint_resume',
                    'current_whole_held_route_to_common_upper_clear'] if takeover else
                   ['no_release_move_or_restart_since_success','resume_current_held_after_completed_lift_confirmed'])
        for key in required:
            if request.get('scene_permission',{}).get(key) is not True:raise ValueError('Current permission missing: '+key)
        return success,candidate
    mode=request.get('mode');profile=request.get('wrist_profile','far')
    success,digest=success_for(profile)
    if request.get('success_source_sha256')!=digest:raise ValueError('Selected wrist source SHA differs')
    if mode not in ('deliver_held','park_empty','visual_cycle'):raise ValueError('Unknown action mode')
    required=['exclusive','onsite_stop','inert_shell','fixed_head_board_mount','whole_route_and_sweep_clear']
    if mode=='deliver_held':
        required+=['same_current_held_shell','no_release_move_or_restart_since_success']
        if profile!='near':raise ValueError('h continues only the actual140429 held near shell')
    else:required+=['fresh_actual_open_posture_required','initial_stationary_group_open_authorized',
        'initial_opening_and_inert_shell_drop_zone_clear']
    if mode=='park_empty' and profile!='far':raise ValueError('Observation parking uses the original accepted far upper')
    if mode=='visual_cycle':required+=['target_unchanged_since_capture','continuous_pick_sort_authorized']
    if profile=='near' and mode in ('deliver_held','visual_cycle'):
        required+=['profile_carry_common_upper_authorized']
    if request.get('return_to_scan_upper_after_drop') is True:
        required+=['batch_route_and_scan_return_authorized']
    for name in required:
        if request.get('scene_permission',{}).get(name) is not True:raise ValueError('Current permission missing: '+name)
    candidate=None
    if mode=='visual_cycle':
        candidate=target_candidate(request['point_record'],request['anchor'],request['near_binding'],request['wrist_regions'])
        if not same_measurement(candidate,request.get('candidate')) or candidate['wrist_profile']!=profile:
            raise ValueError('Current visual wrist selection/candidate changed')
    return success,candidate
