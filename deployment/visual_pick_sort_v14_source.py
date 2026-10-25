"""Actual044001 pickup provenance and live equal-height visual task binding."""
import copy
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/battery_grasp/src'))
MANIFEST=ROOT/'deployment/visual-pick-sort-v14-files.sha256'
SUCCESS=ROOT/'deployment/grasp_success_044001/success_record.json'
SUCCESS_SHA='8a4a92c9cd2adf2b6976f6cdde823b8b33364d8a9522953bff8bc514f69b192b'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verify_delivery():
    inventory={}
    for line in MANIFEST.read_text(encoding='ascii').splitlines():
        digest,name=line.split('  ',1);path=(ROOT/name).resolve()
        if ROOT not in path.parents or name in inventory or sha(path)!=digest:
            raise ValueError('Delivery file differs: '+name)
        inventory[name]=digest
    if not inventory:raise ValueError('Empty delivery manifest')
    return inventory


def accepted_success():
    from grasp_rebuild_records_v1 import assemble_success
    if sha(SUCCESS)!=SUCCESS_SHA:raise ValueError('Actual044001 success record differs')
    value=json.loads(SUCCESS.read_bytes())
    expected=assemble_success(value['start_record'],value['grip_shape_record'],
        value['prepare_shape_record'],value['wrist_record'],value['lift_result'],
        value.get('operator_observed_pickup_success'))
    for key,item in expected.items():
        if value.get(key)!=item:raise ValueError('Actual pickup provenance differs: '+key)
    return value


def point_checked(point):
    if not isinstance(point,dict) or point.get('old_grasp_tcp_mapping_used') is not False:
        raise ValueError('Fresh image point without old wrist/TCP mapping required')
    summary=point.get('burst_summary',{})
    if (summary.get('burst_qualified') is not True or summary.get('frame_quality_pass') is not True
            or summary.get('frame_count',0)<8 or summary.get('elapsed_seconds',0)<2
            or summary.get('reject_reasons') or summary.get('top_center_repeatability_max_mm',math.inf)>3):
        raise ValueError('Original8-frame/2s/3mm image burst required')
    top=point.get('top_center_world_m')
    if (not isinstance(top,list) or len(top)!=3
            or any(type(v) not in (int,float) or not math.isfinite(v) for v in top)
            or top!=summary.get('top_center_world_m')):
        raise ValueError('Image point differs from its complete retained burst')
    if not isinstance(point.get('equal_height_plane'),dict) or not isinstance(point.get('binding_camera_context'),dict):
        raise ValueError('Current plane/camera provenance required')
    return top


def make_anchor(point,original_place_confirmed):
    if original_place_confirmed is not True:
        raise ValueError('Same actual successful pickup place must be confirmed')
    success=accepted_success();top=point_checked(point)
    pose=success['reference_pregrasp_tcp_pose_m_rad']
    return dict(schema_version=14,scope='actual044001_grip_wrist_equal_height_visual_binding',
        success_source_sha256=SUCCESS_SHA,task_binding_recorded=True,
        reference_pregrasp_tcp_pose_m_rad=pose[:],reference_top_center_world_m=top[:],
        top_center_to_pregrasp_TCP_world_m=[pose[i]-top[i] for i in range(3)],
        reference_point_record=copy.deepcopy(point),camera_context=copy.deepcopy(point['binding_camera_context']),
        equal_height_plane=copy.deepcopy(point['equal_height_plane']),
        prepare_targets=success['prepare_targets'][:],grip_targets=success['grip_targets'][:],
        drop_targets=[0,0,0,0,0,success['grip_targets'][5]],lift_mm=180,
        original_actual_pickup_place_confirmed=True,old_wrist_mapping_used=False,
        old_2p8_plus_4p5_offsets_used=False,slot_lookup_used=False,grid_snap_used=False,
        fault_last_state_used=False,handeye_refitted=False,full6D_TCP_calibrated=False,
        absolute_position_accuracy_accepted=False,complete_visual_pick_sort_accepted=False,
        motion_authorized=False,execution_ready=False)


def accepted_anchor(value):
    if not isinstance(value,dict):raise ValueError('Visual reference not bound yet')
    expected=make_anchor(value.get('reference_point_record'),value.get('original_actual_pickup_place_confirmed'))
    if value!=expected:raise ValueError('Visual binding differs from actual044001 pickup and image evidence')
    return expected


def target_candidate(point,anchor):
    accepted_anchor(anchor);top=point_checked(point)
    if (point['binding_camera_context']!=anchor['camera_context']
            or point['equal_height_plane']!=anchor['equal_height_plane']):
        raise ValueError('Target camera/model/plane differs from the new binding')
    old=anchor['reference_pregrasp_tcp_pose_m_rad'];origin=anchor['reference_top_center_world_m']
    delta=[top[i]-origin[i] for i in range(3)]
    pose=[old[i]+delta[i] for i in range(3)]+old[3:]
    upper=pose[:];upper[2]+=.180
    return dict(scope='live_visual_translation_from_actual044001_pickup',
        pregrasp_tcp_candidate_m_rad=pose,hover_tcp_candidate_m_rad=upper,
        delta_from_reference_world_mm=[v*1000 for v in delta],
        fixed_wrist_rpy_rad=old[3:],prepare_recipe=anchor['prepare_targets'][:],
        grip_recipe=anchor['grip_targets'][:],lift_mm=180,
        slot_lookup_used=False,grid_snap_used=False,old_offsets_applied=False,
        actual_finger_arrival_required=False,motion_authorized=False,execution_ready=False)


def accepted_request(request):
    success=accepted_success()
    if request.get('success_source_sha256')!=SUCCESS_SHA:
        raise ValueError('Current request must use actual044001 successful pickup')
    mode=request.get('mode')
    if mode not in ('deliver_held','park_empty','visual_cycle'):
        raise ValueError('Unsupported current cycle mode')
    permission=request.get('scene_permission',{})
    required=['exclusive','onsite_stop','inert_shell','fixed_head_board_mount','whole_route_and_sweep_clear']
    required+=['same_current_held_shell','no_release_move_or_restart_since_success'] if mode=='deliver_held' else ['hand_empty']
    if mode=='visual_cycle':required+=['target_unchanged_since_capture','continuous_pick_sort_authorized']
    for key in required:
        if permission.get(key) is not True:raise ValueError('Current permission missing: '+key)
    candidate=None
    if mode=='visual_cycle':
        candidate=target_candidate(request['point_record'],request['anchor'])
        if request.get('candidate')!=candidate:raise ValueError('Current image target candidate was altered')
    return success,candidate


def bin_rows():
    route=json.loads((ROOT/'deployment/school_battery_bins.json').read_bytes())
    if (route.get('robot_ip')!='169.254.128.18' or route.get('unit')!='metres_radians'
            or route.get('frame_names')!={'work':'World','tool':'Arm_Tip'}):
        raise ValueError('School battery bin identity/frame differs')
    rows=copy.deepcopy(route['transit']+[route['bins']['black']])
    if len(rows)!=3 or [r['phase'] for r in rows]!=['school_high_transit','school_restart_transit','school_black_drop']:
        raise ValueError('Exact three-point black-bin route required')
    transform=json.loads((ROOT/'deployment/camera_position_candidate_225543.json').read_bytes())['world_from_camera']
    x,y=transform[0][2],transform[1][2];length=math.hypot(x,y)
    if not math.isfinite(length) or length<1e-9:raise ValueError('Fixed-camera horizontal direction unavailable')
    offset=[-.040*x/length,-.040*y/length,0.]
    for i in range(3):rows[-1]['pose'][i]+=offset[i]
    for row in rows:
        if len(row['pose'])!=6 or any(type(v) not in (int,float) or not math.isfinite(v) for v in row['pose']):
            raise ValueError('Malformed school bin pose')
    return rows,dict(distance_mm=40,direction='toward_robot_fixed_camera_horizontal',delta_world_mm=[v*1000 for v in offset])
