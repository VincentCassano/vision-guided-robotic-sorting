"""Three current YOLO colors; retain the accepted near/far visual wrist map."""
import copy
import hashlib
import json
import math
from visual_pick_sort_v22_source import (ROOT,SUCCESS_SHA,NEAR_SHA,accepted_anchor,success_for,
    make_near_binding,accepted_near_binding,grid_matrix,far_success)
import visual_pick_sort_v22_source as base
from cylinder_color_policy_v23 import accepted_classification

MANIFEST=ROOT/'deployment/visual-pick-sort-v23-files.sha256'


def post_drop_raise_mm(color):
    if color not in ('black','red','white'):raise ValueError('Unknown drop color')
    setting=json.loads((ROOT/'deployment/color_bin_adjustment_v23.json').read_bytes())
    value=setting.get('post_drop_raise_mm',{}).get(color,80.)
    if type(value) not in (int,float) or not math.isfinite(value) or value<=0:
        raise ValueError('Invalid selected post-drop vertical distance')
    return float(value)


def verify_delivery():
    checked={}
    for line in MANIFEST.read_text(encoding='ascii').splitlines():
        digest,name=line.split('  ',1);path=(ROOT/name).resolve()
        if ROOT not in path.parents or name in checked or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
            raise ValueError('Delivery file differs: '+name)
        checked[name]=digest
    if not checked:raise ValueError('Empty delivery manifest')
    return checked


def target_candidate(point,anchor,near_binding,grid):
    result=base.target_candidate(point,anchor,near_binding,grid)
    color=accepted_classification(point)
    result.update(target_classification=color,target_color=color['color'],logical_bin=color['logical_bin'])
    return result


def accepted_request(request):
    if request.get('mode')=='resume_failed_transfer':
        from visual_transfer_failure_v23 import load_failed_transfer
        proof=request.get('transfer_failure_provenance',{})
        original,expected=load_failed_transfer(ROOT,proof.get('original_result_file'))
        if proof!=expected:raise ValueError('Failed-transfer proof differs')
        for key in ('wrist_profile','success_source_sha256','anchor','near_binding','wrist_regions',
                'point_record','candidate','target_classification'):
            if request.get(key)!=original.get(key):raise ValueError('Held target identity differs: '+key)
        for key in ('exclusive','onsite_stop','inert_shell','fixed_head_board_mount',
                'whole_route_and_sweep_clear','batch_route_and_scan_return_authorized',
                'same_shell_held_no_move_release_or_restart','resume_failed_transfer_authorized'):
            if request.get('scene_permission',{}).get(key) is not True:
                raise ValueError('Held continuation condition missing: '+key)
        return accepted_request(original)
    if request.get('mode') not in ('park_empty','visual_cycle'):
        raise ValueError('v23 supports fresh visual batches; keep the prior launcher for prior recovery records')
    if request['mode']=='park_empty':return base.accepted_request(request)
    candidate=target_candidate(request['point_record'],request['anchor'],request['near_binding'],request['wrist_regions'])
    if not base.same_measurement(request.get('candidate'),candidate) or request.get('target_classification')!=candidate['target_classification']:
        raise ValueError('Current class/geometry/wrist differs from its retained image evidence')
    original=copy.deepcopy(request)
    original['candidate']=base.target_candidate(request['point_record'],request['anchor'],request['near_binding'],request['wrist_regions'])
    success,_=base.accepted_request(original)
    return success,candidate


def bin_rows(color):
    if color not in ('black','red','white'):raise ValueError('Unknown target color; no default black-bin fallback')
    rows,black_offset=base.bin_rows()
    if color=='black':
        return adjusted_bin(rows,color,black_offset)
    route=json.loads((ROOT/'deployment/school_battery_bins.json').read_bytes())
    setting=json.loads((ROOT/'deployment/color_bin_adjustment_v23.json').read_bytes())
    source_color=setting.get('red_white_shared_pose_source',color)
    if source_color not in ('red','white'):raise ValueError('Invalid shared waste-bin pose source')
    if 'red_white_shared_pose_source' in setting and setting.get('user_confirmed_shared_waste_bin_pose') is not True:
        raise ValueError('Shared waste-bin pose requires the recorded operator choice')
    final=copy.deepcopy(route['bins'][source_color])
    if final.get('phase')!='school_'+source_color+'_drop' or final.get('logical_bin')!='red_white_bin':
        raise ValueError('School red/white bin identity differs')
    final['phase']='school_'+color+'_drop'
    pose=final.get('pose')
    if not isinstance(pose,list) or len(pose)!=6 or any(type(v) not in (int,float) or not math.isfinite(v) for v in pose):
        raise ValueError('School colored-bin pose is malformed')
    rows[-1]=final
    return adjusted_bin(rows,color,dict(target_color=color,distance_mm=0,delta_world_mm=[0.,0.,0.],
        source=final['source'],logical_bin='red_white_bin',drop_pose_source_color=source_color,
        red_white_shared_pose=source_color!=color or setting.get('red_white_shared_pose_source') is not None,
        black40mm_task_correction_applied=False,red_white_route_field_accepted=False))


def adjusted_bin(rows,color,previous_offset):
    setting=json.loads((ROOT/'deployment/color_bin_adjustment_v23.json').read_bytes())
    if setting.get('user_confirmed_additional_not_replacement') is not True:
        raise ValueError('Drop adjustment must retain the confirmed additional-offset semantics')
    transform=json.loads((ROOT/'deployment/camera_position_candidate_225543.json').read_bytes())['world_from_camera']
    backward=[-float(transform[0][2]),-float(transform[1][2]),0.]
    right=[float(transform[0][0]),float(transform[1][0]),0.]
    for axis in (backward,right):
        length=math.hypot(axis[0],axis[1])
        if not math.isfinite(length) or length<1e-9:
            raise ValueError('Fixed robot-view horizontal direction unavailable')
        axis[0]/=length;axis[1]/=length
    current=setting[color]
    back_mm=current['additional_backward_mm'];right_mm=current['additional_right_mm']
    if any(type(v) not in (int,float) or not math.isfinite(v) for v in (back_mm,right_mm)):
        raise ValueError('Drop adjustment must contain finite millimetres')
    delta=[(back_mm*backward[i]+right_mm*right[i])/1000. for i in range(3)]
    up_mm=setting.get('red_white_additional_up_mm',0.) if color in ('red','white') else 0.
    if type(up_mm) not in (int,float) or not math.isfinite(up_mm):raise ValueError('Invalid drop height offset')
    delta[2]+=up_mm/1000.
    previous=rows[-1]['pose'][:]
    for i in range(3):rows[-1]['pose'][i]+=delta[i]
    old=previous_offset['delta_world_mm']
    return rows,dict(target_color=color,scope=setting['scope'],
        source_adjustment_sha256=hashlib.sha256((ROOT/'deployment/color_bin_adjustment_v23.json').read_bytes()).hexdigest(),
        previous_offset=previous_offset,previous_drop_pose_m_rad=previous,
        additional_backward_mm=back_mm,additional_right_mm=right_mm,
        additional_up_mm=up_mm,
        robot_view_backward_unit_World=backward,robot_view_right_unit_World=right,
        additional_delta_world_mm=[v*1000 for v in delta],
        total_delta_from_school_world_mm=[old[i]+delta[i]*1000 for i in range(3)],
        adjusted_drop_pose_m_rad=rows[-1]['pose'][:],
        height_and_orientation_unchanged=up_mm==0,orientation_unchanged=True,
        applied_once_from_immutable_source=True,
        physical_new_drop_acceptance=False)
