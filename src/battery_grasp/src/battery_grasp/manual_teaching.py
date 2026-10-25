"""Pure evaluation of manually taught endpoints, never a motion executor."""
import copy
import hashlib
import json
import math
from .real_grasp import vector, rotation

RESUME_SHA='b4ec8ac98c92a6393c4168a49318fa9bf5c66804f2c92c4fbe61fbc367418d2a'
PREPARE=[8000,8000,8000,8000,65535,65535]
CALIBRATION_SHA='70809f5b27269949f42977fbec9923c8d384f728d8eef1653317edeb320bc764'
TEACHING_RESUME_SHA='be8b0228e10c60f5f562349d4e1ede18eb70ef1dbaa4cf5f4d9aa60b2d910d22'


def reviewed_resume(raw, profile, hover):
    if hashlib.sha256(raw).hexdigest()!=RESUME_SHA:
        raise ValueError('Only audited193957 reference data can be reused; no connection')
    source=json.loads(raw)
    if (source.get('outcome')!='aborted_no_further_writes'
            or source.get('hardware_arm_writes')!=0 or source.get('cleanup_errors')
            or source.get('empty_prepare_feedback_pass') is not True
            or source.get('configuration')!=profile['observed_configuration']
            or source.get('vision_process',{}).get('return_code')!=0
            or source.get('upper',{}).get('accepted') is not True
            or source['upper'].get('frame_signatures')!=hover['frame_signatures']
            or source['upper'].get('ordinary_limits')!=hover['limits']):
        raise ValueError('Audited successful camera/upper/empty-hand evidence required')
    target=source['target'];upper=source['upper']
    if (target.get('serial')!='344322075008' or target.get('world_transform_sha256')!=CALIBRATION_SHA
            or target.get('point_kind')!='visible_surface_not_grasp_center'
            or upper.get('hand_target')!=PREPARE or upper.get('clearance_mm',0)<50
            or upper.get('hand_after',{}).get('force_targets')!=[0]*5):
        raise ValueError('Target or upper reference differs from reviewed source')
    upper=copy.deepcopy(upper)
    upper.update(role='upper_reference',data_pass=True,source='audited193957 upper',
                 operator_pregrasp_placement_confirmed=False)
    upper['drive_reserve_degrees']=[min(q-a,b-q) for q,a,b in zip(upper['joint_degrees'],
        upper['drive_limits']['minimum'],upper['drive_limits']['maximum'])]
    return source,copy.deepcopy(target),upper


def endpoint_delta(a,b):
    pa,pb=vector(a['tool_world_mm'],'previous TCP'),vector(b['tool_world_mm'],'current TCP')
    ra,rb=vector(a['tool_rpy_rad'],'previous wrist'),vector(b['tool_rpy_rad'],'current wrist')
    wrapped=[math.atan2(math.sin(y-x),math.cos(y-x)) for x,y in zip(ra,rb)]
    r1,r2=rotation(ra),rotation(rb)
    cosine=(sum(r1[i][j]*r2[i][j] for i in range(3) for j in range(3))-1)/2
    return dict(tcp_delta_world_mm=[y-x for x,y in zip(pa,pb)],xy_delta_mm=math.dist(pa[:2],pb[:2]),
        rpy_delta_degrees=[math.degrees(v) for v in wrapped],
        rotation_angle_degrees=math.degrees(math.acos(max(-1,min(1,cosine)))),
        fixed_world_z_endpoint_pair=math.dist(pa[:2],pb[:2])<=.5 and pb[2]<pa[2]
            and all(abs(v)<=.005 for v in wrapped),
        interpolated_path_verified=False)


def reviewed_teaching_resume(raw,original_raw,profile,hover):
    """Reuse verified data from the audited terminal failure, never its authority."""
    original,target,upper=reviewed_resume(original_raw,profile,hover)
    digest=hashlib.sha256(raw).hexdigest()
    if digest==RESUME_SHA:
        return original,target,upper,[]
    if digest!=TEACHING_RESUME_SHA:
        raise ValueError('Only audited193957 or210330 data can be resumed; no connection')
    document=json.loads(raw)
    if (document.get('outcome')!='aborted_readonly_no_actuator_writes'
            or document.get('hardware_arm_writes')!=0 or document.get('hand_writes_allowed') is not False
            or document.get('cleanup_errors') or document.get('execution_ready') is not False
            or document.get('resume_source_sha256')!=RESUME_SHA
            or document.get('target')!=target or document.get('upper_reference')!=upper
            or document.get('configuration')!=profile['observed_configuration']
            or document.get('force_pid_on_device')!=original['force_pid_on_device']
            or len(document.get('exceptions',[]))!=1
            or document['exceptions'][0].get('error')!="OSError(9, 'Bad file descriptor')"):
        raise ValueError('Audited terminal-only failure and unchanged successful data required')
    points=document.get('points')
    if not isinstance(points,list) or len(points)!=1 or points[0].get('role')!='approach_waypoint':
        raise ValueError('Require the audited single successful approach waypoint')
    check_taught_point(points[0])
    if points[0]['frame_signatures']!=hover['frame_signatures'] or points[0]['ordinary_limits']!=hover['limits']:
        raise ValueError('Resumed waypoint frame/limits changed')
    return original,target,upper,copy.deepcopy(points)


def check_taught_point(point):
    if point.get('data_pass') is not True or point.get('hand_target')!=PREPARE:
        raise ValueError('Fresh prepared-hand endpoint brackets required')
    vector(point.get('tool_world_mm'),'taught TCP');vector(point.get('tool_rpy_rad'),'taught wrist')
    joints=point.get('joint_degrees');reserve=point.get('joint_reserve_degrees')
    drive=point.get('drive_reserve_degrees')
    for values,name in ((joints,'joints'),(reserve,'ordinary reserves'),(drive,'drive reserves')):
        if (not isinstance(values,list) or len(values)!=6
                or any(type(v) not in (int,float) or not math.isfinite(v) for v in values)):
            raise ValueError('Invalid '+name)
    if any(v<3 for v in reserve+drive): raise ValueError('Joint endpoint reserve below3 degrees')
    gap=point.get('clearance_mm')
    if type(gap) not in (int,float) or not math.isfinite(gap) or gap<10:
        raise ValueError('Measured whole-hand scene clearance must be >=10mm')
    if point.get('clearance_source')!='operator_measured_minimum_whole_hand_to_scene_mm':
        raise ValueError('Measured clearance source required')
    for name in ('hand_before','hand_after'):
        row=point.get(name,{})
        if row.get('force_targets')!=[0]*5 or row.get('speeds')!=[65535]*6 or row.get('current_limit_ma')!=[1299]*6:
            raise ValueError('Zero force and unchanged measured speed/protection required')
        for field in ('targets','positions','status','current'):
            values=row.get(field)
            if not isinstance(values,list) or len(values)!=6 or any(type(v) is not int or not 0<=v<=65535 for v in values):
                raise ValueError('Incomplete hand bracket '+field)
        if any(s not in (0,1,2,5) for s in row['status']) or any(c>1299 for c in row['current']):
            raise ValueError('Hand protection or invalid feedback in endpoint bracket')
        if any(abs(a-b)>2 for a,b in zip(row['targets'],PREPARE)) or any(abs(a-b)>150 for a,b in zip(row['positions'],PREPARE)):
            raise ValueError('Prepared hand changed before or after endpoint read')
    return True


def make_teaching_draft(target, upper, points):
    """Endpoints define a draft; freehand motion between them was not recorded."""
    result=dict(schema_version=1,draft_ready=False,execution_ready=False,hardware_writes=0,
        path_type='manually_taught_joint_endpoints',automatic_execution_allowed=False,
        intermediate_tcp_path_known=False,physical_stop_verified=False,
        whole_route_geometry_certified=False,grasp_geometry_verified=False,
        target=copy.deepcopy(target),waypoints=[],endpoint_deltas=[],blockers=[
            'Full hand/transition, arm and cable swept geometry is not measured',
            'Table, battery and obstacle collision bounds are not measured',
            'Height and wrist dependent workspace bounds are not measured',
            'Controller trajectory interpolation and full-route >=3deg joint reserve are not checked',
            'Fresh execution target and absolute vision accuracy are not certified',
            'Manual endpoints do not certify the path between them'])
    try:
        vector(target.get('surface_world_mm'),'visible target surface')
        check_taught_point(upper)
        previous=upper
        for point in points:
            check_taught_point(point)
            if point.get('role') not in ('approach_waypoint','operator_pregrasp'):
                raise ValueError('Unknown taught point role')
            if point['role']=='operator_pregrasp' and point.get('operator_pregrasp_placement_confirmed') is not True:
                raise ValueError('Pregrasp requires explicit battery-in-grasp-space confirmation')
            result['endpoint_deltas'].append(endpoint_delta(previous,point))
            previous=point
        all_points=[upper]+points
        result['waypoints']=[dict(role=p['role'],tool_world_mm=p['tool_world_mm'],tool_rpy_rad=p['tool_rpy_rad'],
            joint_degrees=p['joint_degrees'],clearance_mm=p['clearance_mm']) for p in all_points]
        if not points or points[-1]['role']!='operator_pregrasp':
            result['blockers'].append('Actual operator-taught pregrasp endpoint is missing')
            return result
        final=points[-1]
        result.update(draft_ready=True,operator_pregrasp_placement_confirmed=True,
            mapping=dict(surface_to_taught_tcp_world_mm=[a-b for a,b in zip(final['tool_world_mm'],target['surface_world_mm'])],
                taught_tool_rpy_rad=final['tool_rpy_rad'][:],mapping_is_grasp_center=False,
                scope='same inert shell, orientation, visible feature and unchanged scene/camera',
                operator_position_is_not_grasp_geometry_certificate=True,automatic_execution_allowed=False))
    except (ValueError,TypeError,KeyError,AttributeError,OverflowError) as exc:
        result['draft_ready']=False;result['blockers'].append(str(exc))
    return result
