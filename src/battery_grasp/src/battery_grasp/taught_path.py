"""Compile and numerically assess taught approach candidates. No actuators."""
import copy
import hashlib
import math
import json
from .manual_teaching import check_taught_point, endpoint_delta, PREPARE
from .real_grasp import vector

CAPTURE_SHA='e2bf4c18faec5184f3a47ff2dade3d344a334af8fc4aa15faba7fecebe48fa5a'
NATIVE_SHA='052f94a92d5c7320f227ea811916375fa0d352250329570bd50e0c7abe603ad5'
FK_POSITION_TOLERANCE_MM=1.0
FK_ROTATION_TOLERANCE_DEGREES=0.5


def six(values,name):
    if (not isinstance(values,(list,tuple)) or len(values)!=6
            or any(type(v) not in (int,float) or not math.isfinite(v) for v in values)):
        raise ValueError('Six finite values required: '+name)
    return list(values)


def reserves(joints,limits):
    q=six(joints,'joints');lo=six(limits['minimum'],'minimum');hi=six(limits['maximum'],'maximum')
    if any(a>=b for a,b in zip(lo,hi)):raise ValueError('Reversed joint limits')
    return [min(v-a,b-v) for v,a,b in zip(q,lo,hi)]


def accepted_capture(raw):
    if hashlib.sha256(raw).hexdigest()!=CAPTURE_SHA:
        raise ValueError('Only the audited212012 completed teaching capture is accepted')
    d=json.loads(raw)
    if (d.get('outcome')!='taught_pregrasp_endpoint_saved_path_unverified'
            or d.get('diagnostic_pass') is not True or d.get('execution_ready') is not False
            or d.get('hardware_arm_writes')!=0 or d.get('hand_writes_allowed') is not False
            or d.get('exceptions')!=[] or d.get('cleanup_errors')!=[]
            or d.get('same_scene_operator_confirmed') is not True
            or d.get('native_sdk_isolated') is not True or d.get('sdk_in_parent') is not False
            or d.get('sdk_environment',{}).get('loaded_native',{}).get('sha256')!=NATIVE_SHA):
        raise ValueError('Completed read-only teaching evidence and unchanged scene required')
    points=[d['upper_reference']]+d['points']
    if len(points)!=4 or [p['role'] for p in points]!=['upper_reference','approach_waypoint','approach_waypoint','operator_pregrasp']:
        raise ValueError('Audited point sequence differs')
    ordinary=points[0]['ordinary_limits'];drive=points[0]['drive_limits'];frames=points[0]['frame_signatures']
    for p in points:
        # This validates the immutable collector record's hardware/data format.
        # It does not endorse the legacy scene-clearance label. compact() applies
        # the user's later table-only correction before any candidate is emitted.
        check_taught_point(p)
        if p['ordinary_limits']!=ordinary or p['drive_limits']!=drive or p['frame_signatures']!=frames:
            raise ValueError('Limits/frames differ between endpoints')
        for field,limits in (('joint_reserve_degrees',ordinary),('drive_reserve_degrees',drive)):
            calculated=reserves(p['joint_degrees'],limits)
            if any(abs(a-b)>1e-7 for a,b in zip(calculated,p[field])):
                raise ValueError('Reported joint reserves do not match measured joints/limits')
    if points[-1].get('operator_pregrasp_placement_confirmed') is not True:
        raise ValueError('True pregrasp placement confirmation missing')
    install=d['arm_safety_reads']['Get_Install_Pose']
    if install!=[0,0.0,45.0,0.0]:raise ValueError('Audited installation angles changed')
    return d


def compact(p,index):
    result=dict(role=p['role'],source_point_indices=[index],tool_world_mm=p['tool_world_mm'][:],
        tool_rpy_rad=p['tool_rpy_rad'][:],joint_degrees=p['joint_degrees'][:],
        operator_measured_clearance_mm=p['clearance_mm'],hand_target=PREPARE[:])
    if p['role']=='operator_pregrasp':
        # Human correction supersedes the original collector's broad field label.
        # Preserve the immutable raw report; do not infer battery/obstacle gaps.
        result.update(operator_measured_clearance_mm=None,recorded_legacy_clearance_mm=p['clearance_mm'],
            lowest_hand_outer_surface_to_table_mm=p['clearance_mm'],minimum_whole_hand_to_scene_mm=None,
            hand_to_battery_clearance_mm=None,hand_to_other_obstacles_clearance_mm=None,
            clearance_scope='lowest_hand_outer_surface_to_table_only',
            clearance_scope_source='user_explicit_correction_2026-10-11',
            original_capture_scene_clearance_label_superseded=True)
    return result


def compile_candidate(raw,joint_step_degrees=0.5):
    if type(joint_step_degrees) not in (int,float) or not math.isfinite(joint_step_degrees) or not 0.05<=joint_step_degrees<=1:
        raise ValueError('Numerical joint sampling step must be0.05..1degree; not a motion increment')
    d=accepted_capture(raw);upper=d['upper_reference'];source=[upper]+d['points']
    points=[];duplicates=[]
    for index,p in enumerate(source):
        item=compact(p,index)
        if points and p['role']==points[-1]['role']=='approach_waypoint':
            previous=points[-1];delta=endpoint_delta(previous,item)
            if (math.dist(previous['tool_world_mm'],item['tool_world_mm'])<0.1
                    and delta['rotation_angle_degrees']<0.1
                    and max(abs(a-b) for a,b in zip(previous['joint_degrees'],item['joint_degrees']))<0.02):
                item['source_point_indices']=previous['source_point_indices']+[index]
                duplicates.append(dict(indices=item['source_point_indices'][:],tcp_distance_mm=math.dist(previous['tool_world_mm'],item['tool_world_mm'])))
                points[-1]=item
                continue
        points.append(item)
    ordinary=copy.deepcopy(upper['ordinary_limits']);drive=copy.deepcopy(upper['drive_limits'])
    segments=[];samples=[]
    for index,(a,b) in enumerate(zip(points,points[1:])):
        delta=[y-x for x,y in zip(a['joint_degrees'],b['joint_degrees'])]
        count=max(1,math.ceil(max(abs(v) for v in delta)/joint_step_degrees))
        lo=[min(x,y) for x,y in zip(a['joint_degrees'],b['joint_degrees'])]
        hi=[max(x,y) for x,y in zip(a['joint_degrees'],b['joint_degrees'])]
        def continuous_reserve(limits):
            return [min(x-low,high-y) for x,y,low,high in zip(lo,hi,limits['minimum'],limits['maximum'])]
        segment=dict(index=index,start_role=a['role'],end_role=b['role'],joint_delta_degrees=delta,
            endpoint_delta=endpoint_delta(a,b),sample_intervals=count,
            continuous_affine_joint_minimum_reserve_degrees=min(continuous_reserve(ordinary)),
            continuous_affine_drive_minimum_reserve_degrees=min(continuous_reserve(drive)),
            reserve_proof='For each q(t)=(1-t)q0+t*q1, t in[0,1], extrema occur at endpoints; no joint angle wrapping',
            proof_is_controller_trajectory_certificate=False)
        segments.append(segment)
        for k in range(count+1):
            if index and k==0:continue
            t=k/count
            q=[x+t*v for x,v in zip(a['joint_degrees'],delta)]
            samples.append(dict(segment=index,fraction=t,joint_degrees=q,
                ordinary_minimum_reserve_degrees=min(reserves(q,ordinary)),drive_minimum_reserve_degrees=min(reserves(q,drive)),
                tool_world_mm=None,tool_rpy_rad=None,pose_source='unknown_until_native_FK_witnesses_match'))
    final=points[-1];feature=vector(d['target']['surface_world_mm'],'visible feature')
    return dict(schema_version=1,scope='numerical_taught_pregrasp_candidates_only',capture_result_sha256=CAPTURE_SHA,
        frame='World',tool='Arm_Tip',units=dict(tcp='mm',wrist='rad',joints='degree'),
        candidate_ready=True,execution_ready=False,arm_motion_allowed=False,hand_motion_allowed=False,
        physical_stop_verified=False,hardware_writes=0,controller_trajectory_verified=False,
        endpoint_minimum_scene_clearance_certified=False,
        pregrasp_clearance_interpretation=dict(lowest_hand_outer_surface_to_table_mm=14.0,
            minimum_whole_hand_to_scene_mm=None,hand_to_battery_clearance_mm=None,
            hand_to_other_obstacles_clearance_mm=None,source='user_explicit_correction_2026-10-11',
            raw_capture_bytes_preserved=True,table_world_z_not_derived=True),
        ordinary_limits=ordinary,drive_limits=drive,algorithm_installation_degrees=d['arm_safety_reads']['Get_Install_Pose'][1:],
        frame_signatures=copy.deepcopy(upper['frame_signatures']),sdk_source=d['sdk_environment'],
        waypoints=points,merged_duplicate_approach_points=duplicates,segments=segments,joint_samples=samples,
        candidate_affine_joint_limits_pass=all(min(s['continuous_affine_joint_minimum_reserve_degrees'],
            s['continuous_affine_drive_minimum_reserve_degrees'])>=3 for s in segments),
        mapping=dict(surface_to_terminal_tcp_world_mm=[a-b for a,b in zip(final['tool_world_mm'],feature)],
            terminal_rpy_rad=final['tool_rpy_rad'][:],mapping_is_grasp_center=False,
            scope='same inert shell/orientation/visible feature and unchanged camera/scene',execution_allowed=False),
        fixed_wrist_vertical_spec=dict(upper_world_mm=[final['tool_world_mm'][0],final['tool_world_mm'][1],points[0]['tool_world_mm'][2]],
            lower_world_mm=final['tool_world_mm'][:],rpy_rad=final['tool_rpy_rad'][:],
            vertical_step_mm=5.0,high_alignment_pose_is_taught=False,execution_allowed=False),
        blockers=['Native kinematic model must match all measured joint/TCP witnesses',
            '14mm measures lowest hand outer surface to table only; battery/other obstacle gaps remain unknown',
            'Installed whole hand/little finger, arm and cable swept geometry is unknown',
            'Table/battery/obstacle bounds and height/wrist dependent workspace are unknown',
            'Numerical samples do not establish controller interpolation or physical collision clearance',
            'Fresh runtime target, arm/hand preflight and supervised actual path acceptance remain required'])


def pose_record(pose):
    p=six(pose,'native FK pose in metres/radians')
    return dict(tool_world_mm=[v*1000 for v in p[:3]],tool_rpy_rad=p[3:])


def witness_comparison(point,pose):
    record=pose_record(pose);delta=endpoint_delta(point,record)
    error=math.dist(point['tool_world_mm'],record['tool_world_mm'])
    angle=delta['rotation_angle_degrees']
    return dict(role=point['role'],joint_degrees=point['joint_degrees'],measured_tcp_world_mm=point['tool_world_mm'],
        measured_rpy_rad=point['tool_rpy_rad'],calculated=record,translation_error_mm=error,rotation_error_degrees=angle,
        pass_agreement=error<=FK_POSITION_TOLERANCE_MM and angle<=FK_ROTATION_TOLERANCE_DEGREES)


def evaluate_candidates(candidate,fk,ik,emit):
    """fk/ik are isolated local numerical capabilities, never robot instances."""
    result=dict(schema_version=1,native_model_agreement=False,joint_candidate_calculated=False,
        vertical_candidate_calculated=False,execution_ready=False,hardware_writes=0,exceptions=[],
        witness_thresholds=dict(position_mm=FK_POSITION_TOLERANCE_MM,rotation_degrees=FK_ROTATION_TOLERANCE_DEGREES,
            source='engineering model-agreement thresholds only; not device protection or safe clearance'),
        witnesses=[],joint_path=[],vertical_path=[],blockers=candidate['blockers'][:])
    for point in candidate['waypoints']:
        try:
            emit(dict(event='fk_witness_before',role=point['role'],joint_degrees=point['joint_degrees']))
            pose=fk(point['joint_degrees']);emit(dict(event='fk_witness_result',role=point['role'],pose=pose))
            result['witnesses'].append(witness_comparison(point,pose))
        except Exception as exc:
            result['witnesses'].append(dict(role=point['role'],pass_agreement=False,error=repr(exc)))
    result['native_model_agreement']=all(row['pass_agreement'] for row in result['witnesses'])
    if not result['native_model_agreement']:
        result['blockers'].append('Native FK differs from actual measured endpoints; no route FK/IK performed; no fitted correction')
        return result
    result['blockers'].remove('Native kinematic model must match all measured joint/TCP witnesses')
    try:
        for index,sample in enumerate(candidate['joint_samples']):
            pose=fk(sample['joint_degrees']);emit(dict(event='fk_joint_sample',index=index,joint_degrees=sample['joint_degrees'],pose=pose))
            row=copy.deepcopy(sample);row.update(pose_record(pose),pose_source='matched_native_algorithm_numerical_candidate')
            result['joint_path'].append(row)
        result['joint_candidate_calculated']=True
    except Exception as exc:
        result['exceptions'].append(dict(phase='joint_candidate_FK',error=repr(exc)))
    spec=candidate['fixed_wrist_vertical_spec'];bottom=spec['lower_world_mm'];top=spec['upper_world_mm']
    count=max(1,math.ceil((top[2]-bottom[2])/spec['vertical_step_mm']))
    seed=candidate['waypoints'][-1]['joint_degrees'][:];rows=[]
    try:
        # Solve upward from the known pregrasp branch, then reverse for a descent candidate.
        for k in range(count+1):
            wanted=[bottom[0],bottom[1],bottom[2]+(top[2]-bottom[2])*k/count]
            pose=[v/1000 for v in wanted]+spec['rpy_rad']
            if k==0:tag,q=0,seed[:]
            else:
                emit(dict(event='ik_vertical_before',upward_index=k,seed_degrees=seed[:],pose_m_rad=pose))
                tag,returned=ik(seed[:],pose)
                emit(dict(event='ik_vertical_result',upward_index=k,status=tag,joints=returned))
                if type(tag) is not int or tag!=0:raise ValueError('Vertical IK failed at upward sample'+str(k)+': '+str(tag))
                if not isinstance(returned,(tuple,list)) or len(returned) not in (6,7):raise ValueError('Unexpected IK vector length')
                q=six(list(returned[:6]),'IK joints')
            normal=min(reserves(q,candidate['ordinary_limits']));drive=min(reserves(q,candidate['drive_limits']))
            if min(normal,drive)<3:raise ValueError('Vertical IK joint reserve below3degrees')
            jump=max(abs(a-b) for a,b in zip(seed,q))
            if jump>10:raise ValueError('IK branch jump greater than10degrees per <=5mm numerical sample')
            computed=fk(q);emit(dict(event='fk_vertical_check',upward_index=k,joint_degrees=q,pose=computed))
            compared=witness_comparison(dict(role='vertical_'+str(k),joint_degrees=q,tool_world_mm=wanted,tool_rpy_rad=spec['rpy_rad']),computed)
            if not compared['pass_agreement']:raise ValueError('Vertical IK/FK roundtrip does not match desired pose')
            rows.append(dict(upward_index=k,desired_world_mm=wanted,desired_rpy_rad=spec['rpy_rad'],joint_degrees=q,
                ordinary_minimum_reserve_degrees=normal,drive_minimum_reserve_degrees=drive,
                adjacent_joint_change_degrees=jump,calculated=compared['calculated']))
            seed=q
        result['vertical_path']=list(reversed(rows));result['vertical_candidate_calculated']=True
        result['blockers'].append('Computed fixed-wrist aligned upper pose and orientation transition were not taught or certified')
    except Exception as exc:
        result['partial_vertical_upward_path']=rows
        result['exceptions'].append(dict(phase='vertical_candidate_IK_FK',error=repr(exc)))
    return result
