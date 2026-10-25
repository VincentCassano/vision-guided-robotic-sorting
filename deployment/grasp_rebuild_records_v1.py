"""Saved-record rules for start -> grip -> prepare -> wrist -> 180mm lift.

Pure data and mathematics only. No driver, network or actuator authority.
An onsite y observation records pickup; motion feedback alone never does.
"""
import copy
import math

from battery_grasp.real_grasp import orientation_angle_rad


LIFT_COMPLETED_OUTCOME='lift_completed_waiting_operator_result'


def targets(value,label='手指目标'):
    """Return six raw unsigned position counts; there is no frozen recipe."""
    if (not isinstance(value,(list,tuple)) or len(value)!=6
            or any(type(v) is not int or not 0<=v<=65535 for v in value)):
        raise ValueError(label+'：需要六路0..65535原始整数。')
    return list(value)


def _numbers(value,count,label):
    if (not isinstance(value,(list,tuple)) or len(value)!=count
            or any(type(v) not in (int,float) or not math.isfinite(v) for v in value)):
        raise ValueError(label+'：需要完整有限数值。')
    return list(value)


def _recorded_hand(row,label):
    if (not isinstance(row,dict) or row.get('ok') is not True
            or row.get('snapshot_stale') is not False or row.get('needs_reconnect')
            or not isinstance(row.get('snapshot'),dict)):
        raise ValueError(label+'：手部读取不完整或已过期。')
    hand=row['snapshot']
    targets(hand.get('targets'),label+'目标')
    targets(hand.get('positions'),label+'实际位置')
    status=targets(hand.get('status'),label+'状态')
    if any(v not in range(6) for v in status):
        raise ValueError(label+'：手部状态值无效。')
    return copy.deepcopy(hand)


def hand_snapshot(row,label='当前手部'):
    """Require a fresh live row, permitting normal stall5 and stopped-short fingers."""
    hand=_recorded_hand(row,label)
    if any(v not in (0,1,2,5) for v in hand['status']):
        raise ValueError(label+'：当前存在真实保护3/4，请先现场处理。')
    return hand


def _arm_data(arm,label,min_reserve):
    if type(min_reserve) not in (int,float) or not math.isfinite(min_reserve) or min_reserve<0:
        raise ValueError('关节余量要求必须为非负有限数。')
    if (not isinstance(arm,dict)
            or arm.get('frame_names')!={'work':'World','tool':'Arm_Tip'}
            or not isinstance(arm.get('frame_signatures'),dict) or not arm['frame_signatures']):
        raise ValueError(label+'：需要完整World / Arm_Tip参考。')
    if any(type(arm.get(k)) is not int or arm[k]!=0 for k in ('arm_error','system_error')):
        raise ValueError(label+'：臂或系统错误非0，不能作正常参考。')
    _numbers(arm.get('pose'),6,label+'位姿')
    joints=_numbers(arm.get('joints'),6,label+'关节角')
    limits=arm.get('limits')
    if not isinstance(limits,dict):raise ValueError(label+'：缺少关节限位。')
    low=_numbers(limits.get('minimum'),6,label+'下限')
    high=_numbers(limits.get('maximum'),6,label+'上限')
    speeds=_numbers(limits.get('speed'),6,label+'速度限制')
    accelerations=_numbers(limits.get('acceleration'),6,label+'加速度限制')
    if any(a>=b for a,b in zip(low,high)) or any(v<=0 for v in speeds+accelerations):
        raise ValueError(label+'：关节配置不完整或无效。')
    reserve=[min(q-a,b-q) for q,a,b in zip(joints,low,high)]
    failed=[i for i,v in enumerate(reserve) if v<min_reserve]
    if failed:
        i=failed[0]
        raise ValueError('%s：J%d当前%.3f°，普通限位余量%.3f°，要求至少%.3f°。'%(
            label,i+1,joints[i],reserve[i],min_reserve))
    result=copy.deepcopy(arm)
    # The original report is retained by the caller; use recomputed margins.
    result['reserve']=reserve
    return result


def arm_reference(record,label='臂位参考',min_reserve=3):
    """Validate a complete healthy read-only bracket, never a fault last state."""
    if (not isinstance(record,dict) or record.get('reference_qualified') is not True
            or record.get('SDK_cleanup_finished') is not True or record.get('read_only') is not True
            or record.get('exceptions') or record.get('cleanup_errors')):
        raise ValueError(label+'：需要本次健康只读记录，且SDK清理完成。')
    return _arm_data(record.get('reference_arm'),label,min_reserve)


def _shape(record,label):
    if not isinstance(record,dict) or record.get('recorded') is not True:
        raise ValueError(label+'：本阶段尚未确认保存。')
    value=targets(record.get('targets'),label+'目标')
    snapshot=record.get('snapshot')
    if isinstance(snapshot,dict):
        observed=targets(snapshot.get('targets'),label+'保存时目标')
        targets(snapshot.get('positions'),label+'保存时实际位置')
        if any(abs(a-b)>2 for a,b in zip(value,observed)):
            raise ValueError(label+'：保存目标与原始反馈不一致。')
        # A teaching record is historical data, not a later live-protection gate.
        # In particular do not silently relabel recorded3/4 as normal feedback.
    return value


def assemble_success(start_record,grip_shape_record,prepare_shape_record,wrist_record,
                     lift_result,operator_passed):
    """Save only a completed180mm lift plus the operator's actual pickup verdict."""
    if operator_passed is not True:
        raise ValueError('只有现场明确确认夹起且稳定，才能保存成功记录。')
    if not isinstance(start_record,dict) or not isinstance(wrist_record,dict):
        raise ValueError('缺少本轮开始位置或抓前腕姿记录。')
    start=arm_reference(start_record.get('arm'),'开始位置')
    wrist=arm_reference(wrist_record.get('arm'),'抓前腕姿')
    if start['frame_signatures']!=wrist['frame_signatures'] or start['limits']!=wrist['limits']:
        raise ValueError('开始位置与抓前腕姿的坐标系/限位改变。')
    grip=_shape(grip_shape_record,'抓取手势')
    prepare=_shape(prepare_shape_record,'预抓取手势')
    start_hand=_recorded_hand(start_record.get('hand'),'开始手型原始记录')
    wrist_hand=_recorded_hand(wrist_record.get('hand'),'抓前手型原始记录')
    if any(abs(a-b)>2 for a,b in zip(wrist_hand['targets'],prepare)):
        raise ValueError('记录腕姿时的预抓目标与本轮预抓手势不一致。')
    if (not isinstance(lift_result,dict) or lift_result.get('phase')!='completed'
            or lift_result.get('outcome')!=LIFT_COMPLETED_OUTCOME
            or lift_result.get('exceptions') or lift_result.get('cleanup_errors')
            or lift_result.get('SDK_cleanup_finished') is not True):
        raise ValueError('自动提起流程尚未完整完成，不能保存成功。')
    upper=_arm_data(lift_result.get('qualified_upper'),'实际升后点',3)
    if upper['frame_signatures']!=wrist['frame_signatures'] or upper['limits']!=wrist['limits']:
        raise ValueError('提起前后坐标系或关节限位改变。')
    upper_hand=hand_snapshot(lift_result.get('qualified_upper_hand'),'实际升后手部')
    if any(abs(a-b)>2 for a,b in zip(upper_hand['targets'],grip)):
        raise ValueError('实际升后抓取目标与本轮抓取手势不一致。')
    delta=[upper['pose'][i]-wrist['pose'][i] for i in range(3)]
    xy=math.hypot(delta[0],delta[1])
    angle=orientation_angle_rad(wrist['pose'][3:],upper['pose'][3:])
    if delta[2]<.1795:
        raise ValueError('实际World Z只提起%.3fmm，尚未到180mm目标范围。'%(delta[2]*1000))
    if xy>.001 or angle>.005:
        raise ValueError('实际提起端点改变了X/Y或腕姿：XY=%.3fmm，姿态差=%.6frad。'%(xy*1000,angle))
    return dict(schema_version=1,scope='new_start_custom_grip_prepare_wrist_and180mm_pickup_record',
        recording_order=['start','grip_shape','prepare_shape','wrist','automatic_lift180mm'],
        start_record=copy.deepcopy(start_record),grip_shape_record=copy.deepcopy(grip_shape_record),
        prepare_shape_record=copy.deepcopy(prepare_shape_record),wrist_record=copy.deepcopy(wrist_record),
        lift_result=copy.deepcopy(lift_result),start_reference_arm=start,
        start_hand_targets=start_hand['targets'][:],grip_targets=grip,prepare_targets=prepare,
        reference_pregrasp_tcp_pose_m_rad=wrist['pose'][:],wrist_reference_arm=wrist,
        qualified_upper=upper,qualified_upper_hand=copy.deepcopy(lift_result['qualified_upper_hand']),
        lift_command_mm=180,actual_lift_world_mm=[v*1000 for v in delta],
        actual_lift_xy_mm=xy*1000,actual_lift_orientation_rad=angle,
        operator_observed_pickup_success=True,operator_observation_source='onsite explicit y after actual lift',
        object_pickup_inferred_from_arm_motion=False,holding_sensor_verified=False,
        ordinary_status5_is_grasp_failure=False,actual_finger_arrival_required=False,
        continuous_path_fully_sensed=False,visual_binding_recorded=False,
        complete_visual_pick_sort_accepted=False,old_v13_reference_automatically_replaced=False,
        other_target_reachability_accepted=False,motion_authorized=False,execution_ready=False)
