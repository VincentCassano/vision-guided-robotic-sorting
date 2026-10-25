"""Bind a newly taught wrist only after a real empty-hand vertical lift.

Pure saved-record validation. No driver, network, motion or configuration API.
The caller invokes this after the operator's explicit u confirmation; endpoint
records do not independently prove continuous-path clearance or object pickup.
"""
import copy
import math

from battery_grasp.real_grasp import orientation_angle_rad
from new_grasp_task_anchor_v1 import PRE, make_anchor, vector


def _arm(record, label):
    if (not isinstance(record, dict) or record.get('reference_qualified') is not True
            or record.get('SDK_cleanup_finished') is not True
            or record.get('read_only') is not True
            or record.get('exceptions') or record.get('cleanup_errors')):
        raise ValueError(label+'：需要完整健康的只读臂位记录，且SDK已退出。')
    arm=record.get('reference_arm')
    if not isinstance(arm, dict) or arm.get('frame_names')!={'work':'World','tool':'Arm_Tip'}:
        raise ValueError(label+'：需要World / Arm_Tip坐标系。')
    if not isinstance(arm.get('frame_signatures'), dict) or not arm['frame_signatures']:
        raise ValueError(label+'：坐标系参数记录不完整。')
    if any(type(arm.get(k)) is not int or arm[k]!=0 for k in ('arm_error','system_error')):
        raise ValueError(label+'：臂或系统错误非0，不能用作新参考。')
    pose=vector(arm.get('pose'),6,label+'位姿')
    joints=vector(arm.get('joints'),6,label+'关节角')
    limits=arm.get('limits')
    if not isinstance(limits,dict):raise ValueError(label+'：缺少关节限位。')
    low=vector(limits.get('minimum'),6,label+'关节下限')
    high=vector(limits.get('maximum'),6,label+'关节上限')
    if any(a>=b for a,b in zip(low,high)):
        raise ValueError(label+'：关节上下限无效。')
    reserve=[min(q-a,b-q) for q,a,b in zip(joints,low,high)]
    failed=[i+1 for i,v in enumerate(reserve) if v<3.]
    if failed:
        raise ValueError(label+'：J'+','.join(str(i) for i in failed)+'距普通限位不足3°，请重新选腕姿。')
    return arm,pose,reserve


def _hand(record,label):
    if (not isinstance(record,dict) or record.get('ok') is not True
            or record.get('snapshot_stale') is not False or record.get('needs_reconnect')
            or not isinstance(record.get('snapshot'),dict)):
        raise ValueError(label+'：需要本次完整的新手部读数。')
    hand=record['snapshot']
    targets=vector(hand.get('targets'),6,label+'目标')
    if any(abs(a-b)>2 for a,b in zip(targets,PRE)):
        raise ValueError(label+'：请保持指定PRE预抓目标，不要在空手升高期间改手型。')
    if any(v not in (0,1,2,5) for v in vector(hand.get('status'),6,label+'状态')):
        raise ValueError(label+'：存在真实保护或无效手部状态，请现场处理。')
    vector(hand.get('positions'),6,label+'实际位置')
    return hand


def validate_lift_reference(seated_arm_record,seated_hand_record,upper_arm_record,upper_hand_record):
    """Validate endpoints of the operator-confirmed empty-hand teaching lift."""
    start,start_pose,start_reserve=_arm(seated_arm_record,'抓前点')
    end,end_pose,end_reserve=_arm(upper_arm_record,'升高点')
    start_hand=_hand(seated_hand_record,'抓前点')
    end_hand=_hand(upper_hand_record,'升高点')
    if start['frame_signatures']!=end['frame_signatures'] or start['limits']!=end['limits']:
        raise ValueError('上下记录的坐标系或关节限位改变，需要重新记录同一会话。')
    delta=[end_pose[i]-start_pose[i] for i in range(3)]
    xy=math.hypot(delta[0],delta[1])
    angle=orientation_angle_rad(start_pose[3:],end_pose[3:])
    if delta[2]<.140:
        raise ValueError('实际World Z只增加%.3fmm；需要至少140mm，建议示教增加150mm。'%(delta[2]*1000))
    if xy>.001:
        raise ValueError('升高时X/Y合计改变%.3fmm；请保持X/Y，仅沿World Z正向升高。'%(xy*1000))
    if angle>.005:
        raise ValueError('升高时腕姿改变%.6frad；请保持腕姿重新做竖直升高。'%angle)
    return dict(scope='operator_taught_empty_hand_vertical_lift_endpoint_evidence',
        endpoint_checks_passed=True,actual_delta_world_mm=[v*1000 for v in delta],
        actual_z_increase_mm=delta[2]*1000,actual_xy_distance_mm=xy*1000,
        orientation_difference_rad=angle,minimum_z_increase_mm=140.,
        maximum_xy_distance_mm=1.,maximum_orientation_difference_rad=.005,
        required_joint_reserve_degrees=3.,start_joint_reserve_degrees=start_reserve,
        end_joint_reserve_degrees=end_reserve,seated_reference_arm=copy.deepcopy(start),
        upper_reference_arm=copy.deepcopy(end),seated_reference_hand=copy.deepcopy(start_hand),
        upper_reference_hand=copy.deepcopy(end_hand),
        operator_explicitly_confirmed_empty_hand_vertical_teaching=True,
        operator_confirmation_source='explicit u command in parent, not a holding sensor',
        empty_hand_arm_lift_only=True,object_pickup_evidence=False,
        continuous_path_sensed=False,new_whole_cycle_accepted=False,
        other_target_reachability_accepted=False,fault_last_state_used=False,
        motion_authorized=False,execution_ready=False)


def make_lift_anchor(seated_arm_record,seated_hand_record,upper_arm_record,upper_hand_record,
                     point_record,recipe,provenance):
    if not isinstance(provenance,dict):raise ValueError('缺少本次上下参考文件及SHA来源。')
    keys=('arm_reference_file','arm_reference_sha256','upper_arm_reference_file','upper_arm_reference_sha256')
    for key in keys:
        value=provenance.get(key)
        if not isinstance(value,str) or not value:
            raise ValueError('参考来源缺少字段：'+key)
        if key.endswith('sha256') and (len(value)!=64 or any(c not in '0123456789abcdef' for c in value)):
            raise ValueError('参考SHA256格式无效：'+key)
    try:
        anchor=make_anchor(seated_arm_record,seated_hand_record,point_record,recipe,provenance)
    except (KeyError,TypeError,AttributeError,ValueError) as exc:
        raise ValueError('抓前腕姿与同位置视觉尚不能绑定：'+str(exc)) from exc
    lift=validate_lift_reference(seated_arm_record,seated_hand_record,upper_arm_record,upper_hand_record)
    lift['provenance']={key:copy.deepcopy(provenance[key]) for key in keys}
    anchor.update(reference_lift=lift,reference_lift_recorded=True,
        reference_lift_is_empty_hand_only=True,new_wrist_and_lift_endpoint_binding_recorded=True,
        new_whole_cycle_accepted=False,motion_authorized=False,execution_ready=False)
    return anchor
