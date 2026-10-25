"""New seated wrist/reference translation only; no device or motion authority."""
import copy
import math

PRE=[35000,7500,9480,30000,65535,65535]
GRIP=[40000,12500,14480,30000,65535,65535]


def vector(value,n,label):
    if (not isinstance(value,(tuple,list)) or len(value)!=n
            or any(type(v) not in (int,float) or not math.isfinite(v) for v in value)):
        raise ValueError(label+' requires finite numbers')
    return list(value)


def make_anchor(arm_record,hand_record,point_record,recipe,provenance):
    if (arm_record.get('reference_qualified') is not True
            or arm_record.get('SDK_cleanup_finished') is not True
            or arm_record.get('exceptions') or arm_record.get('cleanup_errors')):
        raise ValueError('New current healthy seated pregrasp reference required')
    arm=arm_record['reference_arm'];pose=vector(arm['pose'],6,'new pregrasp pose')
    if arm.get('frame_names')!={'work':'World','tool':'Arm_Tip'}:
        raise ValueError('World / Arm_Tip reference required')
    if recipe.get('pregrasp_targets')!=PRE or recipe.get('grasp_targets')!=GRIP:
        raise ValueError('Current explicit operator recipe required')
    if (hand_record.get('snapshot_stale') is not False or hand_record.get('needs_reconnect')
            or hand_record.get('ok') is not True):
        raise ValueError('Current hand targets not recorded')
    hand=hand_record['snapshot']
    if any(abs(a-b)>2 for a,b in zip(vector(hand['targets'],6,'hand targets'),PRE)):
        raise ValueError('New pregrasp target readback differs')
    if any(v not in (0,1,2,5) for v in vector(hand['status'],6,'hand status')):
        raise ValueError('Current hand protection requires onsite handling')
    # Latest operator instruction: actual positions may stop short on contact.
    # They are evidence, not an additional target-arrival gate.
    vector(hand['positions'],6,'actual hand positions')
    summary=point_record['burst_summary']
    if (summary.get('burst_qualified') is not True or summary.get('frame_quality_pass') is not True
            or summary.get('frame_count',0)<8 or summary.get('elapsed_seconds',0)<2
            or summary.get('reject_reasons')
            or summary.get('top_center_repeatability_max_mm',math.inf)>3):
        raise ValueError('Fresh original8-frame/2s/3mm image reference required')
    point=vector(point_record['top_center_world_m'],3,'new reference visual point')
    if point!=summary.get('top_center_world_m'):
        raise ValueError('Reference point differs from saved image burst')
    if point_record.get('old_grasp_tcp_mapping_used') is not False:
        raise ValueError('Old grasp TCP must not participate in new binding')
    return dict(schema_version=2,scope='new_grasp_wrist_equal_height_task_reference',
        reference_pregrasp_tcp_pose_m_rad=pose,
        reference_top_center_world_m=point,
        top_center_to_pregrasp_TCP_world_m=[pose[i]-point[i] for i in range(3)],
        reference_arm=copy.deepcopy(arm),reference_hand=copy.deepcopy(hand),
        reference_burst=copy.deepcopy(summary),equal_height_plane=copy.deepcopy(point_record['equal_height_plane']),
        camera_context=copy.deepcopy(point_record['binding_camera_context']),
        recipe=copy.deepcopy(recipe),provenance=copy.deepcopy(provenance),
        seated_pregrasp_not_lifted_pose_confirmed_by_operator=True,
        same_shell_place_unchanged_between_arm_and_image_confirmed_by_operator=True,
        old_grasp_tcp_mapping_used=False,old_2p8_plus_4p5_offsets_applied=False,
        fault_last_state_used=False,actual_finger_arrival_required=False,
        task_binding_recorded=True,independent_absolute_accuracy_accepted=False,
        six_dimensional_TCP_calibrated=False,handeye_refitted=False,
        protection_ready_for_later_motion=arm_record.get('protection_ready_for_later_motion',False),
        motion_authorized=False,execution_ready=False)


def target_candidate(point_record,anchor):
    if (anchor.get('schema_version')!=2 or anchor.get('task_binding_recorded') is not True
            or anchor.get('scope')!='new_grasp_wrist_equal_height_task_reference'):
        raise ValueError('New wrist task reference required')
    if (not isinstance(point_record,dict)
            or point_record.get('binding_camera_context')!=anchor.get('camera_context')
            or point_record.get('equal_height_plane')!=anchor.get('equal_height_plane')
            or point_record.get('old_grasp_tcp_mapping_used') is not False):
        raise ValueError('Current camera/model/plane provenance differs from new wrist reference')
    summary=point_record.get('burst_summary',{})
    if (summary.get('burst_qualified') is not True or summary.get('frame_quality_pass') is not True
            or summary.get('frame_count',0)<8 or summary.get('elapsed_seconds',0)<2
            or summary.get('reject_reasons')
            or summary.get('top_center_repeatability_max_mm',math.inf)>3):
        raise ValueError('A complete current image burst is required for a target candidate')
    point=vector(point_record.get('top_center_world_m'),3,'new equal-height visual point')
    if point!=summary.get('top_center_world_m'):
        raise ValueError('Current point differs from its saved image burst')
    origin=vector(anchor['reference_top_center_world_m'],3,'reference point')
    original=vector(anchor['reference_pregrasp_tcp_pose_m_rad'],6,'reference pose')
    delta=[point[i]-origin[i] for i in range(3)]
    pose=[original[i]+delta[i] for i in range(3)]+original[3:]
    hover=pose[:];hover[2]+=.140
    return dict(scope='new_wrist_live_equal_height_translation_candidate',
        pregrasp_tcp_candidate_m_rad=pose,hover_tcp_candidate_m_rad=hover,
        delta_from_new_reference_world_mm=[v*1000 for v in delta],
        fixed_wrist_rpy_rad=original[3:],prepare_recipe=PRE[:],grip_recipe=GRIP[:],
        grasp_order=['thumb','index','middle','lift'],
        ring_little_opposition_targets_retained=True,actual_finger_arrival_required=False,
        old_offsets_applied=False,slot_lookup_used=False,grid_snap_used=False,
        motion_authorized=False,execution_ready=False)
