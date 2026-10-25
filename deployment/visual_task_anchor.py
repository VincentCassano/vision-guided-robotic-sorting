"""Fixed-wrist task mapping from one visual reference and an accepted grip.

This is not a hand-eye fit or a six-dimensional tool calibration. All outputs
remain candidates. No SDK imports, device access or actuator authority exist.
"""
import copy
import hashlib
import json
import math

SOURCE_SHA='6652427ab133c3d48ed334f548595b523874436f16d174d833a596b634848614'
PREGRASP_SHA='d69a9e0c91d4aa34b68ab8d7ae75600d14f147135fec1b7ce7c5d36b9b0e3372'
PREGRASP_CROSSCHECK_SHA='2480000a1d2d0670a05ce3dbbe7272dc59346d28a671a9f713e98eab571893f6'
RECIPE=[26500,30500,29500,22000,65535,65056]


def vector(value,n,label):
    if (not isinstance(value,(list,tuple)) or len(value)!=n
            or any(type(v) not in (int,float) or not math.isfinite(v) for v in value)):
        raise ValueError(label+': finite numeric vector required')
    return list(value)


def accepted_grasp(raw,pregrasp_raw,crosscheck_raw):
    if hashlib.sha256(raw).hexdigest()!=SOURCE_SHA:
        raise ValueError('Original123051 grip evidence bytes differ')
    source=json.loads(raw);receipt=source['new_grip_receipt']
    if (source.get('active_targets')!=RECIPE or receipt.get('recipe')!=RECIPE
            or receipt.get('old_fault_is_not_motion_authority') is not True
            or receipt['user_event'].get('value')!='l'
            or receipt['user_event'].get('opposed_grip_and_loaded_up140_clear') is not True):
        raise ValueError('Accepted pre-lift grip receipt required')
    rows=receipt['groups']
    if len(rows)!=3:raise ValueError('Three accepted original grip groups required')
    for row in rows:
        status=vector(row['status'],6,'original status')
        positions=vector(row['positions'],6,'original position')
        targets=vector(row['targets'],6,'original target')
        if (any(v not in (0,1,2,5) for v in status)
                or any(abs(a-b)>150 for a,b in zip(positions,RECIPE))
                or any(abs(a-b)>2 for a,b in zip(targets,RECIPE))):
            raise ValueError('Original accepted grip feedback differs')
    grip_arm=receipt['full_current_arm'];pose=vector(grip_arm['pose'],6,'accepted grip TCP')
    before=receipt['current_arm']
    if (grip_arm.get('arm_error')!=0 or grip_arm.get('system_error')!=0
            or grip_arm['frame_names']!={'work':'World','tool':'Arm_Tip'}
            or grip_arm['frame_signatures']!=before['frame_signatures'] or grip_arm['limits']!=before['limits']
            or math.dist(pose[:3],before['pose'][:3])>.0005
            or any(abs(a-b)>.08 for a,b in zip(grip_arm['joints'],before['joints']))):
        raise ValueError('Accepted successful pre-lift arm provenance differs')
    if (hashlib.sha256(pregrasp_raw).hexdigest()!=PREGRASP_SHA
            or hashlib.sha256(crosscheck_raw).hexdigest()!=PREGRASP_CROSSCHECK_SHA):
        raise ValueError('Original completed noncontact pregrasp evidence differs')
    pregrasp=json.loads(pregrasp_raw);crosscheck=json.loads(crosscheck_raw)
    segment=pregrasp['arm_segments'][-1];arm=segment['endpoint']
    events={row['index']:row for row in pregrasp['operator_inputs']}
    if (segment.get('phase')!='descending_to_supported_shell'
            or arm!=crosscheck['qualified_reference'] or arm.get('arm_error')!=0 or arm.get('system_error')!=0
            or arm['frame_signatures']!=grip_arm['frame_signatures'] or arm['limits']!=grip_arm['limits']
            or events[1].get('battery_in_grasp_space_no_contact') is not True
            or events[5].get('current_noncontact_pose_and_same_shell_scene_confirmed') is not True
            or events[5].get('freeze_supported_contact_while_shell_supported') is not True):
        raise ValueError('Completed supported-shell noncontact endpoint not corroborated')
    return dict(source_worker_sha256=SOURCE_SHA,pregrasp_source_sha256=PREGRASP_SHA,
        pregrasp_crosscheck_sha256=PREGRASP_CROSSCHECK_SHA,
        source='024412.completed_descending_endpoint_crosschecked081023.qualified_reference',
        arm=copy.deepcopy(arm),recipe=RECIPE[:],grip_feedback=copy.deepcopy(rows),
        later_partially_lifted_grip_arm_for_recipe_only=copy.deepcopy(grip_arm),
        later_lifted_pose_used_as_seated_reference=False,
        fault_last_arm_state_used=False,full140mm_endpoint_used=False,
        actual_lift_confirmation='onsite user reported shell lifted and retained after123051',
        tool_frame_is_grasp_center=False)


def point_from_burst(summary):
    if (summary.get('burst_qualified') is not True or summary.get('frame_quality_pass') is not True
            or summary.get('frame_count',0)<8 or summary.get('elapsed_seconds') is None
            or summary['elapsed_seconds']<2 or summary.get('reject_reasons')
            or summary.get('top_center_repeatability_max_mm',math.inf)>3):
        raise ValueError('Original complete fresh8-frame/2-second/3mm burst required')
    return vector(summary['top_center_world_m'],3,'visual top center')


def context_checked(context):
    if (context.get('camera_serial')!='344322075008' or context.get('image_size')!=[1280,720]
            or context.get('measured_shell')!={'diameter_mm':45.8,'total_height_mm':98.0}
            or context.get('camera_fixed_confirmed') is not True):
        raise ValueError('Same fixed camera and measured shell context required')
    for key in ('candidate_sha256','weights_sha256','shell_model_sha256','source_grasp_sha256',
            'source_pregrasp_sha256','source_pregrasp_crosscheck_sha256'):
        value=context.get(key)
        if not isinstance(value,str) or len(value)!=64:raise ValueError('Missing context hash: '+key)
    if context['source_grasp_sha256']!=SOURCE_SHA:raise ValueError('Grip evidence context differs')
    if (context['source_pregrasp_sha256']!=PREGRASP_SHA
            or context['source_pregrasp_crosscheck_sha256']!=PREGRASP_CROSSCHECK_SHA):
        raise ValueError('Seated pregrasp evidence context differs')
    return copy.deepcopy(context)


def make_anchor(summary,context,grasp,original_place_confirmed):
    if original_place_confirmed is not True:
        raise ValueError('Reference shell must occupy the exact original successful grasp place')
    point=point_from_burst(summary);context=context_checked(context)
    pose=grasp['arm']['pose']
    return dict(schema_version=1,scope='one_reference_fixed_wrist_task_translation',
        camera_context=context,reference_top_center_world_m=point,
        reference_pregrasp_tcp_pose_m_rad=pose[:],
        top_center_to_pregrasp_TCP_world_m=[pose[i]-point[i] for i in range(3)],
        reference_burst=copy.deepcopy(summary),accepted_grasp=copy.deepcopy(grasp),
        original_grasp_place_confirmed=True,
        place_confirmation_source='onsite user restores same shell to original successful full recess',
        task_binding_recorded=True,independent_absolute_accuracy_accepted=False,
        six_dimensional_TCP_calibrated=False,handeye_refitted=False,
        head_or_mount_or_board_movement_permitted=False,
        reference_location_operator_confirmed=True,
        robot_current_pose_assumed=False,clearance_inherited=False,
        motion_authorized=False,execution_ready=False)


def target_candidate(point,anchor,context):
    point=vector(point,3,'new visual top center')
    if anchor.get('schema_version')!=1 or anchor.get('task_binding_recorded') is not True:
        raise ValueError('Task anchor required')
    if context_checked(context)!=anchor['camera_context']:
        raise ValueError('Camera/model/grasp context changed after binding')
    origin=vector(anchor['reference_top_center_world_m'],3,'reference visual top')
    original=vector(anchor['reference_pregrasp_tcp_pose_m_rad'],6,'reference pregrasp TCP')
    delta=[point[i]-origin[i] for i in range(3)]
    pose=[original[i]+delta[i] for i in range(3)]+original[3:]
    hover=pose[:];hover[2]+=.140
    return dict(scope='continuous_visual_displacement_to_fixed_wrist_task_TCP_candidate',
        selected_top_center_world_m=point,delta_from_reference_world_mm=[v*1000 for v in delta],
        pregrasp_tcp_candidate_m_rad=pose,pregrasp_tcp_candidate_mm=[v*1000 for v in pose[:3]],
        fixed_wrist_rpy_rad=original[3:],hover_tcp_candidate_m_rad=hover,
        hover_offset_mm=140,hover_is_not_clearance_certificate=True,
        prepare_recipe=[8000,8000,8000,8000,65535,RECIPE[5]],
        grip_recipe_candidate=RECIPE[:],grip_recipe_at_this_pregrasp_wrist_accepted=False,
        normal_bin_open_recipe=[0,0,0,0,0,RECIPE[5]],
        frame_names={'work':'World','tool':'Arm_Tip'},
        reference_binding_source='restore_original_grasp_place_then_live_reference_burst',
        slot_lookup_used=False,grid_snap_used=False,coordinates_rescaled=False,
        rotation_estimated=False,arbitrary_object_or_orientation_supported=False,
        independent_absolute_accuracy_accepted=False,six_dimensional_TCP_calibrated=False,
        camera_calibration_overwritten=False,current_arm_or_hand_qualified=False,
        inverse_kinematics_or_swept_geometry_verified=False,
        motion_authorized=False,execution_ready=False)


def saved_target(summary,anchor,context):
    result=target_candidate(point_from_burst(summary),anchor,context)
    result['selected_burst']=copy.deepcopy(summary)
    result['fresh_burst_qualified']=True
    return result


def validate_saved_anchor(anchor,context,grasp):
    if (anchor.get('scope')!='one_reference_fixed_wrist_task_translation'
            or anchor.get('schema_version')!=1 or anchor.get('task_binding_recorded') is not True
            or anchor.get('original_grasp_place_confirmed') is not True
            or anchor.get('accepted_grasp')!=grasp
            or anchor.get('reference_pregrasp_tcp_pose_m_rad')!=grasp['arm']['pose']
            or context_checked(context)!=anchor.get('camera_context')
            or anchor.get('motion_authorized') is not False or anchor.get('execution_ready') is not False):
        raise ValueError('Saved binding source/context or authority differs')
    point=point_from_burst(anchor['reference_burst'])
    if (point!=anchor.get('reference_top_center_world_m')
            or anchor.get('top_center_to_pregrasp_TCP_world_m')!=[grasp['arm']['pose'][i]-point[i] for i in range(3)]):
        raise ValueError('Saved binding does not match its original visual/grip evidence')
    return copy.deepcopy(anchor)
