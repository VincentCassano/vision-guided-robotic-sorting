"""Measured adjacent-groove World-X following, never a calibration certificate."""
import copy
import hashlib
import math
from battery_grasp.localized_pregrasp import select_surface
from battery_grasp.real_grasp import number,orientation_angle_rad,vector
from return_after_release_worker import (
    strict_json,arm_shape,metadata_signature,model_signature,prepare_readback,SAFETY_CONFIGURATION)
from high_descent_prepare_worker import same_pose
from vision_cycle_sequence import CALIBRATION_SHA
from vision_cycle_hand import PREPARE
from vision_follow_sequence import FollowSequence
from battery_grasp.vision_numeric import depth_spread_check

SUCCESS='completed_adjacent_groove_visual_follow_grasp_release_empty_return'


def checked_bytes(folder,name,record):
    raw=(folder/name).read_bytes()
    if len(raw)!=record['byte_count'] or hashlib.sha256(raw).hexdigest()!=record['sha256']:
        raise ValueError('Captured follow-source bytes changed: '+name)
    return raw


def read_follow_source(folder,profile):
    current_parent=strict_json((folder/'parent_result.json').read_bytes())
    records=current_parent['follow_source']
    worker=strict_json(checked_bytes(folder,'follow_source/worker_result.json',records['worker_report']))
    parent=strict_json(checked_bytes(folder,'follow_source/parent_result.json',records['parent_report']))
    scope=worker.get('scope')
    if scope=='same_saved_corridor_shell_visual_descent_supported_grasp_release_empty_return':
        expected_binding_scope='fresh_visual_same_original_unmoved_shell_same_completed_corridor_only'
        scene_permissions=((3,'same_original_shell_mounts_and_scene_unchanged'),
                           (4,'same_original_shell_mounts_and_scene_unchanged'),
                           (7,'shell_on_board_hand_empty_and_corridor_clear'))
    elif scope=='same_qualified_upper_new_wrist_visual_grasp_cycle':
        expected_binding_scope='fresh_visual_same_qualified_upper_same_unmoved_shell_only'
        scene_permissions=tuple((index,'same_high_return_shell_scene_and_clearance_confirmed')
                                for index in (3,4,7))
        if (parent.get('scope')!=scope
                or worker.get('high_return_source_accepted') is not True
                or worker.get('hand_above_target_top_confirmed') is not True
                or worker.get('return_height_reproduced_from_qualified_upper') is not True
                or worker.get('lateral_transfer_ready') is not False):
            raise ValueError('Qualified-upper reference lacks its observed height and scene qualification')
    else:
        raise ValueError('Unsupported completed visual follow-reference scope')
    if (type(worker.get('schema_version')) is not int or worker['schema_version']!=1
            or worker.get('outcome')!='completed_visual_descent_grasp_release_empty_return'
            or parent.get('outcome')!=worker['outcome'] or parent.get('worker_result')!=worker
            or parent.get('worker_return_code')!=0 or type(parent.get('worker_return_code')) is not int
            or parent.get('exceptions')!=[] or parent.get('worker_cleanup_timeout') is not False
            or worker.get('exceptions')!=[] or worker.get('cleanup_errors')!=[]
            or worker.get('grip_observed_by_operator') is not True
            or worker.get('opening_retreat_confirmed') is not True
            or worker.get('explicit_supported_opening_requested') is not True):
        raise ValueError('Follow reference requires the complete observed successful supported visual cycle')
    for key,expected in (('arm_actuator_writes',2),('hand_register_writes',2),
            ('controller_configuration_writes',0),('hand_bus_configuration_writes',0)):
        if type(worker.get(key)) is not int or worker[key]!=expected:
            raise ValueError('Unexpected follow reference write count: '+key)
    if (worker.get('arm_stop_attempt') is not None or worker['configuration']!=profile['observed_configuration']
            or worker['final_hand']['identity']!=profile['device_identity']):
        raise ValueError('Follow reference fault stop or measured hand configuration differs')
    prepare_readback(worker['final_hand'])
    segments=worker.get('arm_segments',[])
    if len(segments)!=2 or [row.get('phase') for row in segments]!=['descending','returning_empty']:
        raise ValueError('Reference must contain its unique completed descent and empty return')
    for arm in (worker['upper'],worker['final_arm'],segments[0]['endpoint'],segments[1]['endpoint']):
        arm_shape(arm)
        if arm['limits']!=worker['upper']['limits'] or arm['frame_signatures']!=worker['upper']['frame_signatures']:
            raise ValueError('Reference arm frame/configuration changed inside completed cycle')
    if not same_pose(worker['final_arm'],segments[1]['endpoint']):
        raise ValueError('Reference final arm differs from observed empty return')
    metadata_signature(worker['sdk_environment']);model_signature(worker['fresh_model'])
    for index,flag in scene_permissions+((5,'only_intended_shell_contact_and_closure_clear'),
                                        (6,'shell_fully_supported_and_prepare_release_clear')):
        rows=[row for row in worker['operator_inputs'] if row.get('index')==index]
        if len(rows)!=1 or rows[0].get(flag) is not True:
            raise ValueError('Completed reference physical confirmation missing: '+str(index))
    for name in ('detections.json','original.png','detections.png','aligned_depth_u16.png'):
        checked_bytes(folder,'reference_vision/'+name,records['reference_vision']['files'][name])
    raw=(folder/'reference_vision/detections.json').read_bytes()
    if (hashlib.sha256(raw).hexdigest()!=parent['vision_source']['detections_sha256']
            or parent['vision_source']['captured_in_this_invocation'] is not True
            or records['reference_vision']['detections_sha256']!=parent['vision_source']['detections_sha256']):
        raise ValueError('Reference fresh RGB-D provenance differs')
    document=strict_json(raw)
    binding=worker['visual_binding'];target=select_surface(document,binding['target']['index'])
    if (target!=binding['target'] or target!=parent['target'] or target['class_name']!='black_cyl_battery'
            or target['world_transform_sha256']!=CALIBRATION_SHA
            or binding['scope']!=expected_binding_scope):
        raise ValueError('Accepted reference visual binding incomplete')
    point=vector(target['surface_world_mm'],'reference point')
    offset=vector(binding['surface_to_taught_TCP_mm'],'reference frozen offset')
    mapped=vector(binding['mapped_TCP_mm'],'reference mapped TCP')
    if math.dist([a+b for a,b in zip(point,offset)],mapped)>1e-6:
        raise ValueError('Reference frozen surface-to-TCP offset inconsistent')
    if (not 30<=(worker['upper']['pose'][2]*1000-mapped[2])<=120
            or math.dist(mapped[:2],[v*1000 for v in worker['upper']['pose'][:2]])>.5
            or orientation_angle_rad(binding['tool_rpy_rad'],worker['upper']['pose'][3:])>.005
            or number(worker['board_clearance_mm'],'reference board gap')<10):
        raise ValueError('Reference binding outside its accepted vertical corridor')
    return dict(worker=worker,parent=parent,source_records=records,reference_document=document)


class FollowCycle(FollowSequence):
    def __init__(self,*args,follow_context,raise_for_vision_40mm=False,fixture_indexed_116mm=False,**kwargs):
        super().__init__(*args,**kwargs)
        self.follow_context=follow_context
        if type(raise_for_vision_40mm) is not bool:
            raise ValueError('Camera clearance mode must be an explicit boolean')
        self.raise_for_vision_40mm=raise_for_vision_40mm
        if type(fixture_indexed_116mm) is not bool:
            raise ValueError('Measured fixture mode must be an explicit boolean')
        self.fixture_indexed_116mm=fixture_indexed_116mm
        self._qualified_follow_wrist=None

    def check_frozen_wrist(self,pose):
        if self._qualified_follow_wrist is None:return
        initial_angle=orientation_angle_rad(pose[3:],self._qualified_follow_wrist)
        baseline_angle=orientation_angle_rad(pose[3:],self.follow_context['worker']['upper']['pose'][3:])
        self.report['last_frozen_follow_wrist_check']=dict(
            physical_angle_to_this_start_rad=initial_angle,physical_angle_to_reference_rad=baseline_angle,
            limit_rad=.005,tolerances_not_added_between_segments=True)
        if max(initial_angle,baseline_angle)>.005:
            raise RuntimeError('Physical follow wrist left fixed reference0.005rad; no next action')

    def arm_state(self):
        joints,pose=super().arm_state()
        self.check_frozen_wrist(pose)
        return joints,pose

    def full_arm(self):
        current=super().full_arm()
        self.check_frozen_wrist(current['pose'])
        return current

    def qualify_reference_start(self,initial,tip):
        old=self.follow_context['worker']
        if (metadata_signature(self.report['sdk_environment'])!=metadata_signature(old['sdk_environment'])
                or model_signature(self.report['fresh_model'])!=model_signature(old['fresh_model'])
                or self.report['configuration']!=old['configuration'] or tip!=old['live_tip_parameters']):
            raise RuntimeError('Fresh SDK/DH/hand profile/tip differs from successful visual reference')
        if any(self.report['arm_safety_reads'][name]!=old['arm_safety_reads'][name] for name in SAFETY_CONFIGURATION):
            raise RuntimeError('Fresh installation/drive/protection differs from successful reference')
        upper=old['upper']
        old_position_error=math.dist(initial['pose'][:3],upper['pose'][:3])*1000
        raised_z=upper['pose'][2]+.040
        raised_xy_error=math.dist(initial['pose'][:2],upper['pose'][:2])*1000
        raised_z_error=abs(initial['pose'][2]-raised_z)*1000
        at_original=old_position_error<=.5
        at_raised=(self.raise_for_vision_40mm and raised_xy_error<=.5 and raised_z_error<=.5)
        if (initial['limits']!=upper['limits'] or initial['frame_signatures']!=upper['frame_signatures']
                or not (at_original or at_raised)
                or orientation_angle_rad(initial['pose'][3:],upper['pose'][3:])>.005):
            raise RuntimeError('Current arm outside original upper or explicit40mm-raised region; no old-point alignment')
        prepare_readback(self.report['last_preflight'])
        self.authority.prepare_feedback_pass=True
        self._qualified_follow_wrist=initial['pose'][3:]
        self.report.update(source_records=self.follow_context['source_records'],
            follow_source_context_accepted=True,reference_upper=upper,current_upper=initial,
            historical_pose_not_used_as_current=True,global_camera_extrinsics_certified=False,
            camera_clearance_mode_requested=self.raise_for_vision_40mm,
            follow_start_region=dict(original_upper_error_mm=old_position_error,
                raised_upper_XY_error_mm=raised_xy_error,raised_upper_Z_error_mm=raised_z_error,
                accepted_region='already_40mm_raised' if at_raised else 'original_upper',
                raised_restart_only_at_original_reference_XY=True,no_automatic_alignment=True),
            baseline_visual_binding=copy.deepcopy(old['visual_binding']))
        self.save()
        return bool(at_raised)

    def camera_clearance_upper(self,initial,tip,already_raised):
        if not self.raise_for_vision_40mm:return initial
        event=self.wait_input(8,'await_camera_clearance_lift',initial,('h',))
        if event.get('camera_clearance_lift_confirmed') is not True:
            raise RuntimeError('Empty PREPARE and complete upward swept volume not confirmed')
        raised_z=self.follow_context['worker']['upper']['pose'][2]+.040
        self.report['camera_clearance']=dict(requested_World_Z_change_mm=40,
            absolute_reference_high_Z_m=raised_z,start=copy.deepcopy(initial),
            scene_clearance_confirmed_by_operator=True,head_camera_extrinsics_changed=False,
            already_at_high_region=already_raised,extra_upward_MoveL_planned=not already_raised,
            old_grasp_goal_Z_retained=True,vertical_limit_after_lift_mm=160)
        self.save()
        if already_raised:
            # Restart after a completed lift and a capture refusal is allowed
            # only at that absolute high region, never another slot or +40 again.
            self.report['phase']='preflight_camera_clearance_lift';self.save()
            high,latest=self.preflight(initial,PREPARE)
            if latest!=tip:raise RuntimeError('Tip parameters changed before raised camera capture')
        else:
            target=initial['pose'][:];target[2]=raised_z
            self.plan=None
            high=self.execute_follow(target,'raising_camera_clearance',arm=initial,tip=tip)
        if (math.dist(high['pose'][:2],initial['pose'][:2])*1000>.5
                or abs(high['pose'][2]-raised_z)*1000>.5):
            raise RuntimeError('Raised camera endpoint outside accepted40mm region; no image or descent')
        self.report['camera_clearance'].update(endpoint=copy.deepcopy(high),
            actual_delta_world_mm=[(b-a)*1000 for a,b in zip(initial['pose'][:3],high['pose'][:3])],
            endpoint_feedback_accepted=True,view_unoccluded_verified=False,
            camera_capture_must_confirm_target_visible=True)
        self.report['current_upper']=high
        self.save()
        return high

    def qualify_visual_shift(self,initial):
        old=self.follow_context['worker'];reference=self.follow_context['reference_document']
        event=self.wait_input(3,'await_vision',initial,('b',))
        expected_shift=event.get('measured_world_X_shift_mm')
        if (event.get('adjacent_groove_same_height_and_pose_confirmed') is not True
                or type(expected_shift) is not int or expected_shift not in (-116,116)):
            raise RuntimeError('Complete adjacent groove, unchanged support height and independently observed direction required')
        document=strict_json((self.folder/'vision/detections.json').read_bytes())
        target=select_surface(document,event['target']['index'])
        if target!=event['target'] or target['class_name']!='black_cyl_battery':
            raise RuntimeError('Fresh visual selection changed')
        for key in ('serial','width','height','intrinsics','depth_scale_m','coordinate_frame',
                'world_transform_sha256','world_from_camera'):
            if document.get(key)!=reference.get(key):
                raise RuntimeError('Camera/frame calibration metadata changed: '+key)
        if target['world_transform_sha256']!=CALIBRATION_SHA:
            raise RuntimeError('Frozen calibration payload changed')
        old_target=old['visual_binding']['target'];before=vector(old_target['surface_world_mm'],'old visible point')
        after=vector(target['surface_world_mm'],'new visible point')
        delta=[b-a for a,b in zip(before,after)]
        row=document['detections'][target['index']];old_row=reference['detections'][old_target['index']]
        confidence=number(row['confidence'],'fresh confidence')
        valid_fraction=number(row['depth_valid_fraction'],'depth fraction')
        spread=depth_spread_check(number(row['depth_spread_m'],'depth spread'))
        stable=number(row['stable_seconds'],'stable window')
        time_difference=abs(number(document['color_timestamp_ms'],'RGB time')
            -number(document['depth_timestamp_ms'],'depth time'))
        quality=dict(confidence_0_8_min=confidence>=.8,
            valid_fraction_0_8_min=valid_fraction>=.8,depth_spread_5mm_inclusive=spread['pass'],
            stable_seconds_1_min=stable>=1,RGB_depth_time_difference_50ms_max=time_difference<=50)
        failures=[name for name,passed in quality.items() if not passed]
        self.report['visual_quality_check']=dict({'pass':not failures},gates=quality,failures=failures,
            confidence=confidence,depth_valid_fraction=valid_fraction,depth_spread_check=spread,
            stable_seconds=stable,RGB_depth_time_difference_ms=time_difference)
        self.save()
        if failures:
            raise RuntimeError('Fresh visual quality failed: '+', '.join(failures)+
                '; exact values retained in visual_quality_check; no alignment, descent or grasp')
        if self.fixture_indexed_116mm:
            from battery_grasp.fixture_visual_binding import qualify_fixture_observation
            association=qualify_fixture_observation(reference,document,old_target['index'],
                target['index'],expected_shift)
            self.report['fixture_visual_association']=association
            self.save()
            if association.get('pass') is not True:
                raise RuntimeError('Measured fixture visual association failed: '+
                    ', '.join(association['failures'])+'; no alignment, descent or grasp')
        else:
            for index in (2,3):
                ratio=number(row['bbox'][index],'bbox size')/number(old_row['bbox'][index],'reference bbox size')
                if not .8<=ratio<=1.2:
                    raise RuntimeError('Same-shell apparent shape changed beyond local trial bounds')
            # The older raw-surface mode retains its strict bounds. Fixture
            # mode instead commands the measured slot displacement, never a
            # relaxed version of this raw visible-surface target.
            if abs(delta[0]-expected_shift)>2 or abs(delta[1])>1.5 or abs(delta[2])>3:
                self.report['rejected_visual_shift']=dict(vision_delta_world_mm=delta,
                    expected_measured_WorldX_mm=expected_shift,
                    X_bounds_mm=[expected_shift-2,expected_shift+2],Y_abs_limit_mm=1.5,Z_abs_limit_mm=3,
                    global_calibration_diagnostic_pass=document.get('calibration_diagnostic_pass'),
                    alignment_descent_and_hand_writes_authorized=False,
                    prior_camera_clearance_arm_writes=self.report['arm_actuator_writes'])
                self.save()
                raise RuntimeError('Vision shift does not match confirmed116mm groove; no alignment, descent or grasp')
        baseline=vector(old['visual_binding']['mapped_TCP_mm'],'reference TCP')
        frozen_offset=vector(old['visual_binding']['surface_to_taught_TCP_mm'],'frozen offset')
        goal=[baseline[0]+expected_shift if self.fixture_indexed_116mm else after[0]+frozen_offset[0],
            baseline[1],baseline[2]]
        align=initial['pose'][:];align[0]=goal[0]/1000
        # The Y and Z observations are gates, not additional movement targets.
        # Existing proven board height and lateral Y are deliberately retained.
        align[1]=baseline[1]/1000
        self.report.update(diagnostic_pass=True,
            diagnostic_scope=('vision_corroborated_measured_fixture_WorldX116mm_only'
                if self.fixture_indexed_116mm else 'measured_adjacent_groove_WorldX116mm_only'),
            camera_global_extrinsic_accuracy_certified=False,global_calibration_diagnostic_pass=document.get('calibration_diagnostic_pass'),
            follow_binding=dict(reference_target=old_target,target=target,
                frozen_surface_to_TCP_mm=frozen_offset,offset_refitted_this_run=False,
                vision_delta_world_mm=delta,expected_ruler_shift_mm=[expected_shift,0,0],
                observed_direction_entered_by_operator=True,complete_adjacent_groove=True,
                local_axis_diagnostic_pass=True,global_calibration_certified=False,
                measured_fixture_target_mode=self.fixture_indexed_116mm,
                target_X_source=('original_accepted_TCP_plus_measured_signed116mm_fixture_spacing'
                    if self.fixture_indexed_116mm else 'fresh_visible_surface_X_plus_frozen_offset'),
                raw_visible_surface_used_as_grasp_center=False,
                current_raw_surface_X_used_for_target=not self.fixture_indexed_116mm,
                target_TCP_world_mm=goal,target_upper_world_mm=[v*1000 for v in align[:3]],
                camera_clearance_mode=self.raise_for_vision_40mm,
                descent_distance_mm=initial['pose'][2]*1000-goal[2],
                empty_return_to_camera_capture_height=True,
                ignored_YZ_observation_delta_mm=delta[1:],same_supported_plane_Z_retained=True,
                scope='same_shell_one_measured_adjacent_groove_WorldX116mm_trial'),
            local_visual_follow_execution_ready=False,autonomous_other_targets_ready=False)
        self.report['visual_binding']=copy.deepcopy(self.report['follow_binding'])
        self.save()
        return goal,align

    def run(self):
        initial,tip=self.preflight(None,PREPARE)
        self.report['initial_arm']=initial;self.save()
        already_raised=self.qualify_reference_start(initial,tip)
        upper=self.camera_clearance_upper(initial,tip,already_raised)
        goal,align_target=self.qualify_visual_shift(upper)
        event=self.wait_input(4,'await_descend',upper,('d',))
        if (event.get('adjacent_groove_same_height_and_pose_confirmed') is not True
                or type(event.get('measured_world_X_shift_mm')) is not int
                or event['measured_world_X_shift_mm']!=self.report['follow_binding']['expected_ruler_shift_mm'][0]
                or event.get('same_corridor_scene_and_stop_confirmed') is not True):
            raise RuntimeError('New empty-hand horizontal/descending swept volume not confirmed')
        self.report['local_visual_follow_execution_ready']=True;self.save()
        self.plan=None
        aligned=self.execute_follow(align_target,'aligning_visual_XY',arm=upper,tip=tip)
        self.report['aligned_upper']=aligned;self.save()
        if math.dist([v*1000 for v in aligned['pose'][:2]],goal[:2])>1:
            raise RuntimeError('Observed visual alignment outside1mm; no descent')
        self.plan=None
        if self.raise_for_vision_40mm:
            descent=aligned['pose'][:];descent[2]=goal[2]/1000
            end=self.execute_follow(descent,'descending',arm=aligned,tip=tip,vertical_limit_mm=160)
        else:
            end=self.execute_vertical(aligned,tip,goal[2]/1000,'descending')
        if (math.dist([v*1000 for v in end['pose'][:3]],goal)>1
                or orientation_angle_rad(end['pose'][3:],aligned['pose'][3:])>.005):
            raise RuntimeError('Observed vision-follow terminal differs; no grasp')
        event=self.wait_input(5,'await_grasp',end,('g',))
        if event.get('only_intended_shell_contact_and_closure_clear') is not True:
            raise RuntimeError('Only intended inert-shell contact not confirmed')
        self.report['phase']='preflight_grasp';self.save()
        _,_,hand=self.preflight(end,PREPARE,keep_hand=True)
        try:
            self.io.set_phase('close',self.authority);self.io.enable_motion();self.hand_motion_enabled=True
            self.report.update(phase='grasping',grip_holding_verified=False,grasp_target_steps_planned=2);self.save()
            targets,last=self.close_hand_candidate(hand)
            self.report['grasp_feedback']=last;self.save()
        finally:self.close_hand()
        event=self.wait_input(6,'await_supported_release',end,('r','n'))
        if event.get('shell_fully_supported_and_prepare_release_clear') is not True:
            raise RuntimeError('Fixed support/opening sweep not confirmed')
        self.report.update(grip_observed_by_operator=event['value']=='r',grip_observation_source='onsite_operator_not_sensor');self.save()
        self.report['phase']='preflight_release';self.save()
        hand=self.current_loaded_hand(end,targets)
        try:
            self.report['phase']='releasing';self.save()
            self.report['release_feedback']=self.release_hand_candidate(hand,targets)
        finally:self.close_hand()
        event=self.wait_input(7,'await_empty_return',end,('u',))
        if event.get('shell_on_board_hand_empty_and_corridor_clear') is not True:
            raise RuntimeError('Empty-hand return not confirmed')
        self.plan=None
        if self.raise_for_vision_40mm:
            returning=end['pose'][:];returning[2]=aligned['pose'][2]
            final=self.execute_follow(returning,'returning_empty',arm=end,tip=tip,vertical_limit_mm=160)
        else:
            final=self.execute_vertical(end,tip,aligned['pose'][2],'returning_empty')
        self.report['phase']='final_cycle_preflight';self.save()
        final,final_tip=self.preflight(final,PREPARE)
        if final_tip!=tip:raise RuntimeError('Actual tip parameters changed at follow-cycle end')
        self.report.update(outcome=SUCCESS,phase='completed',final_arm=final,
            final_hand=copy.deepcopy(self.report['last_preflight']),hand_shape='PREPARE',
            supervised_cycle_feedback_completed=True,vision_follow_trial_feedback_completed=True,
            physical_stop_verified=False,torque_release_verified=False,execution_ready=False,
            autonomous_other_targets_ready=False,camera_global_extrinsic_accuracy_certified=False,
            load_lift_or_transfer_performed=False,local_visual_follow_execution_ready=False,
            functional_validation='onsite_single_measured_axis_follow_feedback_pending_grip_observation')
        self.authority._completed=True;self.save()
