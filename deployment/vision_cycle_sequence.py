"""First stationary empty-shell visual binding + taught-Z descent and supported grip."""
import copy
import hashlib
import json
import math
from pathlib import Path
from pregrasp_replay_sequence import RehearsalSequence
from high_descent_prepare_worker import same_pose
from run_school_battery_hand import ADAPTED_PLAN
from battery_grasp.high_descent_trial import PREPARE
from battery_grasp.localized_pregrasp import select_surface
from battery_grasp.vision_cycle_motion import build_segment
from battery_grasp.real_grasp import orientation_angle_rad

CALIBRATION_SHA='70809f5b27269949f42977fbec9923c8d384f728d8eef1653317edeb320bc764'


class VisualCycle(RehearsalSequence):
    def __init__(self,*args,resume_report=None,taught_only=False,completed_corridor=None,**kwargs):
        super().__init__(*args,**kwargs)
        self.resume_report=resume_report
        self.taught_only=taught_only
        self.completed_corridor=completed_corridor
        if taught_only and resume_report:raise ValueError('Taught-only starts fresh; no previous-result reuse')
        if completed_corridor is not None and (taught_only or resume_report is not None):
            raise ValueError('Completed corridor reuse is separate from fresh teaching and numerical resume')

    def reuse_completed_corridor(self,initial,tip):
        from return_after_release_worker import (
            SAFETY_CONFIGURATION,arm_shape,metadata_signature,model_signature,prepare_readback)
        context=self.completed_corridor
        if context is None or self.report.get('source_context_accepted') is not True:
            raise RuntimeError('Completed corridor source bytes have not been strictly accepted')
        prior=context['grasp'];old_upper=context['upper'];goal=prior['taught_goal']
        for arm in (initial,goal):arm_shape(arm)
        if (initial['frame_signatures']!=old_upper['frame_signatures']
                or initial['limits']!=old_upper['limits']
                or goal['frame_signatures']!=initial['frame_signatures']
                or goal['limits']!=initial['limits']):
            raise RuntimeError('Fresh frames/limits/dynamics differ from completed corridor')
        # Compare the current Cartesian start, not historical joint values.
        # The new runtime branch calculation starts from the freshly read q.
        upper_error_mm=math.dist(initial['pose'][:3],old_upper['pose'][:3])*1000
        upper_rotation_rad=orientation_angle_rad(initial['pose'][3:],old_upper['pose'][3:])
        if upper_error_mm>.5 or upper_rotation_rad>.005:
            raise RuntimeError('Fresh upper is outside the original0.5mm/0.005rad Cartesian corridor; no automatic alignment')
        if (metadata_signature(self.report['sdk_environment'])!=metadata_signature(prior['sdk_environment'])
                or model_signature(self.report['fresh_model'])!=model_signature(prior['fresh_model'])
                or self.report['configuration']!=context['release']['configuration']
                or tip!=context['expected_tip']):
            raise RuntimeError('Fresh SDK/DH/hand profile/tip differs from completed corridor')
        for name in SAFETY_CONFIGURATION:
            if self.report['arm_safety_reads'][name]!=context['release']['arm_safety_reads'][name]:
                raise RuntimeError('Fresh protection/installation/drive configuration differs: '+name)
        prepare_readback(self.report['last_preflight'])
        if self.report['last_preflight']['identity']!=self.profile['device_identity']:
            raise RuntimeError('Fresh hand identity differs from accepted completed corridor')
        board=prior.get('board_clearance_mm')
        if type(board) not in (int,float) or not math.isfinite(board) or board<10:
            raise RuntimeError('Original noncontact endpoint board measurement missing')
        for index,flag in ((1,'noncontact_same_supported_shell'),
                           (2,'onsite_world_Z_corridor_rehearsed_noncontact'),
                           (4,'same_corridor_scene_and_stop_confirmed')):
            rows=[row for row in prior.get('operator_inputs',[]) if row.get('index')==index]
            if len(rows)!=1 or rows[0].get(flag) is not True:
                raise RuntimeError('Original noncontact teaching/observed corridor confirmations incomplete')
        height=(initial['pose'][2]-goal['pose'][2])*1000
        if (not 30<=height<=120 or math.dist(goal['pose'][:2],old_upper['pose'][:2])*1000>.5
                or orientation_angle_rad(goal['pose'][3:],old_upper['pose'][3:])>.005):
            raise RuntimeError('Original goal is outside the completed same-XY/wrist30..120mm corridor')
        mapped=[initial['pose'][0]*1000,initial['pose'][1]*1000,goal['pose'][2]*1000]
        self.report.update(taught_goal=copy.deepcopy(goal),upper=initial,initial_arm=initial,
            board_clearance_mm=board,historical_clearances_reused=True,
            board_clearance_source='original_noncontact_endpoint_pending_same_scene_reconfirmation',
            battery_side_clearance_mm=None,board_clearance_is_not_battery_clearance=True,
            manual_corridor_height_mm=height,previous_vision_reused=False,
            completed_corridor_reuse=dict(source_records=context['source_records'],
                original_outcome=prior['outcome'],completed_descending_segment_observed=True,
                supported_release_observed=True,fresh_upper_xyz_error_mm=upper_error_mm,
                fresh_upper_physical_rotation_rad=upper_rotation_rad,
                historical_joint_values_compared_to_current=False,
                fresh_joint_branch_calculation_required=True,no_automatic_alignment=True,
                original_same_shell_same_position_only=True,scene_reconfirmation_required=True,
                original_taught_goal_is_historical_measurement=True,
                target_uses_fresh_upper_XY_and_wrist_and_original_goal_Z=True),
            re_teaching_or_manual_upper_required=False)
        self.authority.prepare_feedback_pass=True;self.save()
        event=self.wait_input(3,'await_vision',initial,('b',))
        if (event.get('same_taught_stationary_shell_confirmed') is not True
                or event.get('same_original_shell_mounts_and_scene_unchanged') is not True):
            raise RuntimeError('Original shell/location/mounts/scene not reconfirmed for fresh visual binding')
        document=json.loads((self.folder/'vision/detections.json').read_bytes())
        target=select_surface(document,event['target']['index'])
        if (target!=event['target'] or target['class_name']!='black_cyl_battery'
                or target['world_transform_sha256']!=CALIBRATION_SHA):
            raise RuntimeError('Fresh visual target differs from registered fixed-camera black-shell evidence')
        point=target['surface_world_mm'];offset=[a-b for a,b in zip(mapped,point)]
        binding=dict(target=target,surface_to_taught_TCP_mm=offset,
            original_taught_TCP_mm=[v*1000 for v in goal['pose'][:3]],
            mapped_TCP_mm=mapped,tool_rpy_rad=initial['pose'][3:],
            scope='fresh_visual_same_original_unmoved_shell_same_completed_corridor_only',
            point_is_not_grasp_center=True,automatic_other_slot_transfer=False,
            automatic_lateral_motion=False,visual_captured_this_invocation=True,
            target_XY_source='this_session_fresh_upper_readback',
            target_Z_source='original_noncontact_taught_goal',
            mounting_and_scene_unchanged_confirmed_by_operator=True)
        self.report['visual_binding']=binding
        self.report['completed_corridor_reuse'].update(fresh_visual_binding_completed=True,
            same_scene_reconfirmed_by_operator=True)
        self.save()
        (self.folder/'visual_binding.json').write_text(json.dumps(binding,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
        return goal,initial,tip,mapped

    def reuse_context(self,initial,tip):
        path=Path(self.resume_report);raw=path.read_bytes();prior=json.loads(raw)
        if (prior.get('schema_version')!=1 or prior.get('outcome')!='aborted_no_further_writes'
                or prior.get('scope')!='same_taught_shell_visual_descent_supported_grasp_release_empty_return'
                or any(type(prior.get(k)) is not int or prior[k]!=0
                    for k in ('arm_actuator_writes','hand_register_writes','controller_configuration_writes'))
                or prior.get('cleanup_errors') or prior.get('arm_command_attempted') is True):
            raise RuntimeError('Resume only from a clean pre-motion refusal with zero arm/hand/config writes')
        errors=prior.get('exceptions',[])
        if len(errors)!=1 or errors[0].get('error')!="ValueError('Unsafe local branch change')":
            raise RuntimeError('This resume is only for v1 numerical branch refusal; no fault recovery')
        goal,old_upper=prior['taught_goal'],prior['upper']
        if not same_pose(initial,old_upper) or initial['limits']!=old_upper['limits']:
            raise RuntimeError('Current arm moved after the saved upper point; no old-point alignment. Restart without --resume-report to teach a new current corridor')
        if goal['frame_signatures']!=initial['frame_signatures'] or goal['limits']!=initial['limits']:
            raise RuntimeError('Saved endpoint frames/limits differ from this fresh start')
        height=(initial['pose'][2]-goal['pose'][2])*1000
        if (not 30<=height<=120 or math.dist(initial['pose'][:2],goal['pose'][:2])*1000>.5
                or orientation_angle_rad(initial['pose'][3:],goal['pose'][3:])>.005):
            raise RuntimeError('Saved same-XY/wrist corridor no longer matches')
        board=prior.get('board_clearance_mm')
        if type(board) not in (int,float) or not math.isfinite(board) or board<10:
            raise RuntimeError('Saved endpoint board measurement missing')
        events=prior.get('operator_inputs',[])
        for index,flag in ((1,'noncontact_same_supported_shell'),(2,'onsite_world_Z_corridor_rehearsed_noncontact'),
                           (3,'same_taught_stationary_shell_confirmed')):
            matching=[e for e in events if e.get('index')==index]
            if len(matching)!=1 or matching[0].get(flag) is not True:
                raise RuntimeError('Saved physical teaching/visual confirmations incomplete')
        document=json.loads((self.folder/'vision/detections.json').read_bytes())
        target=select_surface(document,prior['visual_binding']['target']['index'])
        if (target!=prior['visual_binding']['target'] or target['class_name']!='black_cyl_battery'
                or target['world_transform_sha256']!=CALIBRATION_SHA):
            raise RuntimeError('Saved visual target differs from registered same-shell evidence')
        fresh_hand=self.report['last_preflight']
        if (any(abs(a-b)>2 for a,b in zip(fresh_hand['targets'],PREPARE))
                or any(abs(a-b)>150 for a,b in zip(fresh_hand['positions'],PREPARE))):
            raise RuntimeError('Resume requires current empty PREPARE; no automatic restore')
        point=target['surface_world_mm'];goal_mm=[v*1000 for v in goal['pose'][:3]]
        offset=[a-b for a,b in zip(goal_mm,point)]
        if any(abs(a-b)>1e-6 for a,b in zip(offset,prior['visual_binding']['surface_to_taught_TCP_mm'])):
            raise RuntimeError('Saved visible-surface registration changed')
        # Return to this session's fresh reference, never move to an old upper.
        self.report.update(taught_goal=goal,upper=initial,board_clearance_mm=board,
            historical_clearances_reused=True,
            board_clearance_source='prior_measurement_same_pose_pending_scene_reconfirmation',
            battery_side_clearance_mm=None,board_clearance_is_not_battery_clearance=True,
            manual_corridor_height_mm=height,visual_binding=copy.deepcopy(prior['visual_binding']),
            resume=dict(source=str(path),source_sha256=hashlib.sha256(raw).hexdigest(),
                old_outcome=prior['outcome'],fresh_upper_matches=True,
                old_vision_reused=True,vision_captured_this_invocation=False,
                no_automatic_alignment=True,scene_reconfirmation_required=True))
        self.save()
        (self.folder/'visual_binding.json').write_text(json.dumps(self.report['visual_binding'],ensure_ascii=False,indent=2)+'\n',encoding='utf8')
        self.authority.prepare_feedback_pass=True
        return goal,initial,tip,goal_mm

    def wait_input(self,index,phase,arm,values,teaching=False):
        self.report.update(phase=phase,permission_index=index);self.save()
        began=self.clock();path=self.folder/'permissions'/('%d.json'%index)
        while self.clock()-began<600:
            if (self.folder/'abort.json').exists():raise KeyboardInterrupt('Cancelled by parent')
            if path.exists():
                row=json.loads(path.read_text(encoding='utf8'))
                if row.get('index')!=index or row.get('phase')!=phase:raise RuntimeError('Stale cycle permission')
                self.report.setdefault('operator_inputs',[]).append(row);self.save()
                if row.get('value')=='q':raise EOFError('Operator quit; no automatic move/open')
                if row.get('value') not in values:raise RuntimeError('Invalid cycle permission')
                return row
            if not teaching:
                q,p=self.arm_state()
                if not same_pose(dict(arm,joints=q,pose=p),arm):raise RuntimeError('Unexpected arm movement during cycle wait')
            self.pause(.15)
        raise TimeoutError('Cycle permission expired')

    def current_loaded_hand(self,arm,expected):
        from sdk_environment import collect_sdk_environment
        from pregrasp_measurement_worker import NATIVE_SHA
        metadata=collect_sdk_environment(self.robot)
        if (not metadata['metadata_complete'] or metadata.get('api_version')!='4.3.1'
                or metadata.get('loaded_native',{}).get('sha256')!=NATIVE_SHA
                or metadata.get('controller_run_mode_raw',{}).get('data')!=[1]):
            raise RuntimeError('Loaded release native identity/real-mode changed')
        self.report['release_sdk_environment']=metadata;self.save()
        fresh=self.full_arm()
        if not same_pose(fresh,arm):raise RuntimeError('Arm changed before hand operation')
        self.guard_target,self.guard_pose=fresh['joints'][:],fresh['pose'][:]
        # Re-read required protections; do not silently change them under load.
        if self.call('Get_Collision_Stage')!=[0,8] or self.call('Get_Self_Collision_Enable')!=[0,True]:
            raise RuntimeError('Protection changed during grasp; no next write')
        hand=self.hand_factory(self,True)
        try:
            hand.configuration();hand._critical_configuration();hand.check_targets(hand.snapshot(),expected)
            return hand
        except BaseException:self.close_hand();raise

    def execute_vertical(self,arm,tip,z,phase):
        self.report['phase']='preflight_'+phase;self.save()
        fresh,latest=self.preflight(arm,PREPARE)
        if latest!=tip:raise RuntimeError('Motion parameters changed after corridor teaching')
        def record(row):
            if (self.folder/'abort.json').exists():raise KeyboardInterrupt('Cancelled during runtime calculation')
            entry=dict(row);event=entry.pop('event')
            self.event(event,**entry)
            self.report['last_runtime_ik_candidate']=row
            if row.get('decision')!='accepted' or row['attempt']%8==0:self.save()
        self.plan=build_segment(fresh,self.drive_limits,z,latest['line_speed_m_s'],latest['line_acc_m_s2'],record=record)
        if (self.folder/'abort.json').exists():raise KeyboardInterrupt('Cancelled before movement')
        after=self.full_arm()
        self.report['arm_after_runtime_calculation']=after;self.save()
        if not same_pose(after,fresh) or after['limits']!=fresh['limits']:
            raise RuntimeError('Arm/frames/dynamics changed during runtime calculation; no MoveL')
        self.report.update(plan=self.plan,phase=phase,execution_ready=True,arm_command_attempted=True)
        self.report['arm_actuator_writes']+=1;self.arm_command_attempted=True;self.save()
        rc=self.call('Movel_Cmd',self.plan['target_sdk_pose_m_rad'],self.plan['speed_percent'],
            trajectory_connect=0,r=0,block=False)
        if type(rc) is not int or rc!=0:raise RuntimeError('MoveL rejected/uncertain; no retry')
        endpoint=self.wait_target()
        self.report.setdefault('arm_segments',[]).append(dict(phase=phase,plan=copy.deepcopy(self.plan),endpoint=endpoint))
        self.save();return endpoint

    def teach_context(self,initial,tip):
        previous=self.report['last_preflight']['targets'][:]
        if any(abs(a-b)>2 for a,b in zip(previous,PREPARE)):
            event=self.wait_input(0,'await_prepare',initial,('h',))
            if event.get('empty_hand_high_clear_confirmed') is not True:raise RuntimeError('Empty-hand PREPARE clearance not confirmed')
            self.report['phase']='preparing_empty_hand';self.save()
            _,_,hand=self.preflight(initial,previous,keep_hand=True)
            try:
                self.io.set_phase('prepare_empty',self.authority);self.io.enable_motion();self.hand_motion_enabled=True
                hand.write_pose(ADAPTED_PLAN[0],previous);hand.preflight(expected=PREPARE)
            finally:self.close_hand()
        self.authority.prepare_feedback_pass=True
        event=self.wait_input(1,'await_goal',initial,('t',),teaching=True)
        board=event.get('lowest_hand_to_board_mm')
        if (type(board) not in (int,float) or not math.isfinite(board) or board<10
                or event.get('noncontact_same_supported_shell') is not True):
            raise RuntimeError('Board clearance/noncontact supported endpoint not confirmed')
        self.report['phase']='preflight_goal';self.save()
        goal=self.full_arm()
        if goal['frame_signatures']!=initial['frame_signatures'] or goal['limits']!=initial['limits']:
            raise RuntimeError('Frames/configuration changed during teaching')
        self.reference=goal
        goal,tip=self.preflight(goal,PREPARE)
        self.report.update(taught_goal=goal,board_clearance_mm=board,battery_side_clearance_mm=None,
            board_clearance_is_not_battery_clearance=True);self.save()
        event=self.wait_input(2,'await_manual_upper',goal,('v',),teaching=True)
        if event.get('onsite_world_Z_corridor_rehearsed_noncontact') is not True:
            raise RuntimeError('Actual noncontact teaching corridor not confirmed')
        self.report['phase']='preflight_upper';self.save()
        upper=self.full_arm()
        if upper['frame_signatures']!=goal['frame_signatures'] or upper['limits']!=goal['limits']:
            raise RuntimeError('Frames/limits changed')
        height=(upper['pose'][2]-goal['pose'][2])*1000
        if (not 30<=height<=120 or math.dist(upper['pose'][:2],goal['pose'][:2])*1000>.5
                or orientation_angle_rad(upper['pose'][3:],goal['pose'][3:])>.005):
            raise RuntimeError('Upper must be World-Z30..120mm above goal, same XY/wrist; use Z jog, not freehand dragging')
        upper,tip=self.preflight(upper,PREPARE)
        self.report.update(upper=upper,manual_corridor_height_mm=height);self.save()
        if self.taught_only:
            self.report.update(vision_used=False,target_source='this_session_taught_TCP',
                camera_extrinsics_used_for_motion=False,visual_binding_created=False,
                historical_target_or_corridor_reused=False)
            self.save()
            return goal,upper,tip,[v*1000 for v in goal['pose'][:3]]
        event=self.wait_input(3,'await_vision',upper,('b',))
        raw=(self.folder/'vision/detections.json').read_bytes();document=json.loads(raw)
        target=select_surface(document,event['target']['index'])
        if target!=event['target'] or target['world_transform_sha256']!=CALIBRATION_SHA:
            raise RuntimeError('Vision payload/calibration mismatch')
        if target['class_name']!='black_cyl_battery':raise RuntimeError('This first-cycle hand profile is bound to black cylindrical test shell')
        if event.get('same_taught_stationary_shell_confirmed') is not True:raise RuntimeError('Vision object does not match taught shell')
        point=target['surface_world_mm'];goal_mm=[v*1000 for v in goal['pose'][:3]]
        offset=[a-b for a,b in zip(goal_mm,point)]
        mapped=[a+b for a,b in zip(point,offset)]
        binding=dict(target=target,surface_to_taught_TCP_mm=offset,taught_TCP_mm=goal_mm,
            mapped_TCP_mm=mapped,tool_rpy_rad=goal['pose'][3:],scope='same_current_unmoved_shell_and_scene_only',
            point_is_not_grasp_center=True,automatic_other_slot_transfer=False)
        self.report['visual_binding']=binding;self.save()
        (self.folder/'visual_binding.json').write_text(json.dumps(binding,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
        return goal,upper,tip,mapped

    def run(self):
        initial,tip=self.preflight(self.reference,None)
        if self.report.get('hand_bus_setup'):
            self.report['hand_bus_setup'].update(identity_pending=False,
                identity=self.report['last_preflight']['identity'],
                outcome='ACK_followed_by_fresh_identity_full_configuration_and_hand_preflight')
            self.save()
        if self.completed_corridor is not None:
            goal,upper,tip,mapped=self.reuse_completed_corridor(initial,tip)
        elif self.resume_report:
            goal,upper,tip,mapped=self.reuse_context(initial,tip)
        else:
            goal,upper,tip,mapped=self.teach_context(initial,tip)
        event=self.wait_input(4,'await_descend',upper,('d',))
        if event.get('same_corridor_scene_and_stop_confirmed') is not True:raise RuntimeError('Corridor no longer qualified')
        if self.completed_corridor is not None:
            if event.get('same_original_shell_mounts_and_scene_unchanged') is not True:
                raise RuntimeError('Original shell/location/mounts/scene changed before descent')
            self.report['completed_corridor_reuse']['same_scene_reconfirmed_before_descent']=True
            self.report['board_clearance_source']='original_noncontact_endpoint_same_scene_reconfirmed_by_operator'
            self.save()
        if self.resume_report:
            self.report['resume']['same_scene_reconfirmed_by_operator']=True
            self.report['board_clearance_source']='prior_measurement_same_pose_and_scene_reconfirmed_by_operator'
            self.save()
        self.enable_trial_protections(upper,copy.deepcopy(self.report['arm_safety_reads']))
        self.plan=None
        end=self.execute_vertical(upper,tip,mapped[2]/1000,'descending')
        self.validate_descent_terminal(end,goal,upper,mapped)
        self.finish_from_pregrasp(end,upper,tip)

    def validate_descent_terminal(self,end,goal,upper,mapped):
        terminal_xyz=[v/1000 for v in mapped] if self.completed_corridor is not None else goal['pose'][:3]
        terminal_wrist=upper['pose'][3:] if self.completed_corridor is not None else goal['pose'][3:]
        if (math.dist(end['pose'][:3],terminal_xyz)>.001
                or orientation_angle_rad(end['pose'][3:],terminal_wrist)>.005):
            raise RuntimeError('Observed terminal pose differs from taught target; no grasp')

    def finish_from_pregrasp(self,end,upper,tip):
        """Continue an accepted actual terminal without repeating vision or descent."""
        event=self.wait_input(5,'await_grasp',end,('g',))
        if event.get('only_intended_shell_contact_and_closure_clear') is not True:
            raise RuntimeError('Grasp target/board sweep not confirmed')
        self.report['phase']='preflight_grasp';self.save()
        _,_,hand=self.preflight(end,PREPARE,keep_hand=True)
        try:
            self.io.set_phase('close',self.authority);self.io.enable_motion();self.hand_motion_enabled=True
            self.report.update(phase='grasping',grip_holding_verified=False)
            self.report['grasp_target_steps_planned']=2;self.save()
            targets,last=self.close_hand_candidate(hand)
            self.report['grasp_feedback']=last;self.save()
        finally:self.close_hand()
        event=self.wait_input(6,'await_supported_release',end,('r','n'))
        if event.get('shell_fully_supported_and_prepare_release_clear') is not True:
            raise RuntimeError('Supported release not confirmed')
        self.report.update(grip_observed_by_operator=event['value']=='r',grip_observation_source='onsite_operator_not_sensor');self.save()
        self.report['phase']='preflight_release';self.save()
        hand=self.current_loaded_hand(end,targets)
        try:
            self.report['phase']='releasing';self.save()
            self.report['release_feedback']=self.release_hand_candidate(hand,targets)
        finally:self.close_hand()
        event=self.wait_input(7,'await_empty_return',end,('u',))
        if event.get('shell_on_board_hand_empty_and_corridor_clear') is not True:
            raise RuntimeError('Empty return not confirmed; no lift under load')
        self.plan=None
        final=self.execute_vertical(end,tip,upper['pose'][2],'returning_empty')
        self.report['phase']='final_cycle_preflight';self.save()
        final,final_tip=self.preflight(final,PREPARE)
        if final_tip!=tip:raise RuntimeError('Actual tip parameters changed at cycle end')
        self.report['final_hand']=copy.deepcopy(self.report['last_preflight'])
        final=self.verify_return_clearance(final)
        outcome=('completed_taught_descent_grasp_release_empty_return' if self.taught_only
            else 'completed_visual_descent_grasp_release_empty_return')
        self.report.update(outcome=outcome,phase='completed',
            final_arm=final,hand_shape='PREPARE',physical_stop_verified=False,torque_release_verified=False,
            autonomous_other_targets_ready=False,load_lift_or_transfer_performed=False,
            supervised_cycle_feedback_completed=True,execution_ready=False,
            functional_validation='onsite_cycle_feedback_and_operator_observation_recorded')
        self.authority._completed=True
        self.save()

    def verify_return_clearance(self,final):
        # Endpoint feedback alone does not establish hand height or lateral sweep clearance.
        self.report.update(hand_above_target_top_confirmed=False,lateral_transfer_ready=False,
            return_endpoint_is_not_lateral_clearance=True)
        self.save()
        return final

    def close_hand_candidate(self,hand):
        return hand.close_candidate(PREPARE[:])

    def release_hand_candidate(self,hand,targets):
        self.io.set_phase('release_to_prepare',self.authority)
        self.io.enable_motion();self.hand_motion_enabled=True
        return hand.release_prepare(targets)
