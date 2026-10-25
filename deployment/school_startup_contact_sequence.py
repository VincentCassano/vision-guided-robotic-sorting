"""One fresh same-slot supported trial; no camera or historical point recovery.

The normal l action includes a short lift AND its normal downward leg. A fault
never authorizes that leg, a retry, or opening. Physical shell retention is an
onsite observation, not a conclusion from arm/finger encoder agreement.
"""
import copy
import math

from vision_cycle_supported_release import SupportedCycle
from school_startup_pose_hand import PREPARE
from battery_grasp.vision_cycle_motion import build_segment
from high_descent_prepare_worker import same_pose
from run_school_battery_hand import ADAPTED_PLAN, TARGET_TOL
from school_startup_contact_transport import FIRST_CLOSE_KEY, FIRST_CLOSE, FREEZE_PHASE
from school_startup_hand import SupportedStatus3
from supported_contact_transport import PhaseAuthority
from battery_grasp.real_grasp import orientation_angle_rad
from battery_grasp.pregrasp_replay import check_segment
from battery_grasp.supported_loaded_motion import build_loaded_segment, LIFT_MM


class SupportedLoadedCycle(SupportedCycle):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.hand_role = 'trial'
        self.load_targets = None
        self.load_anchor = None
        self.loaded_protection_rows = []
        self.loaded_monitor = None
        self.startup_observation_only = True
        self.startup_normalized = False
        self.startup_hand_observation = None
        self.lift_blocked = False

    def teach_current_goal(self, initial, tip):
        previous = self.report['last_preflight']['targets'][:]
        event = self.wait_input(0, 'await_prepare', initial, ('h',))
        if event.get('empty_hand_high_clear_confirmed') is not True:
            raise RuntimeError('Empty hand and full PREPARE sweep not confirmed')
        self.report['phase'] = 'preparing_empty_hand'; self.save()
        _, latest, hand = self.preflight(initial, previous, keep_hand=True)
        try:
            if latest != tip: raise RuntimeError('Tip parameters changed before startup PREPARE')
            hand.prepare_empty_start(previous)
        finally:
            self.close_hand()
        self.authority.prepare_feedback_pass = True
        self.plan = None
        event = self.wait_input(1, 'await_goal', initial, ('t',), teaching=True)
        gap = event.get('lowest_hand_to_board_mm')
        if (type(gap) not in (int, float) or not math.isfinite(gap) or gap < 10
                or event.get('noncontact_same_supported_shell') is not True):
            raise RuntimeError('Fresh supported noncontact goal/board gap missing')
        self.report['phase'] = 'reading_current_taught_goal'; self.save()
        goal = self.full_arm()
        if goal['frame_signatures'] != initial['frame_signatures'] or goal['limits'] != initial['limits']:
            raise RuntimeError('Frames/limits changed during explicit teaching')
        self.reference = goal
        goal, latest = self.preflight(goal, PREPARE)
        if latest != tip: raise RuntimeError('Tip parameters changed during teaching')
        self.report.update(taught_goal=copy.deepcopy(goal), board_clearance_mm=gap,
            board_clearance_is_not_battery_clearance=True, battery_side_clearance_mm=None,
            target_source='this_invocation_explicitly_taught_current_TCP',
            historical_clearances_reused=False, diagnostic_pass=True)
        self.save()
        return goal

    def protected_close_for_supported_opening(self, exc):
        """Route typed status3 to support confirmation; grant no write yet."""
        if not isinstance(exc, SupportedStatus3):
            raise exc
        row = copy.deepcopy(exc.feedback)
        status = exc.status
        targets = list(FIRST_CLOSE[1])
        frozen = self.authority.contact_freeze_values
        if frozen is not None:
            if self.authority.last_contact_freeze_key not in self.authority._acknowledged_writes:
                raise exc
            targets = list(self.authority.active_targets)
        if (exc.origin != 'validated_complete_group' or row.get('status') != status
                or FIRST_CLOSE_KEY not in self.authority._acknowledged_writes
                or self.report.get('phase') != 'grasping'
                or self.load_targets != targets
                or not self.authority._terminal_latched
                or row.get('identity') != self.profile['device_identity']
                or len(status) != 6 or 3 not in status or 4 in status
                or any(type(v) is not int or v not in range(6) for v in status)
                or any(status[ch] == 3 for ch in (4, 5))
                or any(len(row.get(name, [])) != 6 for name in ('targets', 'positions', 'current'))
                or any(type(v) is not int or not 0 <= v <= 65535
                    for name in ('targets', 'positions', 'current') for v in row[name])
                or any(abs(a-b) > TARGET_TOL for a,b in zip(row['targets'], targets))
                or any(c > lim for c,lim in zip(row['current'], self.profile['current_limit_ma']))
                or self.report['configuration'] != self.profile['observed_configuration']
                or self.io is None or self.io.broken or (self.folder/'abort.json').exists()):
            raise exc
        self.lift_blocked = True
        self.load_targets = targets; self.load_anchor = copy.deepcopy(row)
        self.report.update(loaded_lift_blocked=True, loaded_lift_blocked_reason='actual_status3',
            loaded_protection_rows=copy.deepcopy(self.loaded_protection_rows),
            original_close_latch_preserved=self.authority._terminal_latched,
            supported_opening_requires_new_confirmation_and_fresh_preflight=True,
            supported_opening_freshly_qualified=False,
            load_targets=self.load_targets, load_anchor=self.load_anchor)
        self.event('loaded_lift_forbidden_status3', snapshot=row,
            independent_supported_opening_only=True, automatic_release=False)
        self.save()
        return row

    def loaded_context(self, expected_arm, role):
        """Fresh identity/model/protection/mode before a new load capability."""
        from return_after_release_worker import metadata_signature, model_signature, SAFETY_CONFIGURATION
        from sdk_environment import collect_sdk_environment
        from taught_model_readonly import collect_model_parameters
        self.plan = None
        self.report['phase'] = 'preflight_supported_'+role; self.save()
        metadata = collect_sdk_environment(self.robot)
        if metadata_signature(metadata) != metadata_signature(self.report['sdk_environment']):
            raise RuntimeError('Actual SDK/real-mode identity changed during supported trial')
        self.report[role+'_sdk_environment'] = metadata
        model = collect_model_parameters(self.folder/('fresh_model_before_'+role))
        self.report['fresh_model_before_'+role] = model; self.save()
        if model_signature(model) != model_signature(self.report['fresh_model']):
            raise RuntimeError('Actual DH/identity changed during supported trial')
        arm = self.full_arm()
        if not same_pose(arm, expected_arm) or arm['limits'] != expected_arm['limits']:
            raise RuntimeError('Arm/frames/dynamics changed before supported trial operation')
        safety = copy.deepcopy(self.report['arm_safety_reads'])
        low, high = self.safety_reads()
        if any(self.report['arm_safety_reads'][k] != safety[k] for k in SAFETY_CONFIGURATION):
            raise RuntimeError('Actual protection/install/drive configuration changed')
        if any(min(q-a,b-q) < 3 for q,a,b in zip(arm['joints'], low, high)):
            raise RuntimeError('Current drive reserve below3degrees')
        self.drive_limits = dict(minimum=low, maximum=high)
        tip = self.tip_reader()
        if tip != self.report['live_tip_parameters']:
            raise RuntimeError('Actual tip parameters changed during supported trial')
        self.guard_target, self.guard_pose = arm['joints'][:], arm['pose'][:]
        self.hand_role = role
        if role == 'opening':
            if self.release_authority is not None:
                raise RuntimeError('Only one independently confirmed supported opening')
            self.release_authority = PhaseAuthority()
            self.release_authority.allow_protected_opening = True
            self.report['supported_opening_capability'] = dict(granted_after_permission_index=6,
                independent_of_close_token=True, original_close_latch_preserved=self.authority._terminal_latched,
                allowed_address=1135, allowed_word_count=4, allowed_values=[8000]*4,
                no_reset_or_protection_settings=True)
            self.save()
        hand = self.hand_factory(self, True)
        try:
            hand.configuration()
            hand.preflight_loaded(self.load_targets, anchor=self.load_anchor,
                allow_existing_protection=role == 'opening')
            if role == 'opening':
                # A fault row selects the withdrawal scope, never substitutes
                # for a fresh complete reading or evidence of a quiet hand.
                fresh_hand = copy.deepcopy(hand.loaded_anchor)
                hand.monitor_loaded(self.load_targets, fresh_hand)
            after = self.full_arm()
            if not same_pose(after, arm) or after['limits'] != arm['limits']:
                raise RuntimeError('Arm drifted during loaded hand/configuration reads')
            if role == 'opening':
                self.report['supported_opening_freshly_qualified'] = True
                self.save()
            return after, tip, hand
        except BaseException:
            self.close_hand(); raise

    def execute_loaded_vertical(self, arm, tip, z, phase, hand):
        """Never invokes the empty-PREPARE motion helper under load."""
        self.plan = None
        self.report['phase'] = 'preflight_'+phase; self.save()
        fresh = self.full_arm()
        if not same_pose(fresh, arm) or fresh['limits'] != arm['limits']:
            raise RuntimeError('Loaded segment start drifted; no MoveL')
        hand._critical_configuration()
        hand.monitor_loaded(self.load_targets, self.load_anchor)
        last_monitor = [self.clock()]

        def record(row):
            if (self.folder/'abort.json').exists(): raise KeyboardInterrupt('Cancelled during runtime calculation')
            self.event(row['event'], **{k:v for k,v in row.items() if k != 'event'})
            if self.clock()-last_monitor[0] >= .5:
                q,p = self.arm_state()
                if not same_pose(dict(fresh, joints=q, pose=p), fresh):
                    raise RuntimeError('Arm changed while loaded candidate was being calculated')
                hand.monitor_loaded(self.load_targets, self.load_anchor)
                last_monitor[0] = self.clock()

        candidate = build_loaded_segment(fresh, self.drive_limits, z,
            tip['line_speed_m_s'], tip['line_acc_m_s2'], record=record)
        hand.monitor_loaded(self.load_targets, self.load_anchor)
        hand._critical_configuration()
        after = self.full_arm()
        if not same_pose(after, fresh) or after['limits'] != fresh['limits']:
            raise RuntimeError('Loaded source changed before MoveL')
        if (self.folder/'abort.json').exists(): raise KeyboardInterrupt('Cancelled before loaded MoveL')
        self.plan = candidate
        self.report.update(plan=candidate, phase=phase, execution_ready=True, arm_command_attempted=True)
        self.report['arm_actuator_writes'] += 1
        self.arm_command_attempted = True; self.loaded_monitor = hand; self.save()
        if phase == 'lifting_loaded_20mm':
            self.report['loaded_lift_command_attempted'] = True
        rc = self.call('Movel_Cmd', candidate['target_sdk_pose_m_rad'], candidate['speed_percent'],
            trajectory_connect=0, r=0, block=False)
        if type(rc) is not int or rc != 0: raise RuntimeError('Loaded MoveL rejected/uncertain; no retry')
        endpoint = self.wait_target()
        self.report.setdefault('arm_segments', []).append(dict(phase=phase, plan=copy.deepcopy(candidate),
            endpoint=endpoint, actual_world_delta_mm=[(b-a)*1000 for a,b in zip(fresh['pose'][:3], endpoint['pose'][:3])],
            shell_retention_sensor_verified=False))
        self.save()
        return endpoint

    def wait_target(self):
        if self.loaded_monitor is None:
            return super().wait_target()
        start = self.clock(); previous = None; quiet_since = None; groups = 0
        while self.clock()-start < self.plan['arm_timeout_seconds']:
            q,p = self.arm_state()
            self.loaded_monitor.monitor_loaded(self.load_targets, self.load_anchor)
            reached = (math.dist([v*1000 for v in p[:3]], self.plan['target_world_mm']) <= .5
                and orientation_angle_rad(p[3:], self.plan['start']['pose'][3:]) <= .005)
            quiet = (previous is not None and max(abs(a-b) for a,b in zip(q, previous[0])) <= .02
                and math.dist(p[:3], previous[1][:3]) <= .0002
                and orientation_angle_rad(p[3:], previous[1][3:]) <= .002)
            if reached and quiet:
                if quiet_since is None: quiet_since = self.clock()
                groups += 1
                if groups >= 3 and self.clock()-quiet_since >= 1:
                    endpoint = self.full_arm(); check_segment(endpoint, self.plan)
                    if math.dist([v*1000 for v in endpoint['pose'][:3]], self.plan['target_world_mm']) > .5:
                        raise RuntimeError('Loaded terminal differs after stable feedback')
                    self.guard_target, self.guard_pose = endpoint['joints'][:], endpoint['pose'][:]
                    return endpoint
            else:
                quiet_since = None; groups = 0
            previous = q,p; self.pause(.1)
        raise TimeoutError('Short loaded MoveL feedback deadline exceeded; no automatic lower/open')

    def run(self):
        resume=getattr(self,'grasp_resume',None)
        initial, tip = self.preflight(None, PREPARE if resume else None)
        self.report['initial_arm'] = initial; self.save()
        if resume:
            self.qualify_grasp_resume(initial,tip,resume)
            end=initial
            return_z=initial['pose'][2]+.100
            self.report.update(empty_return_target_world_z_m=return_z,
                empty_return_target_is_not_an_observed_endpoint=True)
            self.save()
        else:
            end,return_z=self.prepare_empty_corridor(initial,tip)
        self.complete_grasp_cycle(end,tip,return_z)

    def qualify_grasp_resume(self,initial,tip,resume):
        from return_after_release_worker import metadata_signature,model_signature,SAFETY_CONFIGURATION
        source=resume['completed_descent']
        checks=dict(
            completed_descent_pose=same_pose(initial,source),
            all_limits=initial['limits']==source['limits'],
            SDK=metadata_signature(self.report['sdk_environment'])==metadata_signature(resume['sdk_environment']),
            model=model_signature(self.report['fresh_model'])==model_signature(resume['fresh_model']),
            hand_configuration=self.report['configuration']==resume['configuration'],
            tip=tip==resume['live_tip_parameters'],
            protection=all(self.report['arm_safety_reads'][name]==resume['arm_safety_reads'][name]
                for name in SAFETY_CONFIGURATION))
        self.report['grasp_resume_qualification']=dict(checks=checks,
            failed_checks=[name for name,passed in checks.items() if not passed],
            source_sha256=resume['source_sha256'],
            source_endpoint='successful descending_to_supported_shell endpoint',
            rejected_last_arm_state_used=False,automatic_alignment=False,
            joint_deltas_degrees=[a-b for a,b in zip(initial['joints'],source['joints'])],
            xyz_delta_mm=math.dist(initial['pose'][:3],source['pose'][:3])*1000,
            rotation_delta_rad=orientation_angle_rad(initial['pose'][3:],source['pose'][3:]),
            bounds=dict(joints_degrees=.08,xyz_mm=.5,rotation_rad=.005))
        self.save()
        if not all(checks.values()):
            raise RuntimeError('Current state differs from completed descent; no alignment, no actuator write')
        # PREPARE was freshly checked by preflight; no old hand target is sent.
        self.authority.prepare_feedback_pass=True
        self.report.update(previous_result_reused=True,historical_clearances_reused=False,
            resume_grasp_ready=True,arm_start_source='this_session_fresh_read_matched_successful_descent')
        self.save()

    def prepare_empty_corridor(self,initial,tip):
        goal = self.teach_current_goal(initial, tip)
        event = self.wait_input(2, 'await_empty_lift', goal, ('u',))
        if event.get('whole_path_clear_and_actual_stop') is not True:
            raise RuntimeError('Whole empty upward100mm swept path not confirmed')
        self.plan = None
        upper = self.execute_vertical(goal, tip, goal['pose'][2]+.100, 'lifting_empty_100mm')
        self.report['upper'] = upper; self.save()
        event = self.wait_input(4, 'await_descend', upper, ('d',))
        if event.get('same_corridor_scene_and_stop_confirmed') is not True:
            raise RuntimeError('Current downward corridor/stop not confirmed')
        self.plan = None
        end = self.execute_vertical(upper, tip, goal['pose'][2], 'descending_to_supported_shell')
        self.validate_descent_terminal(end, goal, upper, [v*1000 for v in goal['pose'][:3]])
        return end,upper['pose'][2]

    def complete_grasp_cycle(self,end,tip,return_z):
        self.plan = None
        event = self.wait_input(5, 'await_grasp', end, ('g',))
        if (event.get('only_intended_shell_contact_and_closure_clear') is not True
                or event.get('current_noncontact_pose_and_same_shell_scene_confirmed') is not True
                or event.get('freeze_supported_contact_while_shell_supported') is not True):
            raise RuntimeError('Supported shell contact and one target-freeze permission required')
        self.authority.loaded_grasp_permission = copy.deepcopy(event)
        self.report['phase'] = 'preflight_grasp'; self.save()
        _, latest, hand = self.preflight(end, PREPARE, keep_hand=True)
        try:
            if latest != tip: raise RuntimeError('Tip changed before supported close')
            self.report.update(phase='grasping', grasp_target_steps_planned=1,
                grip_holding_verified=False); self.save()
            try:
                last = hand.close_trial()
            except SupportedStatus3 as exc:
                last = self.protected_close_for_supported_opening(exc)
            self.report['grasp_feedback'] = copy.deepcopy(last); self.save()
        finally:
            self.close_hand()
        supported_end = end
        if not self.lift_blocked:
            event = self.wait_input(8, 'await_loaded_trial', end, ('l', 's'))
            if event['value'] == 'l':
                if (event.get('supervised_same_slot_20mm_lift_and_return_clear') is not True
                        or event.get('thumb_and_opposed_index_or_middle_contact_observed') is not True
                        or event.get('no_uncontrolled_motion_slip_or_overheating_observed') is not True):
                    raise RuntimeError('Explicit loaded lift/lower swept path permission missing')
                arm, latest, hand = self.loaded_context(end, 'moving')
                try:
                    if latest != tip: raise RuntimeError('Tip changed before loaded lift')
                    lifted = self.execute_loaded_vertical(arm, tip, arm['pose'][2]+LIFT_MM/1000,
                        'lifting_loaded_20mm', hand)
                    self.report.update(loaded_arm_up_feedback_observed=True,
                        loaded_lift_performed=True, load_lift_or_transfer_performed=True,
                        object_lift=False, grip_holding_verified=False); self.save()
                    # A normal lower is part of the single l action. If any
                    # prior line raises, execution never reaches this command.
                    supported_end = self.execute_loaded_vertical(lifted, tip, end['pose'][2],
                        'lowering_loaded_to_original_slot', hand)
                    if (math.dist(supported_end['pose'][:3], end['pose'][:3]) > .0005
                            or orientation_angle_rad(supported_end['pose'][3:], end['pose'][3:]) > .005):
                        raise RuntimeError('Loaded return differs from original supported terminal')
                    self.report['loaded_arm_return_feedback_observed'] = True; self.save()
                finally:
                    self.loaded_monitor = None; self.close_hand()
        self.plan = None
        event = self.wait_input(6, 'await_supported_release', supported_end,
            ('n',) if self.lift_blocked else ('r', 'n'))
        if event.get('shell_fully_supported_and_prepare_release_clear') is not True:
            raise RuntimeError('Whole shell support/opening sweep not confirmed')
        observed = event.get('shell_actually_lifted_clear_and_lowered') is True
        if observed and not self.report.get('loaded_arm_return_feedback_observed'):
            raise RuntimeError('Cannot report a loaded lift that this invocation did not perform')
        self.report.update(object_lift=observed, grip_observed_by_operator=observed,
            grip_observation_source='onsite_operator_not_retention_sensor',
            loaded_lift_observed_by_operator=observed, grip_holding_verified=False)
        self.save()
        _, latest, hand = self.loaded_context(supported_end, 'opening')
        try:
            if latest != tip: raise RuntimeError('Tip changed before supported release')
            self.io.set_phase('release_loaded_position', self.release_authority)
            self.io.enable_motion(); self.hand_motion_enabled = True
            self.report['phase'] = 'releasing_supported_shell'; self.save()
            self.report['release_feedback'] = hand.release_prepare()
            self.release_authority._completed = True
            self.report.update(supported_release_completed=True, empty_return_completed=False,
                release_feedback_scope='four_flex_channels_observed_at8000_not_torque_release')
            self.save()
        finally:
            self.close_hand()
        # A retained protection does not permit an empty MoveL. Preserve the
        # completed withdrawal result instead of obscuring it with a new fault.
        if any(v in (3, 4) for v in self.report['release_feedback']['status']):
            self.report.update(outcome='completed_supported_opening_empty_return_blocked', phase='completed',
                execution_ready=False, empty_return_blocked_reason='retained_actual_hand_protection',
                loaded_lift_observed_by_operator=observed, automatic_open=False, automatic_return=False)
            self.save()
            return
        event = self.wait_input(7, 'await_empty_return', supported_end, ('u',))
        if event.get('shell_on_board_hand_empty_and_corridor_clear') is not True:
            raise RuntimeError('Empty hand/board support/upward corridor not confirmed')
        self.hand_role = 'trial'; self.plan = None
        final = self.execute_vertical(supported_end, tip, return_z, 'returning_empty_100mm')
        self.plan = None
        final, latest = self.preflight(final, PREPARE)
        if latest != tip: raise RuntimeError('Actual tip parameters changed at completion')
        self.report.update(outcome=('completed_supported_low_lift_return_release' if observed
                else 'completed_supported_release_lift_not_performed'), phase='completed',
            final_arm=final, final_hand=copy.deepcopy(self.report['last_preflight']),
            execution_ready=False, physical_stop_verified=False, torque_release_verified=False,
            empty_return_completed=True,
            loaded_transfer_performed=False, lateral_transfer_ready=False,
            autonomous_other_targets_ready=False,
            functional_validation='onsite_feedback_and_operator_shell_lift_observation_recorded')
        self.authority._completed = True; self.save()

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

