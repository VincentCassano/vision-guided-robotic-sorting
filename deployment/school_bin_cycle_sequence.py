"""Supervised inert-shell grasp and school-coordinate bin delivery.

This is an adapted route, not certification of the original school's geometry.
No IK/AST/mock project checks run here. Controller MoveJ-P planning is used as
in the actual battery demo; fresh live limits, protections and feedback remain.
"""
import copy
import json
import math

from supported_contact_v6_sequence import SupportedLoadedCycle as ContactCycle
from run_arm_hand_sequence import Sequence
from high_descent_prepare_worker import same_pose
from battery_grasp.real_grasp import orientation_angle_rad
from vision_cycle_hand import PREPARE
from school_bin_drop_hand import DROP_TARGET

SUCCESS = 'completed_school_bin_drop_feedback_and_operator_empty_confirmation'


class SchoolBinCycle(ContactCycle):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.bin_motion = None
        self.bin_route = None

    def arm_state(self):
        if self.bin_motion is None:
            return super().arm_state()
        # A separate route scope; never run the old fixed-wrist descent guard
        # against a deliberately changing school transit/drop orientation.
        if (self.folder/'abort.json').exists(): raise KeyboardInterrupt('Parent abort')
        q,p = Sequence.arm_state(self)
        if any(min(v-a,b-v) < 3 for v,a,b in zip(q,
                self.drive_limits['minimum'], self.drive_limits['maximum'])):
            raise RuntimeError('Bin motion actual drive reserve below3degrees')
        route = self.bin_motion
        if route['method'] == 'Movel_Cmd':
            start, target = route['start']['pose'], route['target']
            if (math.dist(p[:2],start[:2]) > .001
                    or orientation_angle_rad(p[3:],start[3:]) > .005
                    or not min(start[2],target[2])-.001 <= p[2] <= max(start[2],target[2])+.001):
                raise RuntimeError('Loaded lift left this run fixed-wrist vertical corridor')
        return q,p

    def prepare_bin_route(self, end):
        from pathlib import Path
        path = Path(__file__).resolve().parent/'school_battery_bins.json'
        route = json.loads(path.read_text(encoding='utf8'))
        selected = self.report['selected_color']
        if (route.get('robot_ip') != '169.254.128.18'
                or route.get('frame_names') != end['frame_names']
                or route.get('unit') != 'metres_radians'
                or selected not in route['bins']):
            raise RuntimeError('School battery coordinate identity/frame differs')
        for row in list(route['transit']) + [route['bins'][selected]]:
            p = row['pose']
            if len(p) != 6 or any(type(v) not in (int,float) or not math.isfinite(v) for v in p):
                raise RuntimeError('Malformed school battery pose')
        # Endpoints are absolute school poses. No fitted offsets, visual
        # correction, automatic startup alignment or old fault-state anchor.
        self.bin_route = route
        self.report.update(school_bin_route=copy.deepcopy(route),
            school_bin_target=copy.deepcopy(route['bins'][selected]),
            school_route_adapted=True, original_scan_poses_omitted=True,
            actual_continuous_swept_geometry_verified=False,
            automatic_lateral_motion=False, lateral_transfer_ready=False,
            normal_transfer_requires_fresh_loaded_clearance_permission=True)
        self.save()

    def move_bin_segment(self, arm, target, method, phase, hand, speed=10):
        """One nonblocking command, no blend, no retry; live error/limit guards."""
        self.plan = None
        fresh = self.full_arm()
        if not same_pose(fresh,arm) or fresh['limits'] != arm['limits']:
            raise RuntimeError('School segment start changed; no automatic alignment')
        if self.call('Get_Collision_Stage') != [0,8] or self.call('Get_Self_Collision_Enable') != [0,True]:
            raise RuntimeError('School segment collision protections changed')
        hand._critical_configuration()
        hand.monitor_loaded(self.load_targets, self.load_anchor)
        after_hand = self.full_arm()
        if not same_pose(after_hand,fresh) or after_hand['limits'] != fresh['limits']:
            raise RuntimeError('Arm changed during school segment preflight; no command')
        self.bin_motion = dict(method=method, target=target[:], start=copy.deepcopy(fresh))
        self.report.update(phase=phase, plan=None, execution_ready=True,
            current_school_motion=copy.deepcopy(self.bin_motion),
            controller_planned_joint_path=(method == 'Movej_P_Cmd'),
            straight_cartesian_path_claimed=False)
        self.arm_command_attempted = True
        self.report['arm_command_attempted'] = True
        self.report['arm_actuator_writes'] += 1
        self.save()
        began = self.clock()
        rc = self.call(method, target, speed, trajectory_connect=0, r=0, block=False)
        if type(rc) is not int or rc != 0:
            raise RuntimeError('School motion rejected/uncertain; no perturbation or retry')
        previous = None
        quiet_since = None
        groups = 0
        last_hand = began
        endpoint = None
        while self.clock()-began < 100:
            q,p = self.arm_state()
            if self.clock()-last_hand >= .5:
                hand.monitor_loaded(self.load_targets, self.load_anchor)
                last_hand = self.clock()
            reached = (math.dist(p[:3],target[:3]) <= .0005
                and orientation_angle_rad(p[3:],target[3:]) <= .005)
            quiet = (previous is not None
                and max(abs(a-b) for a,b in zip(q,previous[0])) <= .02
                and math.dist(p[:3],previous[1][:3]) <= .0002
                and orientation_angle_rad(p[3:],previous[1][3:]) <= .002)
            if reached and quiet:
                if quiet_since is None: quiet_since = self.clock()
                groups += 1
                if groups >= 3 and self.clock()-quiet_since >= 1:
                    endpoint = self.full_arm()
                    if (endpoint['frame_signatures'] != fresh['frame_signatures']
                            or endpoint['limits'] != fresh['limits']
                            or math.dist(endpoint['pose'][:3],target[:3]) > .0005
                            or orientation_angle_rad(endpoint['pose'][3:],target[3:]) > .005
                            or any(min(v-a,b-v)<3 for v,a,b in zip(endpoint['joints'],
                                self.drive_limits['minimum'],self.drive_limits['maximum']))):
                        raise RuntimeError('School terminal changed after feedback settling')
                    hand.monitor_loaded(self.load_targets, self.load_anchor)
                    break
            else:
                quiet_since = None
                groups = 0
            previous = q,p
            self.pause(.1)
        if endpoint is None: raise TimeoutError('School bin motion feedback timeout')
        self.report.setdefault('arm_segments', []).append(dict(phase=phase, method=method,
            speed_percent=speed, target_sdk_pose_m_rad=target[:], endpoint=copy.deepcopy(endpoint),
            elapsed_seconds=self.clock()-began, shell_retention_sensor_verified=False,
            continuous_swept_geometry_verified=False))
        self.guard_target,self.guard_pose = endpoint['joints'][:],endpoint['pose'][:]
        self.bin_motion = None
        self.report['execution_ready'] = False
        self.save()
        return endpoint

    def complete_grasp_cycle(self, end, tip, return_z):
        self.plan = None
        self.prepare_bin_route(end)
        event = self.wait_input(5,'await_grasp',end,('g',))
        if (event.get('only_intended_shell_contact_and_closure_clear') is not True
                or event.get('freeze_supported_contact_while_shell_supported') is not True
                or event.get('current_noncontact_pose_and_same_shell_scene_confirmed') is not True):
            raise RuntimeError('Fresh supported contact permission missing')
        self.authority.loaded_grasp_permission = copy.deepcopy(event)
        _,latest,hand = self.preflight(end,PREPARE,keep_hand=True)
        try:
            if latest != tip: raise RuntimeError('Tip changed before school-bin close')
            # The readonly pacing qualification is performed while empty, as
            # in v6. State3/4 is a fault here, never promoted to transportable.
            self.report['phase'] = 'grasping'
            self.save()
            self.report['grasp_feedback'] = copy.deepcopy(hand.close_trial())
            self.save()
        finally:
            self.close_hand()
        event = self.wait_input(8,'await_bin_lift',end,('l',))
        if (event.get('opposed_grip_and_loaded_up140_clear') is not True
                or event.get('no_uncontrolled_motion_slip_or_overheating_observed') is not True):
            raise RuntimeError('Observed grip/loaded lift clearance not confirmed')
        self.plan = None
        arm = self.full_arm()
        if not same_pose(arm,end) or arm['limits'] != end['limits']:
            raise RuntimeError('Arm changed after observed grip; no lift')
        self.guard_target,self.guard_pose = arm['joints'][:],arm['pose'][:]
        self.hand_role = 'moving'
        hand = self.hand_factory(self,True)
        try:
            # Full profile/DH/SDK already matched in this exclusive run before
            # close. Retain that provenance; re-read live critical mode and a
            # complete hand group, not all slow metadata again while holding.
            hand._configuration_checked = True
            hand.preflight_loaded(self.load_targets,self.load_anchor)
            self.report['loaded_profile_source'] = 'same_exclusive_run_before_close_full_profile_plus_fresh_critical_and_complete_group'
            upper_target = arm['pose'][:]
            upper_target[2] += .140
            upper = self.move_bin_segment(arm,upper_target,'Movel_Cmd',
                'lifting_loaded_140mm',hand)
            self.report.update(loaded_lift_performed=True,load_lift_or_transfer_performed=True,
                loaded_arm_up_feedback_observed=True, object_lift=False)
            self.save()
            event = self.wait_input(11,'await_bin_transfer',upper,('t',))
            clearance = event.get('loaded_lowest_point_above_obstacle_plane_mm')
            if (type(clearance) not in (int,float) or not math.isfinite(clearance) or clearance < 20
                    or event.get('selected_bin_normal_drop_authorized') is not True
                    or event.get('whole_adapted_school_route_clear') is not True
                    or event.get('shell_actually_lifted_and_retained') is not True):
                raise RuntimeError('Actual held-object clearance/route/drop permission missing')
            self.authority.bin_drop_permission = copy.deepcopy(event)
            self.report.update(object_lift=True,grip_observed_by_operator=True,
                loaded_lift_observed_by_operator=True,
                grip_observation_source='onsite_operator_not_retention_sensor',
                measured_loaded_clearance_mm=clearance,
                normal_bin_open_without_extra_prompt_authorized=True,
                automatic_fault_open=False,automatic_lateral_motion=True)
            self.save()
            current = upper
            for row in self.bin_route['transit'] + [self.bin_route['bins'][self.report['selected_color']]]:
                current = self.move_bin_segment(current,row['pose'],'Movej_P_Cmd',row['phase'],hand)
            self.report.update(loaded_transfer_performed=True,
                selected_bin_arm_endpoint=copy.deepcopy(current),bin_arrival_feedback_completed=True)
            self.save()
        finally:
            self.bin_motion = None
            self.close_hand()
        # The t event authorized this one normal release at this exact healthy
        # bin endpoint. A motion/communication/protection exception bypasses it.
        self.authority.selected_bin_endpoint_qualified = True
        self.hand_role = 'bin_drop'
        self.report['phase'] = 'preflight_bin_opening'
        self.save()
        hand = self.hand_factory(self,True)
        try:
            hand._configuration_checked = True
            if not same_pose(self.full_arm(),current):
                raise RuntimeError('Arm changed before selected-bin opening')
            if self.call('Get_Collision_Stage') != [0,8] or self.call('Get_Self_Collision_Enable') != [0,True]:
                raise RuntimeError('Protection changed before normal bin opening')
            self.report['phase'] = 'opening_in_selected_bin'
            self.save()
            opened = hand.open_at_bin(self.load_targets,self.load_anchor)
        finally:
            self.close_hand()
        event = self.wait_input(12,'await_empty_bin_return',current,('u','e'))
        if event.get('shell_in_selected_bin_and_hand_empty') is not True:
            raise RuntimeError('Actual drop and empty hand not observed; no return')
        self.report.update(shell_in_selected_bin_observed_by_operator=True,
            drop_observation_source='onsite_operator_not_bin_sensor',
            bin_release_feedback_completed=True)
        self.load_targets = DROP_TARGET[:]
        self.load_anchor = copy.deepcopy(opened)
        if event['value'] == 'u':
            self.hand_role = 'empty_bin_return'
            hand = self.hand_factory(self,True)
            try:
                hand._configuration_checked = True
                hand.preflight_empty(DROP_TARGET)
                # Empty return uses its own healthy opening target anchor;
                # it cannot reuse MovingLoadedHand's >=8000 loaded bounds.
                current = self.move_bin_segment(current,self.bin_route['transit'][-1]['pose'],
                    'Movej_P_Cmd','returning_empty_to_school_restart',hand)
                self.report['empty_return_completed'] = True
            finally:
                self.bin_motion = None
                self.close_hand()
        self.authority._completed = True
        self.report.update(outcome=SUCCESS,phase='completed',execution_ready=False,
            final_arm=current,final_hand=copy.deepcopy(self.report.get('last_empty_bin_return_monitor',opened)),
            holding_verified=False,
            final_hand_source='last_fresh_empty_return_or_bin_opening_feedback',
            lateral_transfer_ready=False,autonomous_other_targets_ready=False,
            functional_validation='actual_feedback_plus_operator_grip_clearance_and_drop_observations')
        self.save()
