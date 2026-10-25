"""One prospectively authorized normal route; no loaded keyboard wait.

Upfront scene/route clearance is an operator assertion, not a future measured
clearance or sensor-certified retention. Every segment retains actual SDK,
hand, protection, endpoint and asynchronous notification gates.
"""
import copy
from high_descent_prepare_worker import same_pose
from school_bin_cycle_v4_sequence import SUCCESS
from visual_pick_sort_v7_connected import move_connected_transfer


class ContinuousDelivery:
    def deliver_held_shell(self,end):
        event=getattr(self.authority,'continuous_cycle_permission',None)
        if (not isinstance(event,dict) or type(event.get('index')) is not int or not 15<=event['index']<115
                or event.get('live_tuned_grip_and_continuous_cycle_authorized') is not True
                or event.get('continuous_lift_transfer_normal_drop_authorized') is not True
                or event.get('planned_loaded_route_min20mm_clear_confirmed') is not True
                or self.bin_route is None or self.authority._terminal_latched):
            raise RuntimeError('Upfront full-cycle permission/healthy grip/route missing')
        self.report.update(normal_transfer_requires_fresh_loaded_clearance_permission=False,
            normal_transfer_uses_upfront_supervised_route_permission=True,
            planned_clearance_source='onsite_operator_before_grasp_not_postlift_measurement',
            measured_loaded_clearance_mm=None,retention_sensor_verified=False,
            loaded_lift_observed_by_operator=False,object_lift=False,
            automatic_lateral_motion=True,loaded_keyboard_waits=0)
        self.plan=None
        arm=self.full_arm()
        if not same_pose(arm,end) or arm['limits']!=end['limits']:
            raise RuntimeError('Arm changed during grip; no lift')
        self.guard_target,self.guard_pose=arm['joints'][:],arm['pose'][:]
        self.hand_role='moving';hand=self.hand_factory(self,True)
        try:
            hand._configuration_checked=True
            hand.preflight_loaded(self.load_targets,self.load_anchor)
            upper_target=arm['pose'][:];upper_target[2]+=.140
            upper=self.move_bin_segment(arm,upper_target,'Movel_Cmd',
                'lifting_loaded_140mm',hand,speed=self.loaded_lift_speed_percent)
            self.report.update(loaded_lift_performed=True,load_lift_or_transfer_performed=True,
                loaded_arm_up_feedback_observed=True,object_lift=False,
                actual_object_lift_not_yet_observed=True)
            self.save()
            rows=self.bin_route['transit']+[self.bin_route['bins'][self.report['selected_color']]]
            current=move_connected_transfer(self,upper,rows,hand)
            self.report.update(loaded_transfer_performed=True,
                selected_bin_arm_endpoint=copy.deepcopy(current),bin_arrival_feedback_completed=True)
            self.authority.continuous_loaded_route_completed=True
            self.save()
        finally:
            self.bin_motion=None;self.close_hand()
        self.authority.selected_bin_endpoint_qualified=True
        self.hand_role='bin_drop';self.report['phase']='preflight_bin_opening';self.save()
        if not same_pose(self.full_arm(),current):raise RuntimeError('Arm changed before bin opening')
        if self.call('Get_Collision_Stage')!=[0,8] or self.call('Get_Self_Collision_Enable')!=[0,True]:
            raise RuntimeError('Protection changed before normal bin opening')
        hand=self.hand_factory(self,True)
        try:
            hand._configuration_checked=True;self.report['phase']='opening_in_selected_bin';self.save()
            opened=hand.open_at_bin(self.load_targets,self.load_anchor)
        finally:self.close_hand()
        # Only after normal opening: ask what actually happened. Never label
        # advance authorization as a measured clearance or observed lift.
        event=self.wait_input(12,'await_empty_bin_return',current,('u','e','f'))
        if event.get('value')=='f' and event.get('hand_empty_but_pickup_failed') is True:
            self.authority._completed=True
            self.report.update(outcome='motion_route_completed_pickup_failed_operator',phase='completed',
                execution_ready=False,final_arm=current,final_hand=copy.deepcopy(opened),
                actual_pickup_failed_by_operator=True,shell_in_selected_bin_observed_by_operator=False,
                object_lift=False,observed_full_transfer_at_end=False,empty_return_completed=False)
            self.save();return
        if event.get('shell_in_selected_bin_and_hand_empty') is not True:
            raise RuntimeError('Actual drop/empty hand not observed; no return')
        self.report.update(shell_in_selected_bin_observed_by_operator=True,
            drop_observation_source='onsite_operator_after_normal_opening_not_bin_sensor',
            bin_release_feedback_completed=True,object_lift=True,
            observed_full_transfer_at_end=True)
        drop=self.tuned_drop_targets;self.load_targets=drop[:];self.load_anchor=copy.deepcopy(opened)
        if event['value']=='u':
            self.hand_role='empty_bin_return';hand=self.hand_factory(self,True)
            try:
                hand._configuration_checked=True;hand.preflight_empty(drop)
                current=self.move_bin_segment(current,self.bin_route['transit'][-1]['pose'],
                    'Movej_P_Cmd','returning_empty_to_school_restart',hand)
                self.report['empty_return_completed']=True
            finally:self.bin_motion=None;self.close_hand()
        self.authority._completed=True
        self.report.update(outcome=SUCCESS,phase='completed',execution_ready=False,
            final_arm=current,final_hand=copy.deepcopy(self.report.get('last_empty_bin_return_monitor',opened)),
            holding_verified=False,final_hand_source='fresh_empty_return_or_bin_opening_feedback',
            lateral_transfer_ready=False,autonomous_other_targets_ready=False,
            functional_validation='actual_feedback_plus_operator_observed_sort_result_after_opening')
        self.save()
