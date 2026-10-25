"""Normal bin transfer from a freshly qualified current held position; no lift."""
import copy
import math
from school_bin_cycle_v4_sequence import SchoolBinCycle,SUCCESS,DROP_TARGET
from high_descent_prepare_worker import same_pose


class HeldTransferSequence(SchoolBinCycle):
    def deliver_current_upper(self,upper):
        self.prepare_bin_route(upper)
        self.hand_role='moving'
        hand=self.hand_factory(self,True)
        try:
            hand._configuration_checked=True
            hand.preflight_loaded(self.load_targets,self.load_anchor)
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
        if not same_pose(self.full_arm(),current):
            raise RuntimeError('Arm changed before selected-bin opening')
        if self.call('Get_Collision_Stage') != [0,8] or self.call('Get_Self_Collision_Enable') != [0,True]:
            raise RuntimeError('Protection changed before normal bin opening')
        hand = self.hand_factory(self,True)
        try:
            hand._configuration_checked = True
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
        drop_targets=getattr(self,'tuned_drop_targets',DROP_TARGET)
        self.load_targets = drop_targets[:]
        self.load_anchor = copy.deepcopy(opened)
        if event['value'] == 'u':
            self.hand_role = 'empty_bin_return'
            hand = self.hand_factory(self,True)
            try:
                hand._configuration_checked = True
                hand.preflight_empty(drop_targets)
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
