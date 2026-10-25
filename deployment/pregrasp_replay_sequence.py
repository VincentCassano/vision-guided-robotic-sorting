"""One current teach pose,100mm lift, then explicit100mm descent. No grasp."""
import copy
import json
import math
from high_descent_prepare_worker import HighSequence,same_pose
from run_arm_hand_sequence import Sequence
from run_school_battery_hand import ADAPTED_PLAN
from battery_grasp.high_descent_trial import PREPARE
from battery_grasp.pregrasp_replay import segment,check_segment
from battery_grasp.real_grasp import orientation_angle_rad


class RehearsalSequence(HighSequence):
    def arm_state(self):
        if (self.folder/'abort.json').exists():raise KeyboardInterrupt('Parent abort')
        q,p=Sequence.arm_state(self)
        if self.drive_limits is not None and any(min(v-a,b-v)<3 for v,a,b in zip(q,
                self.drive_limits['minimum'],self.drive_limits['maximum'])):
            raise RuntimeError('Live drive reserve below3degrees')
        if self.plan is not None:
            angle=orientation_angle_rad(p[3:],self.plan['start']['pose'][3:])
            self.report['last_wrist_orientation_check']=dict(
                metric='RzRyRx_relative_rotation_angle',physical_rotation_rad=angle,
                limit_rad=.005,current_rpy_rad=p[3:],start_rpy_rad=self.plan['start']['pose'][3:])
            check_segment(dict(self.plan['start'],joints=q,pose=p),self.plan)
        return q,p

    def permission(self,index,phase,arm,allowed,teaching=False):
        self.report.update(permission_index=index,phase=phase);self.save()
        began=self.clock();limit=600 if teaching else 120
        path=self.folder/'permissions'/('%d.json'%index)
        while self.clock()-began<limit:
            if (self.folder/'abort.json').exists():raise KeyboardInterrupt('Cancelled')
            if path.exists():
                event=json.loads(path.read_text(encoding='utf8'))
                if event.get('index')!=index or event.get('phase')!=phase:raise RuntimeError('Stale permission')
                self.report.setdefault('operator_inputs',[]).append(event);self.save()
                if event.get('value')=='q':raise EOFError('Operator quit; retain pose')
                if event.get('value')!=allowed:raise RuntimeError('Incorrect permission')
                return event
            # The explicit teaching handoff permits manual arm repositioning.
            # No SDK queries or motion writes while the operator uses teaching.
            if not teaching:
                q,p=self.arm_state()
                if not same_pose(dict(arm,joints=q,pose=p),arm):raise RuntimeError('Arm moved during permission wait')
            self.pause(.15)
        raise TimeoutError('Operator wait expired')

    def move_segment(self,arm,tip,delta,phase):
        self.report['phase']='preflight_'+phase;self.save()
        fresh,latest=self.preflight(arm,PREPARE)
        if latest!=tip:raise RuntimeError('Tip parameters changed; no next movement')
        self.plan=segment(fresh,self.drive_limits,delta,latest['line_speed_m_s'],latest['line_acc_m_s2'])
        if (self.folder/'abort.json').exists():raise KeyboardInterrupt('Cancelled before MoveL')
        self.report.update(plan=self.plan,phase=phase,execution_ready=True,arm_command_attempted=True)
        self.report['arm_actuator_writes']+=1;self.arm_command_attempted=True;self.save()
        answer=self.call('Movel_Cmd',self.plan['target_sdk_pose_m_rad'],self.plan['speed_percent'],
            trajectory_connect=0,r=0,block=False)
        if type(answer) is not int or answer!=0:raise RuntimeError('MoveL rejected/uncertain; no retry')
        endpoint=self.wait_target()
        self.report.setdefault('completed_segments',[]).append(dict(phase=phase,plan=copy.deepcopy(self.plan),
            endpoint=endpoint,actual_world_delta_mm=[(b-a)*1000 for a,b in zip(fresh['pose'][:3],endpoint['pose'][:3])]))
        self.save();return endpoint

    def wait_target(self):
        start=self.clock();previous=None;quiet_since=None;groups=0
        while self.clock()-start<self.plan['arm_timeout_seconds']:
            q,p=self.arm_state()
            reached=(math.dist([v*1000 for v in p[:3]],self.plan['target_world_mm'])<=.5
                and orientation_angle_rad(p[3:],self.plan['start']['pose'][3:])<=.005)
            quiet=(previous is not None and max(abs(a-b) for a,b in zip(q,previous[0]))<=.02
                and math.dist(p[:3],previous[1][:3])<=.0002
                and orientation_angle_rad(p[3:],previous[1][3:])<=.002)
            if reached and quiet:
                if quiet_since is None:quiet_since=self.clock()
                groups+=1
                if groups>=3 and self.clock()-quiet_since>=1:
                    endpoint=self.full_arm();check_segment(endpoint,self.plan)
                    if math.dist([v*1000 for v in endpoint['pose'][:3]],self.plan['target_world_mm'])>.5:
                        raise RuntimeError('Fresh endpoint differs from settled feedback')
                    self.guard_target,self.guard_pose=endpoint['joints'][:],endpoint['pose'][:]
                    return endpoint
            else:quiet_since=None;groups=0
            previous=q,p;self.pause(.1)
        raise TimeoutError('Signed100mm segment failed feedback deadline; no retry or return')

    def run(self):
        initial,tip=self.preflight(self.reference,None)
        previous=self.report['last_preflight']['targets'][:]
        if any(abs(a-b)>2 for a,b in zip(previous,PREPARE)):
            event=self.permission(0,'await_hand_prepare',initial,'h')
            if event.get('empty_hand_prepare_sweep_clear') is not True:raise RuntimeError('Empty hand swept range not confirmed')
            self.report['phase']='preparing_empty_hand';self.save()
            _,_,hand=self.preflight(initial,previous,keep_hand=True)
            try:
                self.io.set_phase('prepare_empty',self.authority);self.io.enable_motion();self.hand_motion_enabled=True
                self.report['hand_register_writes']+=1;self.save()
                hand.write_pose(ADAPTED_PLAN[0],previous)
                hand.preflight(expected=PREPARE)
            finally:self.close_hand()
        self.report['teach_handoff_started']=True;self.save()
        taught=self.permission(1,'await_teach_endpoint',initial,'t',teaching=True)
        gap=taught.get('hand_minimum_clearance_mm')
        if (type(gap) not in (int,float) or not math.isfinite(gap) or gap<14
                or taught.get('battery_in_grasp_space_no_contact') is not True
                or taught.get('teaching_exited_and_hands_removed') is not True):
            raise RuntimeError('Noncontact taught endpoint not confirmed')
        self.report['phase']='preflight_taught_endpoint';self.save()
        goal=self.full_arm()
        if goal['frame_signatures']!=initial['frame_signatures'] or goal['limits']!=initial['limits']:
            raise RuntimeError('Frames or joint configuration changed during teaching')
        # Explicit new manual point, before ANY automatic arm command. No stale
        # historical coordinate, wrist, clearance or transfer is reused.
        self.reference=goal
        goal,tip=self.preflight(goal,PREPARE)
        up=segment(goal,self.drive_limits,100,tip['line_speed_m_s'],tip['line_acc_m_s2'])
        self.report.update(taught_endpoint=goal,taught_clearance=taught,plan=up,
            diagnostic_pass=True,execution_ready=False,phase='taught_endpoint_read')
        self.save()
        event=self.permission(2,'await_lift',goal,'u')
        if event.get('whole_path_clear_and_actual_stop') is not True:raise RuntimeError('Full swept range not confirmed')
        self.enable_trial_protections(goal,copy.deepcopy(self.report['arm_safety_reads']))
        self.plan=up
        upper=self.move_segment(goal,tip,100,'lifting_100mm')
        self.report['upper_endpoint']=upper;self.save()
        event=self.permission(3,'await_descent',upper,'d')
        if event.get('actual_stop_and_scene_unchanged') is not True:raise RuntimeError('Upper stop/unchanged scene not confirmed')
        # Discard the old UP guard only AFTER accepted upper feedback. The new
        # downward candidate starts at fresh actual upper pose, same XY/wrist.
        self.plan=None
        delta=(goal['pose'][2]-upper['pose'][2])*1000
        final=self.move_segment(upper,tip,delta,'descending_to_pregrasp')
        if (math.dist(final['pose'][:3],goal['pose'][:3])>.001
                or orientation_angle_rad(final['pose'][3:],goal['pose'][3:])>.005):
            raise RuntimeError('Final pose differs from current taught pregrasp')
        self.report['phase']='final_preflight';self.save()
        _,_,hand=self.preflight(final,PREPARE,keep_hand=True)
        try:self.report['final_hand']=hand.preflight(expected=PREPARE)
        finally:self.close_hand()
        self.report.update(outcome='completed_current_pregrasp_replay',phase='completed',
            final_endpoint=final,pregrasp_replay_feedback_observed=True,
            physical_stop_verified=False,torque_release_verified=False,
            vision_target_binding_verified=False,full_pregrasp_execution_ready=False)
        self.save()
