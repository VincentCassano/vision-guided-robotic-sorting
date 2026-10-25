#!/usr/bin/env python3
"""One50mm high empty-hand supervised trial, current pose only; isolated SDK."""
import argparse
import contextlib
from datetime import datetime,timezone
import hashlib
import json
import math
from pathlib import Path
import platform
import sys
import time
import traceback

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/battery_grasp/src'))
from battery_grasp.high_descent_trial import build_trial,check_live,OPEN,PREPARE
from battery_grasp.taught_dh_model import EXPECTED_DH
from battery_grasp.real_grasp import orientation_angle_rad
from run_arm_hand_sequence import Sequence


def same_pose(a,b):
    return (a['frame_signatures']==b['frame_signatures'] and
        a['limits']['minimum']==b['limits']['minimum'] and a['limits']['maximum']==b['limits']['maximum']
        and all(abs(x-y)<=.08 for x,y in zip(a['joints'],b['joints']))
        and math.dist(a['pose'][:3],b['pose'][:3])<=.0005
        and orientation_angle_rad(a['pose'][3:],b['pose'][3:])<=.005)


class HighSequence(Sequence):
    def __init__(self,*args,folder,authority,tip_reader,**kwargs):
        super().__init__(*args,**kwargs)
        self.folder,self.authority,self.tip_reader=folder,authority,tip_reader
        self.hand_motion_enabled=False
        self.plan=None
        self.drive_limits=None
        # Initial read-only preflight may report missing protections when the
        # operator explicitly opted in. No setting before the m permission.
        self.protection_prepared=not self.enable_arm_protection

    def close_hand(self):
        if self.io is not None:
            io,self.io=self.io,None
            self.hand_motion_enabled=False
            io.disarm_motion()
            try:io.sock.close()
            except BaseException:
                io.disable_motion();raise

    def arm_state(self):
        if (self.folder/'abort.json').exists():raise KeyboardInterrupt('Parent requested abort')
        q,p=super().arm_state()
        if self.drive_limits is not None and any(min(v-a,b-v)<3 for v,a,b in zip(q,
                self.drive_limits['minimum'],self.drive_limits['maximum'])):
            raise RuntimeError('Live drive reserve below3degrees')
        if self.plan is not None:
            live=dict(self.plan['start'],joints=q,pose=p)
            check_live(live,self.plan)
        return q,p

    def preflight(self,expected_arm,expected_hand,keep_hand=False):
        from sdk_environment import collect_sdk_environment
        from pregrasp_measurement_worker import NATIVE_SHA
        metadata=collect_sdk_environment(self.robot)
        self.report['sdk_environment']=metadata;self.save()
        if (not metadata['metadata_complete'] or metadata.get('api_version')!='4.3.1'
                or metadata.get('loaded_native',{}).get('sha256')!=NATIVE_SHA
                or metadata.get('host_machine')!='aarch64'
                or metadata.get('controller_run_mode_raw',{}).get('data')!=[1]):
            raise RuntimeError('Native identity/real-mode preflight failed')
        arm=self.full_arm()
        if expected_arm is None:
            if self.reference is not None or self.plan is not None or self.arm_command_attempted:
                raise RuntimeError('Startup reference may only be bound once before any motion')
            self.reference=arm
            self.report['startup_reference']=arm
            self.report['reference_source']='this_session_fresh_controller_readback'
            self.report['historical_clearances_reused']=False
            self.save()
        elif not same_pose(arm,expected_arm):
            raise RuntimeError('Arm changed after this session bound its start; no automatic alignment')
        self.guard_target,self.guard_pose=arm['joints'][:],arm['pose'][:]
        low,high=self.safety_reads(allow_disabled=self.enable_arm_protection and not self.protection_prepared)
        self.drive_limits=dict(minimum=low,maximum=high)
        install=self.report['arm_safety_reads']['Get_Install_Pose']
        if (not isinstance(install,list) or len(install)!=4 or install[0]!=0
                or any(abs(a-b)>.001 for a,b in zip(install[1:],[0,45,0]))):
            raise RuntimeError('Actual installation differs from measured model')
        if self.io is not None:raise RuntimeError('Tip parameter GET requires a closed hand socket')
        tip=self.tip_reader()
        self.report['live_tip_parameters']=tip;self.save()
        hand=self.hand_factory(self,True)
        try:
            hand.configuration()
            hand.preflight(expected=expected_hand)
        except BaseException:
            self.close_hand();raise
        if not keep_hand:self.close_hand()
        return (arm,tip,hand) if keep_hand else (arm,tip)

    def enable_trial_protections(self,arm,initial):
        """Only after m: one-way8/True, no retries, fresh replies before motion."""
        if not self.enable_arm_protection:
            return
        self.report['phase']='preparing_arm_protection';self.save()
        if not same_pose(self.full_arm(),arm):
            raise RuntimeError('Arm changed before protection setup; no write')
        collision=self.call('Get_Collision_Stage')
        own=self.call('Get_Self_Collision_Enable')
        if collision!=initial['Get_Collision_Stage'] or own!=initial['Get_Self_Collision_Enable']:
            raise RuntimeError('Protection changed after preflight; no setup write')
        # Observed quiet feedback supplements the onsite actual-stop confirmation;
        # it is not a physical stop test and cannot authorize a moving arm.
        self.settled(arm['joints'],timeout=8)
        setup=dict(before=dict(collision=collision,self_collision=own),
            writes=[],readbacks=[],automatic_restore=False,physical_stop_verified=False)
        self.report['arm_protection_setup']=setup;self.save()
        if (self.folder/'abort.json').exists():raise KeyboardInterrupt('Cancelled before protection setup')
        commands=[]
        if collision!=[0,8]:commands.append(('Set_Collision_Stage',8,dict(block=True)))
        if own!=[0,True]:commands.append(('Set_Self_Collision_Enable',True,{}))
        if commands:self.protection_client.authorize_settings()
        for name,value,kwargs in commands:
            if (self.folder/'abort.json').exists():raise KeyboardInterrupt('Cancelled during protection setup')
            attempt=dict(method=name,value=value,outcome='attempted_result_unknown')
            setup['writes'].append(attempt)
            self.report['controller_configuration_writes']=self.report.get('controller_configuration_writes',0)+1
            self.save()
            status=self.call(name,value,**kwargs)
            attempt['status']=status;self.save()
            if type(status) is not int or status!=0:
                raise RuntimeError('Protection setting rejected/uncertain; no retry or motion')
            attempt['outcome']='acknowledged_not_stop_performance_verified';self.save()
        for _ in range(2):
            observed=dict(collision=self.call('Get_Collision_Stage'),
                self_collision=self.call('Get_Self_Collision_Enable'))
            setup['readbacks'].append(observed);self.save()
            if (observed['collision']!=[0,8] or observed['self_collision']!=[0,True]
                    or type(observed['self_collision'][1]) is not bool):
                raise RuntimeError('Protection fresh readback mismatch; no retry or motion')
            q,p=self.arm_state()
            if not same_pose(dict(arm,joints=q,pose=p),arm):
                raise RuntimeError('Arm moved during protection setup; no next write')
        self.protection_prepared=True
        setup['outcome']='already_enabled_readback_only' if not commands else 'configuration_readbacks_passed'
        self.save()

    def wait_permission(self,index,phase,expected_arm,seconds=120):
        self.report.update(phase=phase,permission_index=index)
        self.save();began=self.clock()
        path=self.folder/'permissions'/('%d.json'%index)
        while self.clock()-began<seconds:
            if (self.folder/'abort.json').exists():raise KeyboardInterrupt('Parent requested abort')
            if path.exists():
                event=json.loads(path.read_text(encoding='utf8'))
                if event.get('index')!=index or event.get('phase')!=phase:
                    raise RuntimeError('Stale/incorrect parent permission')
                self.report.setdefault('operator_inputs',[]).append(event);self.save()
                if event.get('value') not in ('m','p'):
                    raise EOFError('Operator quit; no automatic return or open')
                return event
            q,p=self.arm_state()
            live=dict(expected_arm,joints=q,pose=p)
            if not same_pose(live,expected_arm):raise RuntimeError('Arm moved while waiting; no next write')
            self.pause(.15)
        raise TimeoutError('Onsite permission wait expired; no next write')

    def wait_target(self):
        start=self.clock();previous=None;stable_since=None;groups=0
        while self.clock()-start<self.plan['arm_timeout_seconds']:
            if (self.folder/'abort.json').exists():raise KeyboardInterrupt('Parent abort during arm motion')
            q,p=self.arm_state()
            reached=(math.dist([v*1000 for v in p[:3]],self.plan['target_world_mm'])<=.5
                and all(abs(math.atan2(math.sin(a-b),math.cos(a-b)))<=.005
                        for a,b in zip(p[3:],self.plan['start']['pose'][3:])))
            quiet=previous is not None and all(abs(a-b)<=.02 for a,b in zip(q,previous[0]))
            quiet=quiet and math.dist(p[:3],previous[1][:3])<=.0002
            quiet=quiet and all(abs(math.atan2(math.sin(a-b),math.cos(a-b)))<=.002
                                for a,b in zip(p[3:],previous[1][3:]))
            if reached and quiet:
                if stable_since is None:stable_since=self.clock()
                groups+=1
                if groups>=3 and self.clock()-stable_since>=1:
                    arm=self.full_arm();check_live(arm,self.plan)
                    if math.dist([v*1000 for v in arm['pose'][:3]],self.plan['target_world_mm'])>.5:
                        raise RuntimeError('Fresh endpoint differs after stable feedback')
                    self.guard_target,self.guard_pose=arm['joints'][:],arm['pose'][:]
                    return arm
            else:stable_since=None;groups=0
            previous=q,p;self.pause(.1)
        raise TimeoutError('50mm MoveL feedback timeout; no retry or hand motion')

    def run(self):
        arm,tip=self.preflight(self.reference,OPEN)
        self.plan=build_trial(arm,self.drive_limits,tip['line_speed_m_s'],tip['line_acc_m_s2'])
        original_protections=self.report.get('arm_safety_reads',{}).copy()
        self.report['protection_setup_opt_in']=self.enable_arm_protection
        self.report.update(plan=self.plan,diagnostic_pass=True,
            execution_ready=False,full_pregrasp_execution_ready=False)
        self.save()
        event=self.wait_permission(1,'await_move_permission',arm)
        if event['value']!='m' or event.get('qualified_onsite_full_trial_clear') is not True:
            raise RuntimeError('Onsite whole trial supervision/clearance not confirmed')
        self.enable_trial_protections(arm,original_protections)
        self.report['phase']='fresh_before_move';self.save()
        fresh,tip_after=self.preflight(arm,OPEN)
        if tip_after!=tip:raise RuntimeError('Tip motion parameters changed during permission')
        # Recompute all numerical rows from the NEW fresh start, not an old point.
        self.plan=build_trial(fresh,self.drive_limits,tip_after['line_speed_m_s'],tip_after['line_acc_m_s2'])
        if (self.folder/'abort.json').exists():raise KeyboardInterrupt('Parent cancelled before MoveL')
        self.report.update(plan=self.plan,execution_ready=True,phase='moving_high50mm',
            arm_command_attempted=True,arm_actuator_writes=1,
            qualified_onsite_trial_confirmation=True,full_scene_geometry_certified=False)
        self.arm_command_attempted=True;self.save()
        status=self.call('Movel_Cmd',self.plan['target_sdk_pose_m_rad'],self.plan['speed_percent'],
                         trajectory_connect=0,r=0,block=False)
        if type(status) is not int or status!=0:raise RuntimeError('MoveL rejected/uncertain; no retry')
        endpoint=self.wait_target()
        self.report.update(arm_feedback_observed=True,actual_world_delta_mm=[
            (b-a)*1000 for a,b in zip(fresh['pose'][:3],endpoint['pose'][:3])],
            endpoint=endpoint,phase='arm_feedback_observed')
        self.save()
        event=self.wait_permission(2,'await_prepare_permission',endpoint)
        if event['value']!='p' or event.get('actual_arm_stop_and_empty_prepare_sweep_confirmed') is not True:
            raise RuntimeError('Actual stop/empty-hand PREPARE swept range not confirmed')
        self.report['phase']='fresh_before_prepare';self.save()
        _,_,hand=self.preflight(endpoint,OPEN,keep_hand=True)
        try:
            self.io.set_phase('prepare_empty',self.authority)
            self.io.enable_motion();self.hand_motion_enabled=True
            self.report.update(phase='preparing_empty_hand',hand_register_writes=1);self.save()
            from run_school_battery_hand import ADAPTED_PLAN
            hand.write_pose(ADAPTED_PLAN[0],OPEN[:])
            self.report['final_hand']=hand.preflight(expected=PREPARE)
        finally:self.close_hand()
        q,p=self.arm_state()
        if not same_pose(dict(endpoint,joints=q,pose=p),endpoint):raise RuntimeError('Arm drifted during PREPARE')
        self.report.update(outcome='completed_high50mm_then_PREPARE',phase='completed',
            arm_feedback_observed=True,prepare_feedback_observed=True,
            physical_stop_verified=False,torque_release_verified=False,
            full_pregrasp_execution_ready=False)
        self.save()

    def fault(self):
        self.hand_motion_enabled=False
        self.authority._terminal_latched=True
        if self.io is not None:
            try:self.io.disable_motion()
            except BaseException as exc:self.report.setdefault('fault_errors',[]).append(repr(exc))
        if self.arm_command_attempted and not self.arm_stop_attempted:
            self.arm_stop_attempted=True
            try:
                self.report['arm_stop_attempt']=dict(result=self.call('Move_Stop_Cmd',block=False),
                    physical_stop_verified=False,hand_stop_or_release=False)
            except BaseException as exc:
                self.report['arm_stop_attempt']=dict(error=repr(exc),physical_stop_verified=False)
        self.report.update(execution_ready=False,phase='fault',onsite_hardware_stop_required=True,
            automatic_open=False,automatic_return=False,automatic_force_zero=False)
        self.save()


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--folder',required=True,type=Path)
    p.add_argument('--sdk-dir',type=Path,default=Path('/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package'))
    p.add_argument('--execute',action='store_true')
    p.add_argument('--enable-arm-protection',action='store_true')
    args=p.parse_args(argv)
    folder=args.folder;folder.mkdir(parents=True,exist_ok=True)
    if not args.execute:p.error('Explicit single supervised trial execution required')
    report=dict(schema_version=1,scope='qualified_onsite_empty_hand_high50mm_trial',
        timestamp_utc=datetime.now(timezone.utc).isoformat(),outcome='starting',phase='preflight',
        diagnostic_pass=False,execution_ready=False,full_pregrasp_execution_ready=False,
        sdk_in_parent=False,exceptions=[],cleanup_errors=[],arm_actuator_writes=0,
        hand_register_writes=0,controller_configuration_writes=0,physical_stop_verified=False,
        torque_release_verified=False,automatic_open=False,automatic_return=False,
        automatic_grip=False,automatic_lift=False)
    def save():
        temp=folder/'worker_result.tmp'
        temp.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf8')
        temp.replace(folder/'worker_result.json')
    robot=seq=None;save()
    try:
        from pregrasp_measurement_worker import source_audit
        report['source_audit']=source_audit(args.sdk_dir);save()
        import fcntl,socket
        from inspect_left_hand import connect
        from continuous_hand_cycle import ContinuousHand
        from supported_contact_transport import SupportedContactTransport,PhaseAuthority
        from arm_protection_transport import ProtectionClient
        from taught_model_readonly import collect_model_parameters
        from high_trial_tip_readonly import collect
        # Bind the first fresh snapshot in preflight. Historical measured poses
        # and clearances do not authorize this session or cause an old-point move.
        reference=None
        def append(out,row):out.write(json.dumps(row)+'\n');out.flush()
        class PrepareOnly(SupportedContactTransport):
            def set_phase(self,phase,authority):
                if phase!='prepare_empty':
                    self.latch_fault('Only empty PREPARE permitted');raise RuntimeError('No grip/reset/release write phase')
                return super().set_phase(phase,authority)
        with (ROOT/'captures/hand_stage.lock').open('a') as lock, \
             (folder/'hand_wire.jsonl').open('w',encoding='utf8') as wire, \
             (folder/'protection_wire.jsonl').open('w',encoding='utf8') as protection_wire, \
             (folder/'tip_wire.jsonl').open('w',encoding='utf8') as tip_wire, \
             (folder/'samples.jsonl').open('w',encoding='utf8') as samples:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            model=collect_model_parameters(folder/'fresh_model')
            report['fresh_model']=model;save()
            if (not model['dh_valid'] or not model['identity_valid'] or model['errors']
                    or [model['dh_response']['joint_'+str(i)] for i in range(1,7)]!=EXPECTED_DH):
                raise RuntimeError('Fresh actual identity/DH differs; no setters or motion')
            profile=json.loads((ROOT/'deployment/battery_hand_verified_profile.json').read_bytes())
            authority=PhaseAuthority()
            def factory(sequence,guarded):
                if not guarded or sequence.io is not None:raise RuntimeError('Single guarded hand session required')
                sequence.io=PrepareOnly(socket.create_connection(('169.254.128.18',8080),timeout=3),
                    lambda row:append(wire,row),authority)
                return ContinuousHand(sequence,sequence.io,report,save,samples)
            robot=connect(args.sdk_dir)
            seq=HighSequence(robot,reference,profile,report,save,samples,factory,folder=folder,authority=authority,
                tip_reader=lambda:collect(lambda row:append(tip_wire,row)),
                enable_arm_protection=args.enable_arm_protection,
                protection_client=ProtectionClient(lambda row:append(protection_wire,row)))
            try:seq.run()
            except BaseException:
                seq.fault();raise
            finally:seq.close_hand()
    except BaseException as exc:
        report['exceptions'].append(dict(error=repr(exc),traceback=traceback.format_exc()))
        report.update(outcome='aborted_no_further_writes',phase='fault',execution_ready=False)
    finally:
        if robot is not None:
            for name in ('Arm_Socket_Close','RM_API_UnInit'):
                try:getattr(robot,name)()
                except BaseException as exc:report['cleanup_errors'].append(dict(method=name,error=repr(exc)))
        if report['cleanup_errors']:report['outcome']='failed_cleanup'
        save()
    return 0 if report['outcome']=='completed_high50mm_then_PREPARE' else 1


if __name__=='__main__':raise SystemExit(main())
