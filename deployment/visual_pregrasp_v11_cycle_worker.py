"""Isolated supervised vision-target alignment; no grasp/lift/drop capability."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import platform
import socket
import sys
import time
import traceback

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/battery_grasp/src'))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--folder',type=Path,required=True)
    parser.add_argument('--vision-archive',type=Path,required=True)
    parser.add_argument('--sdk-dir',type=Path,required=True)
    parser.add_argument('--execute',action='store_true',required=True)
    parser.add_argument('--delivery-manifest-sha256',required=True)
    parser.add_argument('--retract-only',action='store_true')
    parser.add_argument('--completed-run',type=Path,required=True)
    args=parser.parse_args()
    if platform.system()!='Linux' or platform.machine()!='aarch64':
        parser.error('Reviewed actual Linux aarch64 only; no connection')
    from known_cylinder_records import dump
    folder=args.folder
    report=dict(scope='supervised_visual_target_empty_hand_pregrasp_alignment',phase='preflight',
        arm_actuator_writes=0,hand_register_writes=0,controller_configuration_writes=0,
        execution_ready=False,exceptions=[],cleanup_errors=[],physical_stop_verified=False,
        automatic_open=False,automatic_return=False,grasp_performed=False,
        offline_tests_executed=False,new_target_physical_alignment_accepted=False)
    robot=seq=None
    def save():dump(folder/'worker_result.json',report)
    def append(stream,row):stream.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n');stream.flush()
    save()
    try:
        from visual_run_configuration_v3 import (observe_configuration, bind_or_require_frames,
            require_reference_geometry, require_current_configuration)
        from visual_startup_reads_v4 import StartupReadSession
        from run_visual_pregrasp_stage_v11_cycle import verify_delivery, MANIFEST
        if hashlib.sha256(MANIFEST.read_bytes()).hexdigest()!=args.delivery_manifest_sha256:
            raise ValueError('Delivery manifest changed between parent and worker')
        verify_delivery()
        from visual_pregrasp_source import load_target
        from visual_pick_sort_v7_source import PREPARE
        receipt=load_target(args.vision_archive)
        dump(folder/'vision_source_receipt.json',receipt)
        report['vision_source']=receipt
        from visual_grasp_offset_v8 import corrected_goal,completed_source
        retreat_source=completed_source(args.completed_run) if args.retract_only else None
        if args.retract_only:
            report['retreat_source']=retreat_source
            goal=receipt['target']['pregrasp_tcp_candidate_m_rad'][:]
        else:
            goal,report['grasp_offset']=corrected_goal(receipt)
            dump(folder/'grasp_offset.json',report['grasp_offset'])
        from run_arm_hand_sequence import Sequence
        from high_descent_prepare_worker import same_pose
        from battery_grasp.real_grasp import orientation_angle_rad
        from battery_grasp.taught_dh_model import EXPECTED_DH
        from pregrasp_measurement_worker import source_audit,NATIVE_SHA
        from taught_model_readonly import collect_model_parameters
        from sdk_environment import collect_sdk_environment
        from inspect_left_hand import connect
        from arm_protection_transport import ProtectionClient
        from high_trial_tip_readonly import collect
        from supported_contact_transport import SupportedContactTransport,WritePhase
        from school_battery_transport import MULTI,WRITE
        from school_operational_reads import KnownReadHand,KnownReadAuthority
        from school_operational_readiness import wait_hand_ready
        from school_device_identity import identity_fields,same_live_identity
        from motion_event_hand_transport import MotionEventReadTransport
        import fcntl

        class PrepareTransport(SupportedContactTransport):
            def set_phase(self,*unused,**options):raise RuntimeError('Only the entry-owned exact PREPARE capability exists')
            def authorize_prepare(self):
                if (not getattr(self._phase_authority,'visual_prepare_permission',False)
                        or self.broken or self.terminal_latched or self.motion_enabled
                        or self._phase_authority._attempted_writes):
                    raise RuntimeError('Single empty-hand PREPARE permission missing')
                self._phase=WritePhase.PREPARE_EMPTY;self._phase_index=0
            def _validate(self,message):
                if isinstance(message,dict) and message.get('command')==WRITE:
                    exact=dict(command=WRITE,port=1,address=1135,num=5,
                        data=[b for v in PREPARE[:5] for b in (v>>8,v&255)],device=2)
                    if (message!=exact or any(type(message.get(k)) is not int for k in ('port','address','num','device'))
                            or any(type(v) is not int for v in message.get('data',[]))
                            or not self.motion_enabled or self.broken or self.terminal_latched
                            or self._phase is not WritePhase.PREPARE_EMPTY or self._phase_index!=0
                            or self._phase_authority._attempted_writes):
                        raise RuntimeError('Only one exact five-flex empty PREPARE; opposition remains unwritten')
                    return (1135,tuple(PREPARE[:5]))
                return super()._validate(message)
            def exchange(self,message):
                value=super().exchange(message)
                if message.get('command')==MULTI and message.get('address')==1000:
                    identity_fields(value)
                    if not same_live_identity(value,self._phase_authority.accepted_identity_raw):
                        self.latch_fault('New hand diagnostic/identity event');raise RuntimeError('Hand identity changed')
                    self._phase_authority.accepted_identity_raw=value[:]
                return value

        class MovingTransport(MotionEventReadTransport):
            def exchange(self,message):
                if seq.moving:seq.arm_state()
                value=super().exchange(message)
                if message.get('command')==MULTI and message.get('address')==1000:
                    identity_fields(value)
                    if not same_live_identity(value,self._phase_authority.accepted_identity_raw):
                        self.latch_fault('New hand identity/diagnostic');raise RuntimeError('Hand identity changed')
                    self._phase_authority.accepted_identity_raw=value[:]
                return value

        class Hand(KnownReadHand):
            def guard(self):
                if self.sequence.moving:
                    self.sequence.arm_state()
                    return
                return super().guard()

        class Alignment(Sequence):
            def __init__(self,*values,**options):
                super().__init__(*values,**options)
                self.moving=False;self.motion=None;self.drive_limits=None
                self.folder=folder;self.authority=authority;self.hand_motion_enabled=False
                self.startup_observation_only=True;self.startup_stalled_channels=set()
                self.protection_prepared=True
            def full_arm(self):
                state=super().full_arm()
                bind_or_require_frames(run_configuration,state['frame_signatures'])
                return state
            def call(self,name,*values,**options):
                if not name.startswith('Get_'):
                    if name not in ('Movej_P_Cmd','Movel_Cmd','Move_Stop_Cmd'):
                        raise RuntimeError('No SDK configuration/gripper command capability')
                    if name!='Move_Stop_Cmd' and (not self.moving or self.motion is None
                            or name!=self.motion['method'] or values[0]!=self.motion['target']):
                        raise RuntimeError('Unscoped arm command')
                if name in ('Get_Collision_Stage','Get_Self_Collision_Enable') and self.io is not None:
                    raise RuntimeError('Close hand TCP before raw protection GET')
                answer=super().call(name,*values,**options)
                if name=='Get_Current_Arm_State':
                    report['latest_arm_query_raw']=copy.deepcopy(answer)
                    if (not isinstance(answer,list) or len(answer)!=5
                            or answer[0]!=0 or answer[3:]!=[0,0]):
                        report.setdefault('first_rejected_arm_query_raw',copy.deepcopy(answer))
                if name in ('Movej_P_Cmd','Movel_Cmd'):
                    self.motion['notification']['acknowledged']=type(answer) is int and answer==0
                    self.motion['notification']['acknowledgement_raw']=copy.deepcopy(answer)
                    self.motion['notification']['acknowledgement_observed']=True
                    # Keep the report synchronized with the live notification lane.
                    report['current_segment']=copy.deepcopy(self.motion);save()
                return answer
            def arm_state(self):
                if (folder/'abort.json').exists():raise KeyboardInterrupt('Parent abort')
                q,p=super().arm_state()
                if self.drive_limits is not None and any(min(v-a,b-v)<3 for v,a,b in
                        zip(q,self.drive_limits['minimum'],self.drive_limits['maximum'])):
                    raise RuntimeError('Actual drive reserve below3degrees')
                if self.moving and self.motion['method']=='Movel_Cmd':
                    start,target=self.motion['start']['pose'],self.motion['target']
                    if (math.dist(p[:2],start[:2])>.001 or orientation_angle_rad(p[3:],start[3:])>.005
                            or not min(start[2],target[2])-.001<=p[2]<=max(start[2],target[2])+.001):
                        raise RuntimeError('Visual descent left fixed-wrist vertical corridor')
                return q,p
            def quiet(self,expected,seconds=1.):
                began=time.monotonic();previous=None;count=0
                while count<5 or time.monotonic()-began<seconds:
                    q,p=self.arm_state()
                    if not same_pose(dict(expected,joints=q,pose=p),expected):raise RuntimeError('Arm drifted from accepted current endpoint')
                    if previous is not None and (max(abs(a-b) for a,b in zip(q,previous[0]))>.02
                            or math.dist(p[:3],previous[1][:3])>.0002
                            or orientation_angle_rad(p[3:],previous[1][3:])>.002):
                        raise RuntimeError('Arm not quiet')
                    previous=q,p;count+=1;time.sleep(.25)
            def close_hand(self):
                # Closing a healthy read socket is not an application fault.
                # Fault paths explicitly latch the run before reaching here.
                if self.io is not None:
                    current=self.io;self.io=None
                    current.disarm_motion()
                    try:
                        if isinstance(current,StartupReadSession):current.close()
                        else:current.sock.close()
                    except BaseException as exc:
                        current.latch_fault('Hand socket cleanup failed')
                        report['cleanup_errors'].append(dict(method='hand.close',error=repr(exc)))
                        raise
            def permission(self,index,phase,expected,value):
                self.close_hand()
                report.update(phase=phase,permission_index=index,permission_start_arm=expected)
                save();began=time.monotonic()
                while time.monotonic()-began<570:
                    q,p=self.arm_state()
                    if not same_pose(dict(expected,joints=q,pose=p),expected):raise RuntimeError('Current arm changed while awaiting permission')
                    path=folder/'permissions'/('%d.json'%index)
                    if path.exists():
                        event=json.loads(path.read_text(encoding='utf8'))
                        if event.get('index')!=index or event.get('phase')!=phase:raise RuntimeError('Stale permission')
                        if event.get('value')=='q':raise KeyboardInterrupt('Operator ended this run')
                        if event.get('value')!=value or event.get('scene_unchanged_and_path_clear') is not True:
                            raise RuntimeError('Explicit current path permission missing')
                        report.setdefault('operator_permissions',[]).append(event);save();return
                    time.sleep(.25)
                raise TimeoutError('Permission expired')
            def startup_read_guard(self):
                q,p=self.arm_state()
                if not same_pose(dict(self.reference,joints=q,pose=p),self.reference):
                    raise RuntimeError('Arm changed during read-only startup; no retry')
            def open_hand_reader(self,moving=False,full=False):
                if self.io is not None:raise RuntimeError('Only one hand channel')
                if full:
                    self.io=StartupReadSession(folder,report,save,hand_log,authority,self.startup_read_guard)
                else:
                    sock=socket.create_connection(('169.254.128.18',8080),timeout=3)
                    self.io=(MovingTransport(sock,hand_log,authority,lambda:self.motion['notification'] if self.motion else None)
                        if moving else PrepareTransport(sock,hand_log,authority))
                hand=Hand(self,self.io,report,save,samples)
                if full:hand.configuration()
                else:hand._configuration_checked=True
                hand._critical_configuration()
                return hand
            def protections(self):
                self.close_hand()
                with (folder/'current_configuration_wire.jsonl').open('a',encoding='utf8') as qualified_wire:
                    report['latest_observed_raw_tool']=require_current_configuration(run_configuration,qualified_wire)
                if self.call('Get_Collision_Stage')!=[0,8] or self.call('Get_Self_Collision_Enable')!=[0,True]:
                    raise RuntimeError('Collision protections must remain8/True; no setters')
            def retained_hand_check(self,hand):
                row=hand.snapshot();anchor=report['startup_hand']
                if (any(v not in (2,5) for v in row['status'])
                        or any(abs(a-b)>2 for a,b in zip(row['targets'],anchor['targets']))
                        or any(abs(a-b)>150 for a,b in zip(row['positions'],anchor['positions']))):
                    raise RuntimeError('Retained empty hand moved or target/status changed; no preparation/continuation')
                return row
            def move(self,expected,target,method,phase,speed):
                self.protections()
                fresh=self.full_arm()
                if not same_pose(fresh,expected) or fresh['limits']!=expected['limits']:raise RuntimeError('Segment start changed')
                self.guard_target,self.guard_pose=fresh['joints'][:],fresh['pose'][:]
                self.motion=dict(method=method,target=target[:],start=copy.deepcopy(fresh),
                    notification=dict(acknowledged=False,acknowledgement_observed=False,event_seen=False,method=method,phase=phase))
                hand=self.open_hand_reader(moving=True)
                retained=(phase=='raising_current_empty_hand_before_alignment' and report.get('raise_before_prepare') is True)
                if retained:
                    for _ in range(3):self.retained_hand_check(hand)
                else:hand.preflight(expected=PREPARE)
                if not same_pose(self.full_arm(),fresh):raise RuntimeError('Arm changed during segment hand preflight')
                report.update(phase=phase,execution_ready=True,current_segment=copy.deepcopy(self.motion),
                    controller_planned_joint_path=method=='Movej_P_Cmd',continuous_sweep_verified=False)
                self.arm_command_attempted=True;report['arm_actuator_writes']+=1;self.moving=True;save()
                began=time.monotonic()
                rc=self.call(method,target,speed,trajectory_connect=0,r=0,block=False)
                if type(rc) is not int or rc!=0:raise RuntimeError('Controller rejected/uncertain motion; no retry')
                previous=None;quiet_since=None;groups=0;last_hand=began;endpoint=None
                while time.monotonic()-began<150:
                    q,p=self.arm_state();now=time.monotonic()
                    if now-last_hand>=1.:
                        (self.retained_hand_check(hand) if retained else hand._at_expected(hand.snapshot(),PREPARE));last_hand=time.monotonic()
                    reached=math.dist(p[:3],target[:3])<=.0005 and orientation_angle_rad(p[3:],target[3:])<=.005
                    quiet=(previous is not None and max(abs(a-b) for a,b in zip(q,previous[0]))<=.02
                        and math.dist(p[:3],previous[1][:3])<=.0002 and orientation_angle_rad(p[3:],previous[1][3:])<=.002)
                    if reached and quiet:
                        if quiet_since is None:quiet_since=now
                        groups+=1
                        if groups>=3 and now-quiet_since>=1.:
                            endpoint=self.full_arm()
                            if (endpoint['frame_signatures']!=fresh['frame_signatures'] or endpoint['limits']!=fresh['limits']
                                    or math.dist(endpoint['pose'][:3],target[:3])>.0005
                                    or orientation_angle_rad(endpoint['pose'][3:],target[3:])>.005):
                                raise RuntimeError('Endpoint/frame changed after settling')
                            if retained:self.retained_hand_check(hand)
                            else:hand._at_expected(hand.snapshot(),PREPARE)
                            if not same_pose(self.full_arm(),endpoint):raise RuntimeError('Endpoint changed during final hand check')
                            break
                    else:quiet_since=None;groups=0
                    previous=q,p;time.sleep(.1)
                if endpoint is None:raise TimeoutError('Motion endpoint timeout')
                report.setdefault('arm_segments',[]).append(dict(phase=phase,method=method,speed_percent=speed,
                    target_sdk_pose_m_rad=target[:],endpoint=copy.deepcopy(endpoint),elapsed_seconds=time.monotonic()-began,
                    notification=copy.deepcopy(self.motion['notification']),
                    actual_physical_clearance_verified=False))
                report['current_segment']=copy.deepcopy(self.motion)
                self.close_hand();self.moving=False;self.motion=None
                self.guard_target,self.guard_pose=endpoint['joints'][:],endpoint['pose'][:]
                report['execution_ready']=False;save();return endpoint

        report['sdk_source_audit']=source_audit(args.sdk_dir)
        (ROOT/'captures').mkdir(exist_ok=True)
        with (ROOT/'captures/hand_stage.lock').open('a') as lock, \
             (folder/'samples.jsonl').open('w',encoding='utf8') as samples, \
             (folder/'hand_wire.jsonl').open('w',encoding='utf8') as wire, \
             (folder/'protection_wire.jsonl').open('w',encoding='utf8') as pwire, \
             (folder/'tip_wire.jsonl').open('w',encoding='utf8') as twire:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            report['fresh_model']=collect_model_parameters(folder/'fresh_model')
            model=report['fresh_model']
            if (not model['identity_valid'] or not model['dh_valid'] or model['errors']
                    or [model['dh_response']['joint_'+str(i)] for i in range(1,7)]!=EXPECTED_DH):
                raise RuntimeError('Actual robot identity/DH differs')
            with (folder/'current_configuration_wire.jsonl').open('a',encoding='utf8') as configuration_wire:
                run_configuration=observe_configuration(configuration_wire)
            report['observed_run_configuration']=run_configuration;save()
            if model['identity_response']!=run_configuration['raw']['identity']:
                raise RuntimeError('Raw controller identity/firmware changed from current run configuration')
            with (folder/'current_configuration_wire.jsonl').open('a',encoding='utf8') as qualified_wire:
                report['initial_observed_raw_tool']=require_current_configuration(run_configuration,qualified_wire)
            wait_hand_ready(folder,report,save)
            authority=KnownReadAuthority();authority.accepted_identity_raw=report['startup_readiness']['final_identity_raw'][:]
            def hand_log(row):
                append(wire,row)
                if row.get('event')=='tx' and row.get('request',{}).get('command')==WRITE:
                    report['hand_register_writes']+=1;save()
            report['SDK_connection_attempted']=True;save()
            robot=connect(args.sdk_dir)
            profile=json.loads((ROOT/'deployment/battery_hand_verified_profile.json').read_bytes())
            seq=Alignment(robot,None,profile,report,save,samples,None,
                protection_client=ProtectionClient(lambda row:append(pwire,row)))
            try:
                metadata=collect_sdk_environment(robot);report['sdk_environment']=metadata
                if (not metadata['metadata_complete'] or metadata.get('api_version')!='4.3.1'
                        or metadata.get('loaded_native',{}).get('sha256')!=NATIVE_SHA
                        or metadata.get('controller_run_mode_raw',{}).get('data')!=[1]
                        or metadata.get('controller_software_info',{}).get('data',[{}])[0].get('product_version')!='RM65-BI'):
                    raise RuntimeError('SDK/native/actual real mode differs')
                initial=seq.full_arm();old=receipt['accepted_pregrasp_arm']
                require_reference_geometry(run_configuration,old['frame_signatures'])
                if initial['limits']!=old['limits']:
                    raise RuntimeError('Ordinary limits changed from accepted task source')
                seq.reference=copy.deepcopy(initial);seq.guard_target=initial['joints'][:];seq.guard_pose=initial['pose'][:]
                report['startup_reference']=copy.deepcopy(initial);report['reference_source']='fresh_current_read_not_old_or_fault_pose'
                seq.quiet(initial)
                low,high=seq.safety_reads();seq.drive_limits=dict(minimum=low,maximum=high)
                if report['arm_safety_reads']['Get_Install_Pose']!=[0,0,45,0]:raise RuntimeError('Install differs')
                report['tip_parameters']=collect(lambda row:append(twire,row))
                if (report['tip_parameters']['line_speed_m_s']>.25 or report['tip_parameters']['line_acc_m_s2']>1.6
                        or report['tip_parameters']['angular_speed_rad_s']>.6 or report['tip_parameters']['angular_acc_rad_s2']>4.):
                    raise RuntimeError('Actual end-effector settings exceed recorded bounds')
                if args.retract_only:
                    if not same_pose(initial,retreat_source['final_arm']) or initial['limits']!=retreat_source['final_arm']['limits']:
                        raise RuntimeError('Current arm no longer matches completed143310 hover segment; no retreat from stale source')
                    retreat=initial['pose'][:];retreat[2]=retreat_source['completed_hover_pose'][2]+.010
                    if not .009<=retreat[2]-initial['pose'][2]<=.011:
                        raise RuntimeError('Fixed vertical raise must be10mm, unchanged drift tolerances')
                    report['retreat_pose']=retreat[:]
                report['startup_height_observation']=dict(current_tcp_z_m=initial['pose'][2],hover_z_m=goal[2]+.140,
                    below_hover=initial['pose'][2]<goal[2]+.140,not_a_measured_clearance=True,
                    policy='fresh_empty_hand_vertical_raise_only_after_m_before_any_PREPARE_or_alignment')
                high_goal=goal[:];high_goal[2]=max(initial['pose'][2],goal[2]+.300)
                startup_raise=initial['pose'][:];startup_raise[2]=high_goal[2]
                report['startup_raise_pose']=startup_raise
                report['startup_raise_required']=startup_raise[2]-initial['pose'][2]>.0005
                hover=goal[:];hover[2]+=.140
                report.update(current_arm=initial,visual_pregrasp_pose=goal,high_alignment_pose=high_goal,
                    hover_pose=hover,prepare_target=PREPARE[:],high_and_downward_sweep_not_numerically_verified=True)
                hand=seq.open_hand_reader(full=True)
                start_hand=hand.preflight();report['startup_hand']=copy.deepcopy(start_hand)
                seq.io.finish()
                seq.close_hand();save()
                if args.retract_only:
                    seq.permission(1,'await_visual_clearance_retract',initial,'r')
                    seq.protections()
                    if not same_pose(seq.full_arm(),initial):raise RuntimeError('Retreat start changed after permission')
                    hand=seq.open_hand_reader()
                    hand.preflight(expected=PREPARE);seq.close_hand()
                    seq.startup_observation_only=False
                    current=seq.move(initial,retreat,'Movel_Cmd','vertical_empty_raise_from_completed_hover',2)
                    report.update(outcome='completed_vertical_clearance_raise_before_fresh_vision',phase='completed',
                        final_arm=current,clearance_retract_completed=True,execution_ready=False,no_grasp_or_lift=True)
                    save()
                else:
                    seq.permission(1,'await_visual_high_alignment',initial,'m')
                    seq.protections()
                    if not same_pose(seq.full_arm(),initial):raise RuntimeError('Current start changed after permission')
                    current=initial
                    report['raise_before_prepare']=True;save()
                    if report['startup_raise_required']:
                        current=seq.move(current,startup_raise,'Movel_Cmd','raising_current_empty_hand_before_alignment',2)
                    # No hand actuator write or lateral motion preceded this completed raise.
                    seq.reference=copy.deepcopy(current)
                    if not same_pose(seq.full_arm(),current):raise RuntimeError('Raised current endpoint changed before PREPARE')
                    hand=seq.open_hand_reader()
                    before=hand.preflight(expected=start_hand['targets'])
                    for ch,status in enumerate(before['status']):
                        if (status==5 or ch in seq.startup_stalled_channels) and (PREPARE[ch]>before['targets'][ch] or PREPARE[ch]>before['positions'][ch]+150):
                            raise RuntimeError('PREPARE would add closure to stalled channel')
                    if abs(before['targets'][5]-PREPARE[5])>2 or abs(before['positions'][5]-PREPARE[5])>150:
                        raise RuntimeError('Empty restart must first qualify retained opposition; no redundant rotation during PREPARE')
                    seq.authority.visual_prepare_permission=True
                    hand.guard();seq.io.authorize_prepare();seq.io.enable_motion();seq.hand_motion_enabled=True
                    report['phase']='preparing_empty_visual_hand';save()
                    seq.io.write_group(1135,PREPARE[:5])
                    hand.wait(PREPARE,tuple(range(5)),before['positions'])
                    seq.startup_observation_only=False;seq.hand_motion_enabled=False
                    hand.preflight(expected=PREPARE);seq.close_hand()
                    current=seq.move(current,high_goal,'Movej_P_Cmd','aligning_above_visual_target',5)
                    current=seq.move(current,hover,'Movel_Cmd','approaching_visual_hover_140mm',2)
                    report['hover_feedback_completed']=True;save()
                    seq.permission(2,'await_visual_final_descent',current,'d')
                    current=seq.move(current,goal,'Movel_Cmd','descending_to_visual_noncontact_pregrasp',2)
                    report.update(outcome='completed_visual_pregrasp_endpoint_feedback_pending_physical_verdict',phase='completed',
                        final_arm=current,pregrasp_endpoint_feedback_completed=True,execution_ready=False,
                        no_grasp_or_lift=True,new_target_physical_alignment_accepted=False)
                    save()
            except BaseException:
                if seq.motion is not None:
                    report['current_segment']=copy.deepcopy(seq.motion)
                try:seq.fault()
                except BaseException as exc:report['fault_handler_error']=repr(exc)
                if seq.motion is not None:
                    report['current_segment']=copy.deepcopy(seq.motion)
                save();raise
            finally:
                seq.close_hand()
    except BaseException as exc:
        report.update(outcome='aborted_no_further_writes',phase='fault',execution_ready=False)
        report['exceptions'].append(dict(error=repr(exc),traceback=traceback.format_exc()));save()
    finally:
        if robot is not None:
            for name in ('Arm_Socket_Close','RM_API_UnInit'):
                try:
                    answer=getattr(robot,name)()
                    report.setdefault('sdk_cleanup_results',[]).append(dict(method=name,raw_repr=repr(answer)))
                except BaseException as exc:report['cleanup_errors'].append(dict(method=name,error=repr(exc)))
        if report['cleanup_errors']:report['outcome']='failed_cleanup'
        save()
    complete=report.get('clearance_retract_completed') if args.retract_only else report.get('pregrasp_endpoint_feedback_completed')
    return 0 if complete and not report['exceptions'] and not report['cleanup_errors'] else 1


if __name__=='__main__':raise SystemExit(main())
