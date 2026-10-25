"""Actual SDK child for manual teaching: hand targets only, never arm motion."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import platform
import socket
import sys
import time
import traceback

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/battery_grasp/src'))
PREPARE=[8000,8000,8000,8000,0,64556]
SEED=[26500,30500,27000,19000]
CAPS=[30000,31500,29500,20250,65535,65535]
CHANNELS={'t':0,'i':1,'m':2,'r':3,'l':4,'o':5}


class UserCommandRejected(ValueError):
    """Only a user command rejected before a send, never a protocol fault."""


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--folder',type=Path,required=True)
    p.add_argument('--mode',choices=('prepare','tune','open','capture'),required=True)
    p.add_argument('--sdk-dir',type=Path,required=True)
    p.add_argument('--manifest-sha256',required=True)
    a=p.parse_args()
    if platform.system()!='Linux' or platform.machine()!='aarch64':p.error('Actual Linux aarch64 only')
    from known_cylinder_records import dump
    from run_grasp_teach_loop import verify_delivery,MANIFEST
    if hashlib.sha256(MANIFEST.read_bytes()).hexdigest()!=a.manifest_sha256:raise ValueError('Manifest changed')
    verify_delivery()
    f=a.folder;f.mkdir(parents=True,exist_ok=True);(f/'permissions').mkdir(exist_ok=True)
    report=dict(mode=a.mode,scope='manual_grasp_teaching_hand_only',phase='preflight',outcome='starting',
        arm_actuator_writes=0,arm_stop_writes=0,hand_register_writes=0,controller_configuration_writes=0,
        exceptions=[],cleanup_errors=[],execution_ready=False,automatic_arm_motion=False,
        automatic_lift=False,automatic_sort=False,physical_stop_verified=False,SDK_in_parent=False,
        offline_tests_executed=False)
    def save():dump(f/'worker_result.json',report)
    def append(out,row):out.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n');out.flush()
    robot=seq=None;save()
    try:
        from inspect_left_hand import connect
        from pregrasp_measurement_worker import source_audit,NATIVE_SHA
        from taught_model_readonly import collect_model_parameters
        from battery_grasp.taught_dh_model import EXPECTED_DH
        from battery_grasp.real_grasp import orientation_angle_rad
        from visual_run_configuration_v3 import observe_configuration,bind_or_require_frames,require_current_configuration
        from school_operational_readiness import wait_hand_ready
        from school_operational_reads import KnownReadHand,KnownReadAuthority
        from visual_startup_reads_v4 import StartupReadSession
        from supported_contact_transport import SupportedContactTransport,WritePhase
        from visual_pick_sort_v11_hand import ReadPacing,IdentityGuard
        from school_battery_transport import WRITE
        from run_arm_hand_sequence import Sequence
        from high_descent_prepare_worker import same_pose
        from sdk_environment import collect_sdk_environment
        from arm_protection_transport import ProtectionClient
        import fcntl

        class ExactTarget(ReadPacing,IdentityGuard,SupportedContactTransport):
            def grant(self,address,values):
                if self._attempted_writes or self.broken or self.terminal_latched or self._completed:
                    raise RuntimeError('Faulted/used hand capability')
                self.exact=(address,tuple(values));self.motion_enabled=True;self._phase=WritePhase.CONTACT
            def _validate(self,message):
                if message.get('command')!=WRITE:return super()._validate(message)
                if not getattr(self,'exact',None):raise RuntimeError('No exact operator hand command')
                address,values=self.exact
                expected=dict(command=WRITE,port=1,address=address,num=len(values),
                    data=[b for v in values for b in (v>>8,v&255)],device=2)
                if (message!=expected or self._attempted_writes or not self.motion_enabled
                        or self.broken or self.terminal_latched or self._completed
                        or any(type(message.get(k)) is not int for k in ('port','address','num','device'))
                        or any(type(v) is not int for v in message.get('data',[]))):
                    raise RuntimeError('Only one exact hand target per explicit command')
                self.exact=None;return address,values
            def set_phase(self,*unused,**options):raise RuntimeError('No reset/configuration phase')
            def enable_motion(self):raise RuntimeError('Use one exact operator target')

        class Hand(KnownReadHand):
            def check_status(self,row):
                self.sequence.blocked_closure_channels.update(ch for ch,s in enumerate(row['status']) if s==5)
                if any(s in (3,4) for s in row['status']):
                    self._protection('Current protection/force status blocks tuning: '+str(row['status']))

        class Current(Sequence):
            def call(self,name,*args,**kwargs):
                if not name.startswith('Get_'):raise RuntimeError('No arm motion/stop/configuration capability')
                return super().call(name,*args,**kwargs)
            def full_arm(self):
                arm=super().full_arm();bind_or_require_frames(run,arm['frame_signatures']);return arm
            def arm_state(self):
                if (f/'abort.json').exists():raise KeyboardInterrupt('Parent requested hand-work exit')
                q,pose=super().arm_state()
                if getattr(self,'drive_limits',None) and any(min(v-lo,hi-v)<3 for v,lo,hi in
                        zip(q,self.drive_limits[0],self.drive_limits[1])):
                    raise RuntimeError('Actual joint drive reserve below3degrees')
                return q,pose
            def close_hand(self):
                if self.io is not None:
                    io,self.io=self.io,None;self.hand_motion_enabled=False;io.disarm_motion()
                    if isinstance(io,StartupReadSession):io.close()
                    else:io.sock.close()
            def current_guard(self):
                q,pose=self.arm_state()
                if not same_pose(dict(self.reference,joints=q,pose=pose),self.reference):
                    raise RuntimeError('Arm changed during SDK hand work; exit before manual teaching')
            def reader(self,full=False):
                if self.io is not None:raise RuntimeError('One hand socket only')
                self.io=(StartupReadSession(f,report,save,hand_log,self.authority,self.current_guard) if full
                    else ExactTarget(socket.create_connection(('169.254.128.18',8080),timeout=3),hand_log,self.authority))
                hand=Hand(self,self.io,report,save,samples)
                if full:hand.configuration()
                else:hand._configuration_checked=True
                hand._critical_configuration();return hand
            def protections(self):
                self.close_hand()
                require_current_configuration(run,cwire)
                if self.call('Get_Collision_Stage')!=[0,8] or self.call('Get_Self_Collision_Enable')!=[0,True]:
                    raise RuntimeError('Protection must remain8/True; no setter')
            def fault(self):
                if getattr(self,'authority',None):self.authority._terminal_latched=True
                if self.io is not None:self.io.disable_motion()
                report['execution_ready']=False;save()

        def current_row(hand,expected,anchor=None):
            row=hand.snapshot()
            if any(abs(x-y)>2 for x,y in zip(row['targets'],expected)):
                raise RuntimeError('Hand target changed outside this teaching run')
            if anchor and any(abs(x-y)>150 for x,y in zip(row['positions'],anchor['positions'])):
                raise RuntimeError('Hand changed since last qualified observation')
            return row

        def perform(command,expected,anchor,index):
            # Resolve a complete exact signature before creating any write capability.
            if command in ('prepare','open') and command!=a.mode:
                raise UserCommandRejected('Preparation/opening requires its separate empty/supported phase')
            before=copy.deepcopy(anchor);new=expected[:]
            if command=='prepare':address=1135;values=PREPARE[:];new=values[:];channels=list(range(6));empty=True
            elif command=='open':address=1135;values=[0]*5;new[:5]=values;channels=list(range(5));empty=True
            elif command=='c':address=1135;values=SEED[:];new[:4]=values;channels=list(range(4));empty=False
            else:
                parts=command.split()
                if len(parts)!=2 or parts[0] not in CHANNELS or not parts[1].isdigit():
                    raise UserCommandRejected('Use c, x, or t/i/m/r/l/o plus an integer target')
                ch=CHANNELS[parts[0]];value=int(parts[1])
                if not 0<=value<=CAPS[ch]:raise UserCommandRejected('Target outside displayed candidate range')
                if ch==5 and (any(v>8000 for v in before['positions'][:5]+before['targets'][:5])):
                    raise UserCommandRejected('Opposition changes require five open fingers and empty clear rotation')
                address=1135+ch;values=[value];new[ch]=value;channels=[ch];empty=ch==5
            if any((before['status'][ch]==5 or ch in seq.blocked_closure_channels)
                    and new[ch]>min(before['targets'][ch],before['positions'][ch]+150)
                    for ch in channels):raise UserCommandRejected('Existing status5 forbids added closure; choose less flexion')
            if new==expected:
                if any(abs(before['positions'][ch]-new[ch])>150 for ch in channels):
                    raise RuntimeError('Existing target not reached; do not reissue it')
                report['last_command_result']=dict(command=command,writes=0,unchanged_target=True);save();return before
            seq.close_hand();seq.protections();seq.current_guard()
            old=seq.authority
            if old._terminal_latched:raise RuntimeError('Fault cannot acquire another hand capability')
            pacing=getattr(old,'read_pacing_qualified',False);old._completed=True
            seq.authority=KnownReadAuthority();seq.authority.accepted_identity_raw=before['identity'][:]
            seq.authority.read_pacing_qualified=pacing
            hand=seq.reader();fresh=current_row(hand,expected,before)
            if any(fresh['status'][ch]==5 and new[ch]>min(fresh['targets'][ch],fresh['positions'][ch]+150)
                    for ch in channels):raise RuntimeError('Newly observed status5; command cancelled')
            seq.io.grant(address,values);seq.hand_motion_enabled=True
            report.update(phase='hand_target_in_progress',command=command,permission_index=index,
                exact_hand_target=dict(address=address,values=values,unchanged_channels=[i for i in range(6) if i not in channels]))
            save();seq.io.write_group(address,values)
            began=time.monotonic();rows=[];since=None;previous=None
            minima=fresh['positions'][:];maxima=fresh['positions'][:]
            while time.monotonic()-began<30:
                row=hand.snapshot()
                if any(abs(x-y)>2 for x,y in zip(row['targets'],new)):raise RuntimeError('Written target differs')
                for ch in range(6):
                    pos=row['positions'][ch]
                    if ch not in channels and abs(pos-fresh['positions'][ch])>150:raise RuntimeError('Unwritten finger moved')
                    if ch in channels:
                        if not min(fresh['positions'][ch],new[ch])-150<=pos<=max(fresh['positions'][ch],new[ch])+150:
                            raise RuntimeError('Finger left actual-start/target movement envelope')
                        if (new[ch]<fresh['positions'][ch] and pos>minima[ch]+150
                                or new[ch]>fresh['positions'][ch] and pos<maxima[ch]-150):
                            raise RuntimeError('Finger reversed beyond150')
                    minima[ch]=min(minima[ch],pos);maxima[ch]=max(maxima[ch],pos)
                settled=previous is not None and all(abs(x-y)<=150 for x,y in zip(row['positions'],previous['positions']))
                reached=(not empty or all(abs(row['positions'][ch]-new[ch])<=150 for ch in channels))
                if settled and reached:
                    if since is None:since=time.monotonic()
                    rows.append(copy.deepcopy(row))
                    if len(rows)>=3 and time.monotonic()-since>=1:
                        if any(max(r['positions'][ch] for r in rows[-3:])-min(r['positions'][ch] for r in rows[-3:])>150
                                for ch in range(6)):
                            rows=[];since=None;previous=row;continue
                        hand._critical_configuration()
                        report['last_command_result']=dict(command=command,writes=1,targets=new,groups=rows[-3:],
                            contact_may_stop_short_of_target=not empty,holding_verified=False)
                        report.setdefault('hand_commands',[]).append(copy.deepcopy(report['last_command_result']))
                        seq.hand_motion_enabled=False;save();return row
                else:rows=[];since=None
                previous=row
            raise TimeoutError('Hand target observation not qualified; no retry or automatic opening')

        report['sdk_source_audit']=source_audit(a.sdk_dir)
        with (ROOT/'captures/hand_stage.lock').open('a') as lock, (f/'samples.jsonl').open('w',encoding='utf8') as samples, \
             (f/'hand_wire.jsonl').open('w',encoding='utf8') as wire,(f/'protection_wire.jsonl').open('w',encoding='utf8') as pwire, \
             (f/'configuration_wire.jsonl').open('w',encoding='utf8') as cwire:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            model=collect_model_parameters(f/'fresh_model');report['fresh_model']=model
            if (not model['identity_valid'] or not model['dh_valid'] or model['errors']
                    or [model['dh_response']['joint_'+str(i)] for i in range(1,7)]!=EXPECTED_DH):raise RuntimeError('Identity/DH differs')
            run=observe_configuration(cwire);report['observed_run_configuration']=run
            if model['identity_response']!=run['raw']['identity']:raise RuntimeError('Current controller identities disagree')
            wait_hand_ready(f,report,save)
            authority=KnownReadAuthority();authority.accepted_identity_raw=report['startup_readiness']['final_identity_raw'][:]
            def hand_log(row):
                append(wire,row)
                if row.get('event')=='tx' and row.get('request',{}).get('command')==WRITE:
                    report['hand_register_writes']+=1;save()
            robot=connect(a.sdk_dir)
            profile=json.loads((ROOT/'deployment/battery_hand_verified_profile.json').read_bytes())
            seq=Current(robot,None,profile,report,save,samples,None,
                protection_client=ProtectionClient(lambda row:append(pwire,row)))
            seq.authority=authority;seq.hand_motion_enabled=False;seq.startup_observation_only=True;seq.startup_stalled_channels=set()
            seq.blocked_closure_channels=set()
            metadata=collect_sdk_environment(robot);report['sdk_environment']=metadata
            if (not metadata['metadata_complete'] or metadata.get('api_version')!='4.3.1'
                    or metadata.get('loaded_native',{}).get('sha256')!=NATIVE_SHA
                    or metadata.get('controller_run_mode_raw',{}).get('data')!=[1]
                    or metadata.get('controller_software_info',{}).get('data',[{}])[0].get('product_version')!='RM65-BI'):
                raise RuntimeError('SDK/native/real controller identity differs')
            initial=seq.full_arm();seq.reference=copy.deepcopy(initial)
            seq.guard_target,seq.guard_pose=initial['joints'][:],initial['pose'][:]
            report['initial_arm']=copy.deepcopy(initial);report['reference_source']='fresh_this_child_current_state_not_fault_history'
            low,high=seq.safety_reads();seq.drive_limits=(low,high)
            if report['arm_safety_reads']['Get_Install_Pose']!=[0,0,45,0]:raise RuntimeError('Installation differs')
            previous=None
            for _ in range(5):
                seq.current_guard();q,pose=seq.arm_state()
                if previous is not None and (max(abs(x-y) for x,y in zip(q,previous[0]))>.02
                        or sum((x-y)**2 for x,y in zip(pose[:3],previous[1][:3]))**.5>.0002
                        or orientation_angle_rad(pose[3:],previous[1][3:])>.002):
                    raise RuntimeError('Current arm not quiet after leaving teaching mode')
                previous=q,pose;time.sleep(.25)
            seq.protections();hand=seq.reader(full=True);row=hand.preflight()
            seq.io.finish();seq.close_hand();seq.startup_observation_only=False
            report['initial_hand']=copy.deepcopy(row);save()
            active=row['targets'][:]
            if a.mode=='capture':
                report.update(final_arm=seq.full_arm(),final_hand=row,outcome='recorded_current_arm_hand_readonly',phase='completed')
            elif a.mode in ('prepare','open'):
                row=perform(a.mode,active,row,0)
                report.update(final_arm=seq.full_arm(),final_hand=row,outcome='completed_'+a.mode,phase='completed')
            else:
                if (any(v>8000 for v in active[:5])
                        or any(abs(x-y)>150 for x,y in zip(row['positions'],active))):
                    raise RuntimeError('Expected this program PREPARE changed; restart preparation, no stale recipe')
                report['pregrasp_arm']=copy.deepcopy(initial);report['pregrasp_hand']=copy.deepcopy(row)
                # Qualify the optional100ms read gap in this actual quiet empty-hand run only.
                hand=seq.reader();hand._critical_configuration();before=current_row(hand,active,row)
                seq.io._pacing_probe=True;probe=[]
                try:
                    for _ in range(3):probe.append(current_row(hand,active,before))
                finally:seq.io._pacing_probe=False
                after=current_row(hand,active,before)
                if any(max(r['positions'][ch] for r in probe+[before,after])-min(r['positions'][ch] for r in probe+[before,after])>150 for ch in range(6)):
                    raise RuntimeError('Readonly pacing qualification failed')
                seq.authority.read_pacing_qualified=True;report['read_pacing_qualification']=dict(passed=True,groups=probe,
                    read_gap_seconds=.100,write_boundary_seconds=.300,manufacturer_minimum_claimed=False)
                row=after;save()
                for index in range(1,301):
                    if seq.io is None:hand=seq.reader()
                    else:hand=Hand(seq,seq.io,report,save,samples);hand._configuration_checked=True
                    report.update(phase='await_hand_command',permission_index=index,feedback=copy.deepcopy(row));save()
                    path=f/'permissions'/('%d.json'%index);began=time.monotonic();event=None
                    held_anchor=copy.deepcopy(row)
                    while time.monotonic()-began<600:
                        if (f/'abort.json').exists():raise KeyboardInterrupt('Parent cancelled')
                        row=current_row(hand,active,held_anchor)
                        if path.exists():
                            event=json.loads(path.read_bytes())
                            if (event.get('index')!=index
                                    or event.get('only_intended_shell_contact_and_digit_sweep_clear') is not True):
                                raise RuntimeError('Stale/unconfirmed hand command')
                            break
                    if event is None:raise TimeoutError('Hand input expired')
                    command=event['command'];report.setdefault('operator_commands',[]).append(event);save()
                    if command.startswith('o ') and event.get('opposition_command_confirms_empty_and_rotation_clear') is not True:
                        raise RuntimeError('Empty clear opposition rotation not confirmed')
                    if command=='q':raise KeyboardInterrupt('Operator exit without automatic opening')
                    if command=='x':
                        rows=[current_row(hand,active,row) for _ in range(3)]
                        if any(max(r['positions'][ch] for r in rows)-min(r['positions'][ch] for r in rows)>150
                                for ch in range(6)):
                            raise RuntimeError('Hand not quiet before manual lift handoff')
                        report.update(final_arm=seq.full_arm(),final_hand=rows[-1],tuned_targets=active[:],
                            phase='completed',outcome='ready_for_manual_teach_lift',manual_lift_performed=False,
                            grip_feedback_stable=True,holding_verified=False);save();break
                    try:row=perform(command,active,row,index);active=row['targets'][:]
                    except UserCommandRejected as exc:
                        report['last_command_result']=dict(command=command,rejected_before_write=True,reason=str(exc));save()
                else:raise RuntimeError('Command count exhausted')
            if not same_pose(seq.full_arm(),initial):raise RuntimeError('Arm changed during hand/read capture')
    except BaseException as exc:
        if seq is not None:
            try:seq.fault()
            except BaseException as fault:report['cleanup_errors'].append(dict(method='fault',error=repr(fault)))
        report.update(phase='fault',outcome='aborted_no_automatic_recovery')
        report['exceptions'].append(dict(error=repr(exc),traceback=traceback.format_exc()))
    finally:
        if seq is not None:
            try:seq.close_hand()
            except BaseException as exc:report['cleanup_errors'].append(dict(method='hand.close',error=repr(exc)))
        if robot is not None:
            for name in ('Arm_Socket_Close','RM_API_UnInit'):
                try:
                    result=getattr(robot,name)();report.setdefault('sdk_cleanup_results',[]).append(dict(method=name,raw_repr=repr(result)))
                    if name=='RM_API_UnInit' and (type(result) is not int or result!=0):raise RuntimeError('SDK uninit not confirmed')
                except BaseException as exc:report['cleanup_errors'].append(dict(method=name,error=repr(exc)))
        if report['cleanup_errors']:report['outcome']='failed_cleanup'
        report['execution_ready']=False;report['SDK_cleanup_finished']=not report['cleanup_errors'];save()
    return 0 if report['phase']=='completed' and not report['exceptions'] and not report['cleanup_errors'] else 1


if __name__=='__main__':raise SystemExit(main())
