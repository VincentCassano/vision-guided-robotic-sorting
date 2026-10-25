"""Single native lifecycle, stationary arm, per-key single-channel grip targets."""
import argparse
import copy
from datetime import datetime
import fcntl
import hashlib
import json
import math
from pathlib import Path
import socket
import sys
import time
import traceback
from known_cylinder_records import dump

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/battery_grasp/src'))
STEPS={0:1000,3:250}
CAPS={0:30000,3:22000}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--folder',required=True,type=Path)
    p.add_argument('--source-report',required=True,type=Path)
    p.add_argument('--sdk-dir',required=True,type=Path)
    p.add_argument('--recover-ring-contact',action='store_true')
    a=p.parse_args();folder=a.folder
    report=dict(scope='new_explicit_supported_hand_adjustment_not_fault_authority_resume',
        phase='preflight',arm_actuator_writes=0,hand_register_writes=0,configuration_writes=0,
        automatic_fault_release=False,source_fault_last_arm_state_used=False,
        source_previous_loose_grip_report=True,grip_holding_verified=False,execution_ready=False,
        physical_stop_verified=False,torque_release_verified=False,exceptions=[])
    robot=seq=None
    def save():dump(folder/'worker_result.json',report)
    save()
    try:
        from inspect_left_hand import connect
        from run_arm_hand_sequence import Sequence
        from high_descent_prepare_worker import same_pose
        from arm_protection_transport import ProtectionClient
        from supported_contact_transport import SupportedContactTransport,PhaseAuthority
        from school_battery_transport import WRITE
        from school_operational_pose_hand import CycleHand
        from school_operational_hand import TrialHand
        from school_device_identity import identity_fields,same_live_identity
        from battery_grasp.real_grasp import orientation_angle_rad
        from sdk_environment import collect_sdk_environment
        from pregrasp_measurement_worker import NATIVE_SHA
        source_path=a.source_report
        raw=source_path.read_bytes();source=json.loads(raw)
        if not a.recover_ring_contact or hashlib.sha256(raw).hexdigest()!='ce16acb55c7f250fbf5b45f9d94c2f27f71f3bdcfae48aac92d9b26dd6c67d37':
            raise ValueError('Explicit reviewed075643 supported ring withdrawal required')
        if (source.get('outcome')!='aborted_no_further_writes'
                or source.get('arm_actuator_writes')!=0 or source.get('hand_register_writes')!=2
                or source.get('configuration') is None or source.get('cleanup_errors')):
            raise ValueError('Unexpected075643 fault provenance')
        if (source.get('last_snapshot',{}).get('status')!=[2,2,2,3,5,2]
                or source.get('last_snapshot',{}).get('targets')!=[17500,30000,28000,22499,65535,65222]
                or len(source.get('moves',[]))!=1 or source['moves'][0]['key']!='t'):
            raise ValueError('Reviewed ring protection or thumb success differs')
        anchor_raw=(ROOT/'deployment/grip_tune_source_024412.json').read_bytes()
        if hashlib.sha256(anchor_raw).hexdigest()!='d69a9e0c91d4aa34b68ab8d7ae75600d14f147135fec1b7ce7c5d36b9b0e3372':
            raise ValueError('Original qualified successful descent record differs')
        anchor=json.loads(anchor_raw);segments=anchor.get('arm_segments',[])
        if len(segments)!=2 or segments[-1].get('phase')!='descending_to_supported_shell':
            raise ValueError('Completed descent endpoint provenance missing')
        reference=copy.deepcopy(segments[-1]['endpoint'])
        if reference.get('arm_error')!=0 or reference.get('system_error')!=0:raise ValueError('Rejected source endpoint')
        profile=json.loads((ROOT/'deployment/battery_hand_verified_profile.json').read_bytes())
        if source['configuration']!=profile['observed_configuration']:raise ValueError('Source full hand profile differs')
        # Actual acknowledged TX was22500; device target22499 is within the
        # existing2-unit readback tolerance. The stale report20000 is not used.
        active=[17500,30000,28000,22500,65535,65222]
        recovering=True;existing_three=None;opening_attempted=False
        report.update(source_worker_sha256=hashlib.sha256(raw).hexdigest(),qualified_reference=reference,
            reference_source='completed_descent_endpoint_not_last_fault_state',active_targets=active[:],
            source_previous_target_verified_by_fresh_reads=False,blocked_closure_channels=[],moves=[])
        def append(stream,row):stream.write(json.dumps(row,ensure_ascii=False)+'\n');stream.flush()
        with (ROOT/'captures/hand_stage.lock').open('a') as lock, \
             (folder/'hand_wire.jsonl').open('w',encoding='utf8') as wire, \
             (folder/'protection_wire.jsonl').open('w',encoding='utf8') as pwire, \
             (folder/'samples.jsonl').open('w',encoding='utf8') as samples:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            token=PhaseAuthority();token.accepted_identity_raw=source['last_snapshot']['identity'][:]
            class Stationary(Sequence):
                def call(self,name,*args,**kwargs):
                    if not name.startswith('Get_'):raise RuntimeError('Stationary entry denies every arm write')
                    if name in ('Get_Collision_Stage','Get_Self_Collision_Enable') and self.io is not None:
                        raise RuntimeError('Raw protection GET while hand socket open is forbidden')
                    return super().call(name,*args,**kwargs)
                def arm_state(self):
                    q,pose=super().arm_state()
                    if (max(abs(x-y) for x,y in zip(q,self.guard_target))>.08
                            or math.dist(pose[:3],self.guard_pose[:3])>.0005
                            or orientation_angle_rad(pose[3:],self.guard_pose[3:])>.005):
                        raise RuntimeError('Arm drifted from qualified stationary reference')
                    drives=getattr(self,'tune_drive_limits',None)
                    if drives is not None and any(min(v-low,high-v)<3 for v,low,high in zip(q,*drives)):
                        raise RuntimeError('Fresh drive reserve below3degrees')
                    return q,pose
                def close_hand(self):
                    if self.io is not None:
                        io,self.io=self.io,None;io.disarm_motion();io.sock.close()
            class TuneTransport(SupportedContactTransport):
                def __init__(self,*args,**kwargs):
                    super().__init__(*args,**kwargs);self.approved=None;self.fast=False
                def pre_send_quiet_seconds(self,message):
                    return .3
                def _validate(self,message):
                    if message.get('command')!=WRITE:return super()._validate(message)
                    data=message.get('data',[])
                    if (set(message)!={'command','port','device','address','num','data'} or message.get('port')!=1
                            or message.get('device')!=2 or not data or len(data)!=2*message.get('num',0)
                            or any(type(message.get(k)) is not int for k in ('port','device','address','num'))
                            or any(type(v) is not int or not 0<=v<=255 for v in data)):
                        raise ValueError('Invalid narrow hand adjustment request')
                    signature=(message['address'],tuple(data[i]*256+data[i+1] for i in range(0,len(data),2)))
                    address,values=signature
                    bounded=(signature==(1138,(21250,)) if recovering else
                        (len(values)==1 and address in (1135,1138) and 8000<=values[0]<=CAPS[address-1135]) or signature==(1135,(8000,)*4))
                    if (not bounded or signature!=self.approved or self.terminal_latched or self.broken
                            or not self.motion_enabled or signature in self._attempted_writes):
                        raise RuntimeError('Hand write lacks this-key bounded permission; no retry')
                    self.approved=None
                    return signature
                def grant(self,address,values):
                    if self.terminal_latched or self.broken or self.approved is not None:raise RuntimeError('Invalid new write permission')
                    self.approved=(address,tuple(values));self.motion_enabled=True
                def set_phase(self,*args,**kwargs):raise RuntimeError('No other hand write phase')
                def enable_motion(self):raise RuntimeError('Unbounded hand enable forbidden')
            class TuneHand(CycleHand):
                def read_feedback_group(self):return TrialHand.read_feedback_group(self)
                def _identity_checked(self,raw):
                    identity_fields(raw)
                    if not same_live_identity(raw,token.accepted_identity_raw):raise RuntimeError('New hand diagnostic after audited fault')
                    token.accepted_identity_raw=raw[:]
                def check_targets(self,row,expected):
                    if any(abs(x-y)>2 for x,y in zip(row['targets'],expected)):raise RuntimeError('External target change')
                    for ch in range(6):
                        upper=max(CAPS.get(ch,active[ch]),active[ch],22500 if recovering and ch==3 else 0)+150
                        if not 0<=row['positions'][ch]<=upper:raise RuntimeError('Finger outside accepted envelope')
                def check_status(self,row):
                    nonlocal existing_three
                    if not recovering:return super().check_status(row)
                    channels={ch for ch,v in enumerate(row['status']) if v==3}
                    if 4 in row['status'] or not channels<={3}:
                        self._protection('Only reviewed ring3 is permitted for supported withdrawal')
                    if existing_three is not None and not channels<=existing_three:
                        self._protection('New/reappeared protection3 during withdrawal')
                    existing_three=channels
                    self.report.setdefault('ring_recovery_status',[]).append(dict(status=row['status'][:],
                        opening_attempted=opening_attempted,protection_clear_command_sent=False))
                def guard(self):seq.arm_state()
            robot=connect(a.sdk_dir)
            seq=Stationary(robot,reference,profile,report,save,samples,None,
                protection_client=ProtectionClient(lambda row:append(pwire,row)))
            seq.guard_target=reference['joints'][:];seq.guard_pose=reference['pose'][:]
            metadata=collect_sdk_environment(robot);report['sdk_environment']=metadata
            if (not metadata['metadata_complete'] or metadata.get('api_version')!='4.3.1'
                    or metadata.get('loaded_native',{}).get('sha256')!=NATIVE_SHA
                    or metadata.get('controller_run_mode_raw',{}).get('data')!=[1]):raise RuntimeError('SDK/current real mode differs')
            if not same_pose(seq.full_arm(),reference):raise RuntimeError('Arm moved or frames/limits changed since completed descent')
            seq.tune_drive_limits=seq.safety_reads()
            seq.arm_state()
            if report['fresh_protection_values'].get('Get_Collision_Stage')!=[0,8] or report['fresh_protection_values'].get('Get_Self_Collision_Enable')!=[0,True]:
                raise RuntimeError('Current protection must already be8/True; no setters')
            def open_hand(full=False):
                seq.io=TuneTransport(socket.create_connection(('169.254.128.18',8080),timeout=3),lambda row:append(wire,row),token)
                hand=TuneHand(seq,seq.io,report,save,samples)
                if full:hand.configuration()
                else:hand._configuration_checked=True
                hand._critical_configuration();row=hand.snapshot();hand.check_targets(row,active)
                for ch,(pos,target) in enumerate(zip(row['positions'],active)):
                    if recovering and ch==3:
                        if not 21250<=pos<=22500+150:raise RuntimeError('Ring outside reviewed withdrawal range')
                    elif abs(pos-target)>150:raise RuntimeError('Current hand not settled at recorded targets')
                return hand,row
            hand,row=open_hand(full=True)
            # Keep the existing300ms baseline while a shell is between the
            # fingers. Do not inherit timing qualifications or probe new pacing.
            report['read_pacing']=dict(seconds=.3,new_timing_probe=False)
            report['source_previous_target_verified_by_fresh_reads']=True
            blocked=set(hand._closure_blocked_channels)
            if recovering:
                second=hand.snapshot();hand.check_targets(second,active)
                if any(abs(x-y)>150 for x,y in zip(row['positions'],second['positions'])):
                    raise RuntimeError('Supported recovery requires quiet current groups')
                report['ring_recovery_before']=second
            seq.close_hand();index=0
            while True:
                phase='await_ring_withdrawal' if recovering else 'await_grip_tune'
                report.update(phase=phase,permission_index=index,active_targets=active[:],blocked_closure_channels=sorted(blocked));save()
                path=folder/'permissions'/('%d.json'%index);began=time.monotonic()
                while not path.exists():
                    if (folder/'abort.json').exists():raise KeyboardInterrupt('Parent abort')
                    if time.monotonic()-began>570:raise TimeoutError('Manual adjustment permission expired')
                    seq.arm_state();time.sleep(.5)
                event=json.loads(path.read_text(encoding='utf8'))
                if event.get('index')!=index or event.get('phase')!=phase:raise RuntimeError('Stale input permission')
                key=event['value'];index+=1
                if key!='q' and not all(event.get(k) is True for k in
                        ('shell_fully_supported','hand_swept_volume_clear','onsite_stop_ready')):
                    raise RuntimeError('Explicit supported adjustment conditions missing')
                if recovering and key not in ('r','q'):raise RuntimeError('Only explicit withdrawal or quit before healthy feedback')
                if key in ('s','q'):
                    if key=='s':
                        if not same_pose(seq.full_arm(),reference):raise RuntimeError('Arm changed before recipe confirmation')
                        if seq.call('Get_Collision_Stage')!=[0,8] or seq.call('Get_Self_Collision_Enable')!=[0,True]:raise RuntimeError('Protection changed')
                        hand,final=open_hand();report['final_confirmed_feedback']=final;seq.close_hand()
                    report.update(outcome='supported_grip_recipe_recorded_no_lift' if key=='s' else 'left_current_pose_no_lift',
                        confirmed_recipe=active[:],operator_reported_contact_fit=(key=='s'));break
                if key not in ('t','f','o','r'):raise ValueError('Unexpected input')
                ch=0 if key=='t' else 3
                if key in ('t','f') and (ch in blocked or active[ch]>=CAPS[ch]):
                    report['last_adjustment_refused']='Channel at contact state5 or bounded maximum';save();continue
                if not same_pose(seq.full_arm(),reference):raise RuntimeError('Fresh pose changed; no hand write')
                # Every raw protection query finishes with the hand socket closed.
                if seq.call('Get_Collision_Stage')!=[0,8] or seq.call('Get_Self_Collision_Enable')!=[0,True]:raise RuntimeError('Protection changed')
                hand,before=open_hand()
                address=1135 if key=='o' else 1135+ch
                values=([8000]*4 if key=='o' else [21250] if key=='r'
                    else [min(active[ch]+STEPS[ch],CAPS[ch])])
                if key=='o' and any(v<8000 for v in before['positions'][:4]):raise RuntimeError('PREPARE opening would add closure to an actual finger')
                if key=='r' and values[0]>min(before['positions'][3],before['targets'][3]):raise RuntimeError('Recovery would add ring closure')
                if key in ('t','f') and before['status'][ch]==5:
                    blocked.add(ch);seq.close_hand();continue
                hand.guard()
                operation=dict(key=key,address=address,values=values[:],before=before,
                    attempted=True,acknowledged=False,feedback_accepted=False)
                report.setdefault('command_attempts',[]).append(operation)
                seq.io.grant(address,values);report['hand_register_writes']+=1;save()
                if key=='r':opening_attempted=True
                seq.io.write_group(address,values)
                operation['acknowledged']=True
                if key=='o':active[:4]=values
                else:active[ch]=values[0]
                report['active_targets']=active[:];report['last_acknowledged_targets']=active[:];save()
                frozen=False;stable=0;quiet=None;low=high=None;started=time.monotonic()
                while time.monotonic()-started<20:
                    row=hand.snapshot();hand.check_targets(row,active)
                    if key in ('t','f') and row['status'][ch]==5:
                        blocked.add(ch)
                        if not frozen and row['positions'][ch]<active[ch]-2:
                            value=min(active[ch],row['positions'][ch])
                            if value<8000:raise RuntimeError('Contact earlier than withdrawal-only range')
                            fresh_status=hand.read('status_before_contact_reduction',1085,6)
                            if any(v not in (0,1,2,5) for v in fresh_status):raise RuntimeError('Protection before contact target reduction')
                            hand.guard()
                            reduction=dict(key='contact_reduction',address=1135+ch,values=[value],
                                attempted=True,acknowledged=False,feedback_accepted=False)
                            report['command_attempts'].append(reduction)
                            seq.io.grant(1135+ch,[value]);report['hand_register_writes']+=1;save();seq.io.write_group(1135+ch,[value])
                            reduction['acknowledged']=True;active[ch]=value
                            report['active_targets']=active[:];report['last_acknowledged_targets']=active[:];save()
                            frozen=True;stable=0;quiet=None;low=high=None;continue
                    inactive=range(4,6) if key=='o' else [i for i in range(6) if i!=ch]
                    if any(abs(row['positions'][i]-before['positions'][i])>150 for i in inactive):raise RuntimeError('Uncommanded finger drift')
                    if key=='r' and not 21250-150<=row['positions'][3]<=before['positions'][3]+150:
                        raise RuntimeError('Ring left withdrawal-only sweep')
                    pos=row['positions']
                    if low is None:low=pos[:];high=pos[:]
                    low=[min(x,y) for x,y in zip(low,pos)];high=[max(x,y) for x,y in zip(high,pos)]
                    healthy=not any(v in (3,4) for v in row['status'])
                    if healthy and max(y-x for x,y in zip(low,high))<=150 and all(abs(v-t)<=150 for v,t in zip(pos,active)):
                        if quiet is None:quiet=time.monotonic()
                        stable+=1
                        if stable>=3 and time.monotonic()-quiet>=1:break
                    else:low=pos[:];high=pos[:];stable=0;quiet=None
                else:raise TimeoutError('Adjustment feedback did not settle; no retry')
                report['moves'].append(dict(key=key,address=address,values=values,active_targets=active[:],feedback=row,
                    contact_target_reduction=frozen,calibrated_force_or_holding_verified=False));save()
                operation['feedback_accepted']=True
                save()
                if key=='r':
                    # Retire the withdrawal capability; a distinct normal
                    # token follows only the ACK and three healthy groups.
                    token._completed=True
                    previous_identity=token.accepted_identity_raw[:]
                    seq.close_hand();token=PhaseAuthority();token.accepted_identity_raw=previous_identity
                    recovering=False
                    report['recovery_completed']=dict(single_ring_withdrawal_to21250=True,
                        ACK_and_three_healthy_groups=True,normal_token_is_new=True,
                        old_fault_latch_reset=False,protection_clear_command_sent=False)
                seq.close_hand()
                if key=='o':report['outcome']='supported_PREPARE_opened_no_arm_motion';break
            report['phase']='completed';save()
    except BaseException as exc:
        report.update(outcome='aborted_no_further_writes',phase='fault',error=repr(exc),traceback=traceback.format_exc());save()
    finally:
        if seq is not None:
            try:seq.close_hand()
            except Exception as exc:report.setdefault('cleanup_errors',[]).append(repr(exc))
        if robot is not None:
            for name in ('Arm_Socket_Close','RM_API_UnInit'):
                try:getattr(robot,name)()
                except Exception as exc:report.setdefault('cleanup_errors',[]).append(repr(exc))
        save()
    return 0 if report.get('phase')=='completed' and not report.get('cleanup_errors') else 1


if __name__=='__main__':raise SystemExit(main())
