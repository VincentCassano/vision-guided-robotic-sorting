"""New fixed-support grip session; explicit new low-speed lift only after regrip."""
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
STEPS={0:500,1:500,2:500,3:250}
CAPS={0:30000,1:40000,2:35000,3:22000}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--folder',required=True,type=Path)
    p.add_argument('--source-report',required=True,type=Path)
    p.add_argument('--sdk-dir',required=True,type=Path)
    a=p.parse_args();folder=a.folder
    report=dict(scope='new_explicit_supported_hand_adjustment_not_fault_authority_resume',
        phase='preflight',arm_actuator_writes=0,hand_register_writes=0,configuration_writes=0,
        automatic_fault_release=False,source_fault_last_arm_state_used=False,
        user_reported_shell_not_lifted=True,grip_holding_verified=False,execution_ready=False,
        physical_stop_verified=False,torque_release_verified=False,exceptions=[])
    robot=seq=None
    def save():dump(folder/'worker_result.json',report)
    save()
    try:
        from inspect_left_hand import connect
        from school_bin_cycle_v4_sequence import SchoolBinCycle
        from tuned_grip_bin_hand import TunedHand,HeldReadTransport,NormalDropTransport
        from high_descent_prepare_worker import same_pose
        from arm_protection_transport import ProtectionClient
        from supported_contact_transport import SupportedContactTransport,PhaseAuthority
        from school_battery_transport import WRITE
        from school_operational_pose_hand import CycleHand
        from school_operational_hand import TrialHand
        from school_device_identity import identity_fields,same_live_identity
        from battery_grasp.real_grasp import orientation_angle_rad
        from sdk_environment import collect_sdk_environment
        from pregrasp_measurement_worker import NATIVE_SHA,source_audit
        from taught_model_readonly import collect_model_parameters
        from battery_grasp.taught_dh_model import EXPECTED_DH
        raw=a.source_report.read_bytes();source=json.loads(raw)
        if hashlib.sha256(raw).hexdigest()!='690cb64f3ef4476965b6cbfc613638010d1c1762e0c2ceaaf6b007a80c63e7a9':
            raise RuntimeError('Reviewed082602 fault provenance bytes required')
        if (source.get('arm_actuator_writes')!=1 or source.get('hand_register_writes')!=0
                or source.get('outcome')!='aborted_no_further_writes'
                or len(source.get('exceptions',[]))!=1
                or 'Loaded lift left this run fixed-wrist vertical corridor' not in source['exceptions'][0]['error']):
            raise RuntimeError('Unexpected source event; no hand adjustment')
        profile=json.loads((ROOT/'deployment/battery_hand_verified_profile.json').read_bytes())
        from school_device_identity import configuration_matches
        if not configuration_matches(source['configuration'],profile['observed_configuration']):
            raise RuntimeError('Source full device settings differ')
        reference=copy.deepcopy(source['qualified_success_reference'])
        active=[25500,30000,28000,21250,65535,65222]
        report.update(source_fault_sha256=hashlib.sha256(raw).hexdigest(),active_targets=active[:],
            reference_source='this_run_fresh_stationary_hand_only_qualification_not_fault_last_state',
            rejected_fault_pose_used_for_motion=False,blocked_closure_channels=[],moves=[],
            delivery_mode=False,selected_color='black',no_automatic_fault_resumption=True,
            user_reported_partial_lift_and_tilt=True,
            full_fixed_support_required_before_hand_work=True)
        def append(stream,row):stream.write(json.dumps(row,ensure_ascii=False)+'\n');stream.flush()
        with (ROOT/'captures/hand_stage.lock').open('a') as lock, \
             (folder/'hand_wire.jsonl').open('w',encoding='utf8') as wire, \
             (folder/'protection_wire.jsonl').open('w',encoding='utf8') as pwire, \
             (folder/'samples.jsonl').open('w',encoding='utf8') as samples:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            report['source_audit']=source_audit(a.sdk_dir)
            model=collect_model_parameters(folder/'fresh_model');report['fresh_model']=model;save()
            if (not model['dh_valid'] or not model['identity_valid'] or model['errors']
                    or [model['dh_response']['joint_'+str(i)] for i in range(1,7)]!=EXPECTED_DH):
                raise RuntimeError('Fresh actual identity/DH differs; no settings write')
            token=PhaseAuthority();token.accepted_identity_raw=source['last_loaded_preflight']['identity'][:]
            class Stationary(SchoolBinCycle):
                delivery_mode=False
                def wait_input(self,index,phase,arm,values,teaching=False):
                    if index==8:
                        event=copy.deepcopy(self.new_lift_permission)
                        event.update(index=8,phase=phase)
                        self.report.setdefault('operator_inputs',[]).append(event)
                        self.report['lift_permission_origin']='explicit_l_in_this_run_tuning_menu'
                        self.save();return event
                    return super().wait_input(index,phase,arm,values,teaching)

                def call(self,name,*args,**kwargs):
                    if not name.startswith('Get_') and not (self.delivery_mode and name in ('Movel_Cmd','Movej_P_Cmd','Move_Stop_Cmd')):
                        raise RuntimeError('Hand-only stage denies every arm write')
                    if name in ('Get_Collision_Stage','Get_Self_Collision_Enable') and self.io is not None:
                        raise RuntimeError('Raw protection GET while hand socket open is forbidden')
                    return super().call(name,*args,**kwargs)
                def arm_state(self):
                    q,pose=super().arm_state()
                    if not self.delivery_mode and (max(abs(x-y) for x,y in zip(q,self.guard_target))>.08
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
                    bounded=(len(values)==1 and address in (1135,1136,1137,1138) and 8000<=values[0]<=CAPS[address-1135]) or signature==(1135,(8000,)*4)
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
                        upper=CAPS.get(ch,active[ch])+150
                        if not 0<=row['positions'][ch]<=upper:raise RuntimeError('Finger outside accepted envelope')
                def guard(self):seq.arm_state()
            robot=connect(a.sdk_dir)
            seq=Stationary(robot,reference,profile,report,save,samples,None,
                folder=folder,authority=token,tip_reader=None,
                protection_client=ProtectionClient(lambda row:append(pwire,row)),
                enable_arm_protection=False,taught_only=True,resume_report=None,completed_corridor=None,
                allow_protected_opening=False)
            seq.guard_target=reference['joints'][:];seq.guard_pose=reference['pose'][:]
            metadata=collect_sdk_environment(robot);report['sdk_environment']=metadata
            if (not metadata['metadata_complete'] or metadata.get('api_version')!='4.3.1'
                    or metadata.get('loaded_native',{}).get('sha256')!=NATIVE_SHA
                    or metadata.get('controller_run_mode_raw',{}).get('data')!=[1]):raise RuntimeError('SDK/current real mode differs')
            current=seq.full_arm()
            if current['frame_signatures']!=reference['frame_signatures'] or current['limits']!=reference['limits']:
                raise RuntimeError('Current frames/limits differ; no new stationary trial')
            # This fresh reference grants only stationary HAND work. It is not
            # copied from the fault last pose and grants no resume/up command.
            reference=copy.deepcopy(current);seq.reference=reference
            seq.guard_target=current['joints'][:];seq.guard_pose=current['pose'][:]
            report['fresh_hand_only_reference']=copy.deepcopy(current)
            quiet_start=time.monotonic();prior=None;quiet_count=0
            while quiet_count<5 or time.monotonic()-quiet_start<1:
                q,p=seq.arm_state()
                if prior is not None and (max(abs(a-b) for a,b in zip(q,prior[0]))>.02
                        or math.dist(p[:3],prior[1][:3])>.0002
                        or orientation_angle_rad(p[3:],prior[1][3:])>.002):
                    raise RuntimeError('Arm not quiet for independent stationary trial')
                prior=(q,p);quiet_count+=1;time.sleep(.25)
            report['fresh_hand_only_stationary_observation']=dict(groups=quiet_count,seconds=time.monotonic()-quiet_start,
                physical_stop_verified=False,arm_motion_authorized=False)

            seq.tune_drive_limits=seq.safety_reads()
            seq.arm_state()
            if report['fresh_protection_values'].get('Get_Collision_Stage')!=[0,8] or report['fresh_protection_values'].get('Get_Self_Collision_Enable')!=[0,True]:
                raise RuntimeError('Current protection must already be8/True; no setters')
            install=report['arm_safety_reads']['Get_Install_Pose']
            if (not isinstance(install,list) or len(install)!=4 or install[0]!=0
                    or any(abs(a-b)>.001 for a,b in zip(install[1:],[0,45,0]))):
                raise RuntimeError('Current installation differs from accepted mounting')
            def open_hand(full=False):
                seq.io=TuneTransport(socket.create_connection(('169.254.128.18',8080),timeout=3),lambda row:append(wire,row),token)
                hand=TuneHand(seq,seq.io,report,save,samples)
                if full:hand.configuration()
                else:hand._configuration_checked=True
                hand._critical_configuration();row=hand.snapshot();hand.check_targets(row,active)
                if any(abs(x-y)>150 for x,y in zip(row['positions'],active)):raise RuntimeError('Current hand not settled at recorded targets')
                return hand,row
            hand,row=open_hand(full=True)
            # Keep the existing300ms baseline while a shell is between the
            # fingers. Do not inherit timing qualifications or probe new pacing.
            report['read_pacing']=dict(seconds=.3,new_timing_probe=False)
            report['source_previous_target_verified_by_fresh_reads']=True
            blocked=set(hand._closure_blocked_channels)
            seq.close_hand();index=1000
            while True:
                report.update(phase='await_grip_tune',permission_index=index,active_targets=active[:],blocked_closure_channels=sorted(blocked));save()
                path=folder/'permissions'/('%d.json'%index);began=time.monotonic()
                while not path.exists():
                    if (folder/'abort.json').exists():raise KeyboardInterrupt('Parent abort')
                    if time.monotonic()-began>570:raise TimeoutError('Manual adjustment permission expired')
                    seq.arm_state();time.sleep(.2)
                event=json.loads(path.read_text(encoding='utf8'))
                if event.get('index')!=index or event.get('phase')!='await_grip_tune':raise RuntimeError('Stale input permission')
                key=event['value'];index+=1
                if key!='q' and not all(event.get(k) is True for k in
                        ('shell_fully_supported','hand_swept_volume_clear','onsite_stop_ready')):
                    raise RuntimeError('New fixed-support/clear-path conditions missing')
                if key=='l':
                    if not any(move['key'] in ('t','i','m','f') for move in report['moves']):
                        raise RuntimeError('This session needs an accepted new grip adjustment before a new lift')
                    if not all(event.get(k) is True for k in ('opposed_grip_and_loaded_up140_clear',
                            'no_uncontrolled_motion_slip_or_overheating_observed')):
                        raise RuntimeError('New explicit lift permission missing')
                    if not same_pose(seq.full_arm(),reference):raise RuntimeError('Arm changed after stationary hand work')
                    if seq.call('Get_Collision_Stage')!=[0,8] or seq.call('Get_Self_Collision_Enable')!=[0,True]:
                        raise RuntimeError('Protections changed before new grip qualification')
                    from high_trial_tip_readonly import collect
                    with (folder/'tip_wire.jsonl').open('w',encoding='utf8') as tipwire:
                        tip=collect(lambda row:append(tipwire,row))
                    if tip!=source['live_tip_parameters']:raise RuntimeError('Tip parameters changed')
                    report['live_tip_parameters']=tip
                    hand,before=open_hand()
                    qualification_started=time.monotonic()
                    rows=[hand.snapshot() for _ in range(3)]
                    for row in rows:hand.check_targets(row,active)
                    if (any(any(abs(a-b)>150 for a,b in zip(row['positions'],active)) for row in rows)
                            or any(max(row['positions'][ch] for row in rows)-min(row['positions'][ch] for row in rows)>150 for ch in range(6))):
                        raise RuntimeError('New tuned grip not reached/quiet; no lift')
                    seq.close_hand()
                    # Distinct current full-arm read after new grip work and
                    # healthy groups. Never copy the failed last_arm_state.
                    qualified=seq.full_arm()
                    if not same_pose(qualified,reference) or qualified['limits']!=reference['limits']:
                        raise RuntimeError('Arm changed during new grip qualification')
                    receipt=dict(recipe=active[:],current_arm=reference,groups=rows,
                        full_current_arm=qualified,observation_seconds=time.monotonic()-qualification_started,
                        configuration=report['configuration'],user_event=event,old_fault_is_not_motion_authority=True)
                    dump(folder/'new_grip_receipt.json',receipt)
                    token._completed=True
                    delivery_authority=PhaseAuthority();delivery_authority.accepted_identity_raw=token.accepted_identity_raw[:]
                    delivery_authority.tuned_grip_freshly_qualified=True
                    delivery_authority.tuned_source_sha256=hashlib.sha256((folder/'new_grip_receipt.json').read_bytes()).hexdigest()
                    def delivery_log(row):
                        append(wire,row)
                        if row.get('event')=='tx' and row.get('request',{}).get('command')==WRITE:
                            report['hand_register_writes']+=1;save()
                    def delivery_factory(active_seq,guarded):
                        if not guarded or active_seq.io is not None:raise RuntimeError('One fresh delivery hand session required')
                        cls=NormalDropTransport if active_seq.hand_role=='bin_drop' else HeldReadTransport
                        active_seq.io=cls(socket.create_connection(('169.254.128.18',8080),timeout=3),delivery_log,delivery_authority)
                        return TunedHand(active_seq,active_seq.io,report,save,samples)
                    seq.authority=delivery_authority;seq.hand_factory=delivery_factory
                    seq.tuned_recipe=active[:];seq.load_targets=active[:];seq.load_anchor=copy.deepcopy(rows[-1])
                    seq.drive_limits=dict(minimum=seq.tune_drive_limits[0],maximum=seq.tune_drive_limits[1])
                    seq.loaded_lift_speed_percent=2;seq.new_lift_permission=copy.deepcopy(event)
                    seq.delivery_mode=True
                    report.update(delivery_mode=True,new_grip_receipt=receipt,loaded_lift_speed_percent=2,
                        normal_token_is_new=True,old_fault_latch_reset=False,load_targets=active[:],load_anchor=seq.load_anchor,
                        loaded_lift_performed=False,load_lift_or_transfer_performed=False,object_lift=False)
                    save()
                    try:seq.deliver_held_shell(reference)
                    except BaseException:
                        # Stop while diagnostic streams are still open; the
                        # outer exception handler runs after the with exits.
                        try:seq.fault()
                        except BaseException as stop_error:report['fault_stop_error']=repr(stop_error)
                        report['motion_fault_handled_inside_open_logs']=True
                        raise
                    break
                if key in ('s','q'):
                    if key=='s':
                        if not same_pose(seq.full_arm(),reference):raise RuntimeError('Arm changed before recipe confirmation')
                        if seq.call('Get_Collision_Stage')!=[0,8] or seq.call('Get_Self_Collision_Enable')!=[0,True]:raise RuntimeError('Protection changed')
                        hand,final=open_hand();report['final_confirmed_feedback']=final;seq.close_hand()
                    report.update(outcome='supported_grip_recipe_recorded_no_lift' if key=='s' else 'left_current_pose_no_lift',
                        confirmed_recipe=active[:],operator_reported_contact_fit=(key=='s'));break
                if key not in ('t','i','m','f','o'):raise ValueError('Unexpected input')
                ch={'t':0,'i':1,'m':2,'f':3}.get(key,0)
                if key!='o' and (ch in blocked or active[ch]>=CAPS[ch]):
                    report['last_adjustment_refused']='Channel at contact state5 or bounded maximum';save();continue
                if not same_pose(seq.full_arm(),reference):raise RuntimeError('Fresh pose changed; no hand write')
                # Every raw protection query finishes with the hand socket closed.
                if seq.call('Get_Collision_Stage')!=[0,8] or seq.call('Get_Self_Collision_Enable')!=[0,True]:raise RuntimeError('Protection changed')
                hand,before=open_hand()
                address=1135 if key=='o' else 1135+ch
                values=[8000]*4 if key=='o' else [min(active[ch]+STEPS[ch],CAPS[ch])]
                if key=='o' and any(v<8000 for v in before['positions'][:4]):raise RuntimeError('PREPARE opening would add closure to an actual finger')
                if key!='o' and before['status'][ch]==5:
                    blocked.add(ch);seq.close_hand();continue
                hand.guard()
                attempt=dict(key=key,address=address,values=values[:],attempted=True,acknowledged=False,feedback_accepted=False)
                report.setdefault('command_attempts',[]).append(attempt)
                seq.io.grant(address,values);report['hand_register_writes']+=1;save()
                seq.io.write_group(address,values);attempt['acknowledged']=True
                if key=='o':active[:4]=values
                else:active[ch]=values[0]
                report['active_targets']=active[:];report['last_acknowledged_targets']=active[:];save()
                frozen=False;stable=0;quiet=None;low=high=None;started=time.monotonic()
                while time.monotonic()-started<20:
                    row=hand.snapshot();hand.check_targets(row,active)
                    if key!='o' and row['status'][ch]==5:
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
                            reduction['acknowledged']=True
                            active[ch]=value;report['active_targets']=active[:];report['last_acknowledged_targets']=active[:];save()
                            frozen=True;stable=0;quiet=None;low=high=None;continue
                    inactive=range(4,6) if key=='o' else [i for i in range(6) if i!=ch]
                    if any(abs(row['positions'][i]-before['positions'][i])>150 for i in inactive):raise RuntimeError('Uncommanded finger drift')
                    pos=row['positions']
                    if low is None:low=pos[:];high=pos[:]
                    low=[min(x,y) for x,y in zip(low,pos)];high=[max(x,y) for x,y in zip(high,pos)]
                    if max(y-x for x,y in zip(low,high))<=150 and all(abs(v-t)<=150 for v,t in zip(pos,active)):
                        if quiet is None:quiet=time.monotonic()
                        stable+=1
                        if stable>=3 and time.monotonic()-quiet>=1:break
                    else:low=pos[:];high=pos[:];stable=0;quiet=None
                else:raise TimeoutError('Adjustment feedback did not settle; no retry')
                report['moves'].append(dict(key=key,address=address,values=values,active_targets=active[:],feedback=row,
                    contact_target_reduction=frozen,calibrated_force_or_holding_verified=False));save()
                attempt['feedback_accepted']=True;save()
                seq.close_hand()
                if key=='o':report['outcome']='supported_PREPARE_opened_no_arm_motion';break
            report['phase']='completed';save()
    except BaseException as exc:
        if seq is not None and not report.get('motion_fault_handled_inside_open_logs'):
            seq.authority._terminal_latched=True
            seq.hand_motion_enabled=False
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
