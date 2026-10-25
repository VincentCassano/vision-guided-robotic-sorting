"""Freshly qualify a held current start, then transfer; no regrip/new lift."""
import argparse
import copy
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
SOURCE_SHA='6652427ab133c3d48ed334f548595b523874436f16d174d833a596b634848614'
RECIPE=[26500,30500,29500,22000,65535,65056]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--folder',required=True,type=Path)
    p.add_argument('--source-report',required=True,type=Path)
    p.add_argument('--sdk-dir',required=True,type=Path)
    a=p.parse_args();folder=a.folder
    report=dict(scope='new_current_held_transfer_not_fault_resume',phase='preflight',
        arm_actuator_writes=0,hand_register_writes=0,configuration_writes=0,exceptions=[],cleanup_errors=[],
        execution_ready=False,selected_color='black',source_fault_last_arm_state_used=False,
        no_new_lift_or_regrip=True,automatic_fault_release=False,physical_stop_verified=False,
        torque_release_verified=False,previous_140mm_endpoint_accepted=False)
    seq=robot=None
    def save():dump(folder/'worker_result.json',report)
    def append(stream,row):stream.write(json.dumps(row,ensure_ascii=False)+'\n');stream.flush()
    save()
    try:
        from held_bin_transfer_sequence import HeldTransferSequence
        from motion_event_hand_transport import MotionEventReadTransport
        from tuned_grip_bin_hand import TunedHand,NormalDropTransport
        from supported_contact_transport import PhaseAuthority
        from inspect_left_hand import connect
        from high_descent_prepare_worker import same_pose
        from pregrasp_measurement_worker import source_audit,NATIVE_SHA
        from sdk_environment import collect_sdk_environment
        from arm_protection_transport import ProtectionClient
        from taught_model_readonly import collect_model_parameters
        from battery_grasp.taught_dh_model import EXPECTED_DH
        from battery_grasp.real_grasp import orientation_angle_rad
        from high_trial_tip_readonly import collect
        from school_device_identity import configuration_matches
        from school_battery_transport import WRITE
        raw=a.source_report.read_bytes()
        if hashlib.sha256(raw).hexdigest()!=SOURCE_SHA:raise RuntimeError('Reviewed123051 original bytes required')
        source=json.loads(raw);receipt=source['new_grip_receipt']
        if (source.get('arm_actuator_writes')!=1 or source.get('hand_register_writes')!=4
                or source.get('active_targets')!=RECIPE or source.get('outcome')!='aborted_no_further_writes'
                or source.get('error')!="TransportError('Unsolicited or late response before send')"
                or receipt['recipe']!=RECIPE or receipt['old_fault_is_not_motion_authority'] is not True
                or source.get('motion_fault_handled_inside_open_logs') is not True):
            raise RuntimeError('Unexpected original held-grip event')
        baseline=receipt['full_current_arm']  # Successful pre-lift frame/limit provenance only.
        command=source['current_school_motion']
        if (command['method']!='Movel_Cmd' or command['start']['frame_signatures']!=baseline['frame_signatures']
                or command['start']['limits']!=baseline['limits']
                or abs(command['target'][2]-command['start']['pose'][2]-.140)>1e-6):
            raise RuntimeError('Original commanded corridor differs')
        profile=json.loads((ROOT/'deployment/battery_hand_verified_profile.json').read_bytes())
        if not configuration_matches(source['configuration'],profile['observed_configuration']):
            raise RuntimeError('Source device profile differs')
        authority=PhaseAuthority();authority.accepted_identity_raw=receipt['groups'][-1]['identity'][:]
        authority.held_opposition_target=RECIPE[5];authority.normal_drop_targets=[0,0,0,0,0,RECIPE[5]]
        report.update(source_sha256=SOURCE_SHA,source_success_grip_receipt=receipt,
            user_observed_shell_lifted_and_still_held=True,
            reference_source='new_fresh_current_healthy_stationary_held_start_no_fault_last_state',
            original_success_frames_limits=baseline,original_commanded_corridor=command,
            loaded_lift_performed=False,object_lift=False)
        with (ROOT/'captures/hand_stage.lock').open('a') as lock, \
             (folder/'hand_wire.jsonl').open('w',encoding='utf8') as wire, \
             (folder/'protection_wire.jsonl').open('w',encoding='utf8') as pwire, \
             (folder/'tip_wire.jsonl').open('w',encoding='utf8') as twire, \
             (folder/'samples.jsonl').open('w',encoding='utf8') as samples:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            report['source_audit']=source_audit(a.sdk_dir)
            model=collect_model_parameters(folder/'fresh_model');report['fresh_model']=model;save()
            if (not model['identity_valid'] or not model['dh_valid'] or model['errors']
                    or [model['dh_response']['joint_'+str(i)] for i in range(1,7)]!=EXPECTED_DH):
                raise RuntimeError('Fresh device model differs')
            class Transfer(HeldTransferSequence):
                route_allowed=False
                notification=None
                def call(self,name,*args,**kwargs):
                    if not name.startswith('Get_'):
                        if name not in ('Movej_P_Cmd','Move_Stop_Cmd') or not self.route_allowed:
                            raise RuntimeError('No lift/descent/regrip/config write in current-held transfer')
                        if name=='Movej_P_Cmd' and (self.bin_motion is None or args[0]!=self.bin_motion['target']):
                            raise RuntimeError('Unscoped route write')
                    if name in ('Get_Collision_Stage','Get_Self_Collision_Enable') and self.io is not None:
                        raise RuntimeError('Raw protection GET requires closed hand TCP')
                    value=super().call(name,*args,**kwargs)
                    if name=='Movej_P_Cmd' and type(value) is int and value==0:
                        self.notification=dict(acknowledged=True,event_seen=False,method=name,
                            segment=self.report['arm_actuator_writes'],phase=self.report['phase'])
                    return value
                def wait_input(self,index,phase,arm,values,teaching=False):
                    if index not in (11,12):raise RuntimeError('Only transfer/drop/empty-return permissions')
                    result=super().wait_input(index,phase,arm,values,teaching)
                    if index==11:self.route_allowed=True
                    return result
            def hand_log(row):
                append(wire,row)
                if row.get('event')=='tx' and row.get('request',{}).get('command')==WRITE:
                    report['hand_register_writes']+=1;save()
            def factory(active,guarded):
                if not guarded or active.io is not None:raise RuntimeError('One fresh guarded hand connection required')
                sock=socket.create_connection(('169.254.128.18',8080),timeout=3)
                if active.hand_role=='bin_drop':active.io=NormalDropTransport(sock,hand_log,authority)
                else:active.io=MotionEventReadTransport(sock,hand_log,authority,
                    lambda:active.notification if active.bin_motion is not None else None)
                return TunedHand(active,active.io,report,save,samples)
            robot=connect(a.sdk_dir)
            seq=Transfer(robot,baseline,profile,report,save,samples,factory,folder=folder,authority=authority,
                tip_reader=lambda:collect(lambda row:append(twire,row)),
                protection_client=ProtectionClient(lambda row:append(pwire,row)),enable_arm_protection=False,
                resume_report=None,completed_corridor=None,taught_only=True,allow_protected_opening=False)
            seq.hand_role='moving'
            metadata=collect_sdk_environment(robot);report['sdk_environment']=metadata
            if (not metadata['metadata_complete'] or metadata.get('api_version')!='4.3.1'
                    or metadata.get('loaded_native',{}).get('sha256')!=NATIVE_SHA
                    or metadata.get('host_machine')!='aarch64'
                    or metadata.get('controller_run_mode_raw',{}).get('data')!=[1]):
                raise RuntimeError('SDK/native/current real mode differs')
            current=seq.full_arm()
            if current['frame_signatures']!=baseline['frame_signatures'] or current['limits']!=baseline['limits']:
                raise RuntimeError('Fresh frames/limits differ')
            start=command['start']['pose'];target=command['target'];pose=current['pose']
            corridor=dict(xy_error_mm=math.dist(pose[:2],start[:2])*1000,
                rotation_angle_rad=orientation_angle_rad(pose[3:],start[3:]),
                z_in_original_command=min(start[2],target[2])-.001<=pose[2]<=max(start[2],target[2])+.001,
                original_target_error_mm=math.dist(pose[:3],target[:3])*1000,
                original_140mm_endpoint_accepted=False,old_fault_pose_used=False)
            report['fresh_current_corridor_comparison']=corridor;save()
            if corridor['xy_error_mm']>1 or corridor['rotation_angle_rad']>.005 or not corridor['z_in_original_command']:
                raise RuntimeError('Fresh held pose outside original commanded corridor')
            # Independent current start, never fault last_arm_state or a fake
            # successful140mm endpoint. New transfer still needs actual clearance.
            seq.reference=copy.deepcopy(current);seq.guard_target=current['joints'][:];seq.guard_pose=pose[:]
            report['fresh_current_held_reference']=copy.deepcopy(current)
            began=time.monotonic();previous=None;groups=0
            while groups<5 or time.monotonic()-began<1:
                q,p=seq.arm_state()
                if not same_pose(dict(current,joints=q,pose=p),current):raise RuntimeError('Current held start drifted')
                if previous is not None and (max(abs(a-b) for a,b in zip(q,previous[0]))>.02
                        or math.dist(p[:3],previous[1][:3])>.0002
                        or orientation_angle_rad(p[3:],previous[1][3:])>.002):raise RuntimeError('Current held start not quiet')
                previous=q,p;groups+=1;time.sleep(.25)
            low,high=seq.safety_reads();seq.drive_limits=dict(minimum=low,maximum=high)
            if (report['fresh_protection_values'].get('Get_Collision_Stage')!=[0,8]
                    or report['fresh_protection_values'].get('Get_Self_Collision_Enable')!=[0,True]
                    or report['arm_safety_reads']['Get_Install_Pose']!=[0,0,45,0]):
                raise RuntimeError('Fresh protection/install differs; no setters')
            report['live_tip_parameters']=seq.tip_reader();save()
            if report['live_tip_parameters']!=source['live_tip_parameters']:raise RuntimeError('Fresh tip settings differ')
            seq.tuned_recipe=RECIPE[:];seq.tuned_drop_targets=authority.normal_drop_targets[:]
            seq.load_targets=RECIPE[:];seq.load_anchor=copy.deepcopy(receipt['groups'][-1])
            hand=factory(seq,True)
            try:
                hand.configuration();hand.preflight_loaded(RECIPE,seq.load_anchor)
                rows=[hand.monitor_loaded(RECIPE,seq.load_anchor) for _ in range(3)]
                if any(max(row['positions'][ch] for row in rows)-min(row['positions'][ch] for row in rows)>150 for ch in range(6)):
                    raise RuntimeError('Current held grip not quiet')
                seq.load_anchor=copy.deepcopy(rows[-1])
            finally:seq.close_hand()
            if not same_pose(seq.full_arm(),current):raise RuntimeError('Arm changed during current held qualification')
            qualification=dict(arm=current,groups=rows,recipe=RECIPE,old_fault_pose_used=False,
                old_140mm_accepted=False,actual_holding_source='onsite_operator',new_motion_needs_permission11=True)
            dump(folder/'current_held_receipt.json',qualification)
            report['current_held_qualification']=qualification;save()
            authority.tuned_grip_freshly_qualified=True
            authority.tuned_source_sha256=hashlib.sha256((folder/'current_held_receipt.json').read_bytes()).hexdigest()
            try:seq.deliver_current_upper(current)
            except BaseException:
                seq.fault();report['motion_fault_handled_inside_open_logs']=True;raise
    except BaseException as exc:
        report.update(phase='fault',outcome='aborted_no_further_writes',execution_ready=False,
            error=repr(exc),traceback=traceback.format_exc());save()
    finally:
        if seq is not None:
            try:seq.close_hand()
            except BaseException as exc:report['cleanup_errors'].append(repr(exc))
        if robot is not None:
            for name in ('Arm_Socket_Close','RM_API_UnInit'):
                try:getattr(robot,name)()
                except BaseException as exc:report['cleanup_errors'].append(repr(exc))
        save()
    return 0 if report.get('phase')=='completed' and not report['cleanup_errors'] else 1


if __name__=='__main__':raise SystemExit(main())
