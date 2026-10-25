"""Isolated actual SDK: accepted visual pregrasp -> grip -> lift -> bin -> empty return."""
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
    parser.add_argument('--align-empty-opposition-first',action='store_true')
    parser.add_argument('--source-pregrasp',type=Path,required=True)
    parser.add_argument('--source-sha256',required=True)
    parser.add_argument('--manifest-sha256',required=True)
    parser.add_argument('--color',choices=('black',),required=True)
    parser.add_argument('--sdk-dir',type=Path,required=True)
    args=parser.parse_args()
    if platform.system()!='Linux' or platform.machine()!='aarch64':parser.error('Actual robot Linux aarch64 only')
    from known_cylinder_records import dump
    folder=args.folder
    report=dict(scope='continuous_supervised_visual_pregrasp_grip_lift_sort_v4',
        phase='preflight',outcome='starting',exceptions=[],cleanup_errors=[],execution_ready=False,
        arm_actuator_writes=0,hand_register_writes=0,controller_configuration_writes=0,
        configuration_writes=0,selected_color=args.color,offline_tests_executed=False,
        explicit_empty_opposition_withdrawal_requested=args.align_empty_opposition_first,
        source_fault_last_arm_state_used=False,automatic_fault_open=False,automatic_return=False,
        physical_stop_verified=False,torque_release_verified=False,object_lift=False)
    robot=seq=None
    def save():dump(folder/'worker_result.json',report)
    def append(stream,row):stream.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n');stream.flush()
    save()
    try:
        from visual_pick_sort_v4_source import verify_delivery,accepted_pregrasp,grip_recipe_proof,MANIFEST,PREPARE,RECIPE
        if hashlib.sha256(MANIFEST.read_bytes()).hexdigest()!=args.manifest_sha256:raise ValueError('Manifest changed')
        verify_delivery();source=accepted_pregrasp(args.source_pregrasp,args.source_sha256)
        report['accepted_visual_source']=source;report['recipe_provenance']=grip_recipe_proof();save()
        from visual_pick_sort_v4_hand import VisualHand,GripTransport,MovingTransport,DropTransport
        from visual_pick_sort_v4_alignment import AlignmentHand,AlignmentTransport
        from school_bin_cycle_v4_sequence import SchoolBinCycle,SUCCESS
        from visual_pick_sort_v2_sequence import ContinuousDelivery
        from supported_contact_transport import PhaseAuthority
        from school_operational_reads import KnownReadAuthority
        from visual_startup_reads_v4 import StartupReadSession
        from school_operational_readiness import wait_hand_ready
        from inspect_left_hand import connect
        from pregrasp_measurement_worker import source_audit,NATIVE_SHA
        from sdk_environment import collect_sdk_environment
        from taught_model_readonly import collect_model_parameters
        from battery_grasp.taught_dh_model import EXPECTED_DH
        from high_descent_prepare_worker import same_pose
        from battery_grasp.real_grasp import orientation_angle_rad
        from arm_protection_transport import ProtectionClient
        from high_trial_tip_readonly import collect
        from visual_run_configuration_v3 import observe_configuration,bind_or_require_frames,require_current_configuration
        from school_battery_transport import WRITE
        import fcntl
        report['sdk_source_audit']=source_audit(args.sdk_dir)
        with (ROOT/'captures/hand_stage.lock').open('a') as lock, \
             (folder/'hand_wire.jsonl').open('w',encoding='utf8') as wire, \
             (folder/'samples.jsonl').open('w',encoding='utf8') as samples, \
             (folder/'protection_wire.jsonl').open('w',encoding='utf8') as pwire, \
             (folder/'tip_wire.jsonl').open('w',encoding='utf8') as twire:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            model=collect_model_parameters(folder/'fresh_model');report['fresh_model']=model;save()
            if (not model['identity_valid'] or not model['dh_valid'] or model['errors']
                    or [model['dh_response']['joint_'+str(i)] for i in range(1,7)]!=EXPECTED_DH):
                raise RuntimeError('Actual identity/DH differs')
            with (folder/'current_configuration_wire.jsonl').open('w',encoding='utf8') as cwire:
                run=observe_configuration(cwire)
            if run['raw']!=source['configuration']['raw']:raise RuntimeError('Configuration changed since accepted visual endpoint')
            report['observed_run_configuration']=run
            wait_hand_ready(folder,report,save)
            authority=KnownReadAuthority();authority.accepted_identity_raw=report['startup_readiness']['final_identity_raw'][:]
            authority.grip_permission=False;authority.reduced_channels=set()
            def hand_log(row):
                append(wire,row)
                if row.get('event')=='tx' and row.get('request',{}).get('command')==WRITE:
                    report['hand_register_writes']+=1;save()
            class Route(ContinuousDelivery,SchoolBinCycle):
                route_allowed=False
                notification=None
                def close_hand(self):
                    if self.io is not None:
                        current,self.io=self.io,None;self.hand_motion_enabled=False
                        current.disarm_motion()
                        try:
                            if isinstance(current,StartupReadSession):current.close()
                            else:current.sock.close()
                        except BaseException as exc:
                            current.latch_fault('Hand channel cleanup failed')
                            report['cleanup_errors'].append(dict(method='hand.close',error=repr(exc)));raise
                def full_arm(self):
                    state=super().full_arm();bind_or_require_frames(run,state['frame_signatures']);return state
                def call(self,name,*values,**options):
                    if not name.startswith('Get_'):
                        if name not in ('Movel_Cmd','Movej_P_Cmd','Move_Stop_Cmd') or not self.route_allowed:
                            raise RuntimeError('No unscoped arm/configuration write')
                        if name!='Move_Stop_Cmd' and (self.bin_motion is None or name!=self.bin_motion['method']
                                or values[0]!=self.bin_motion['target']):raise RuntimeError('Unscoped route target')
                    if name in ('Get_Collision_Stage','Get_Self_Collision_Enable') and self.io is not None:
                        raise RuntimeError('Close hand channel before protection GET')
                    value=super().call(name,*values,**options)
                    if name in ('Movel_Cmd','Movej_P_Cmd'):
                        self.notification=dict(acknowledged=type(value) is int and value==0,event_seen=False,
                            method=name,phase=report['phase'])
                    return value
                def wait_input(self,index,phase,arm,values,teaching=False):
                    if index not in (4,5,12):raise RuntimeError('No teaching/reposition/prepare phase')
                    event=super().wait_input(index,phase,arm,values,teaching)
                    if index==5:self.route_allowed=True
                    return event
                def move_bin_segment(self,*values,**options):
                    self.close_hand()
                    with (folder/'current_configuration_wire.jsonl').open('a',encoding='utf8') as cwire:
                        require_current_configuration(run,cwire)
                    endpoint=super().move_bin_segment(*values,**options)
                    if not isinstance(self.notification,dict) or self.notification.get('event_seen') is not True:
                        raise RuntimeError('Completed feedback lacks current segment trajectory event; no next segment')
                    report['arm_segments'][-1]['notification']=copy.deepcopy(self.notification);save();return endpoint
            def factory(active,guarded):
                if not guarded or active.io is not None:raise RuntimeError('One guarded hand channel required')
                sock=socket.create_connection(('169.254.128.18',8080),timeout=3)
                if active.hand_role=='align_opposition':active.io=AlignmentTransport(sock,hand_log,active.authority)
                elif active.hand_role=='grip':active.io=GripTransport(sock,hand_log,active.authority)
                elif active.hand_role=='bin_drop':active.io=DropTransport(sock,hand_log,active.authority)
                else:active.io=MovingTransport(sock,hand_log,active.authority,
                    lambda:active.notification if active.bin_motion is not None else None)
                return (AlignmentHand if active.hand_role=='align_opposition' else VisualHand)(active,active.io,report,save,samples)
            robot=connect(args.sdk_dir)
            profile=json.loads((ROOT/'deployment/battery_hand_verified_profile.json').read_bytes())
            seq=Route(robot,source['arm'],profile,report,save,samples,factory,folder=folder,authority=authority,
                tip_reader=lambda:collect(lambda row:append(twire,row)),
                protection_client=ProtectionClient(lambda row:append(pwire,row)),enable_arm_protection=False,
                resume_report=None,completed_corridor=None,taught_only=True,allow_protected_opening=False)
            try:
                metadata=collect_sdk_environment(robot);report['sdk_environment']=metadata
                if (not metadata['metadata_complete'] or metadata.get('api_version')!='4.3.1'
                        or metadata.get('loaded_native',{}).get('sha256')!=NATIVE_SHA
                        or metadata.get('controller_run_mode_raw',{}).get('data')!=[1]):
                    raise RuntimeError('SDK/native/actual real mode differs')
                initial=seq.full_arm()
                if not same_pose(initial,source['arm']) or initial['limits']!=source['arm']['limits']:
                    raise RuntimeError('Current arm changed since accepted visual pregrasp; no automatic reposition')
                seq.reference=copy.deepcopy(initial);seq.guard_target=initial['joints'][:];seq.guard_pose=initial['pose'][:]
                report['startup_reference']=copy.deepcopy(initial);report['reference_source']='fresh_read_matched_accepted_visual_p'
                began=time.monotonic();previous=None;count=0
                while count<5 or time.monotonic()-began<1.:
                    q,p=seq.arm_state()
                    if not same_pose(dict(initial,joints=q,pose=p),initial):raise RuntimeError('Current arm drift before hand work')
                    if previous is not None and (max(abs(a-b) for a,b in zip(q,previous[0]))>.02
                            or math.dist(p[:3],previous[1][:3])>.0002
                            or orientation_angle_rad(p[3:],previous[1][3:])>.002):raise RuntimeError('Arm not quiet')
                    previous=q,p;count+=1;time.sleep(.25)
                low,high=seq.safety_reads();seq.drive_limits=dict(minimum=low,maximum=high)
                if report['arm_safety_reads']['Get_Install_Pose']!=[0,0,45,0]:raise RuntimeError('Install differs')
                tip=seq.tip_reader();report['live_tip_parameters']=tip
                if tip['line_speed_m_s']>.25 or tip['line_acc_m_s2']>1.6 or tip['angular_speed_rad_s']>.6 or tip['angular_acc_rad_s2']>4.:
                    raise RuntimeError('Actual tip parameters exceed recorded bounds')
                seq.hand_role='grip';seq.tuned_recipe=RECIPE[:];seq.tuned_drop_targets=[0,0,0,0,0,RECIPE[5]]
                def startup_guard():
                    if not same_pose(seq.full_arm(),initial):raise RuntimeError('Arm moved during read-only startup')
                seq.io=StartupReadSession(folder,report,save,hand_log,authority,startup_guard)
                if args.align_empty_opposition_first:
                    hand=AlignmentHand(seq,seq.io,report,save,samples)
                    hand.configuration();start=hand.before_alignment();seq.io.finish();seq.close_hand()
                    event=seq.wait_input(4,'await_empty_opposition_withdrawal',initial,('r',))
                    if event.get('empty_hand_and_opposition_sweep_clear') is not True:
                        raise RuntimeError('Current empty hand and thumb rotation sweep not confirmed')
                    if not same_pose(seq.full_arm(),initial):raise RuntimeError('Arm changed before opposition retreat')
                    with (folder/'current_configuration_wire.jsonl').open('a',encoding='utf8') as cwire:
                        require_current_configuration(run,cwire)
                    if seq.call('Get_Collision_Stage')!=[0,8] or seq.call('Get_Self_Collision_Enable')!=[0,True]:
                        raise RuntimeError('Protection changed before opposition retreat')
                    authority.empty_opposition_permission=True;seq.hand_role='align_opposition'
                    hand=factory(seq,True);hand._configuration_checked=True
                    start=hand.align_once(start);seq.close_hand()
                    if authority._terminal_latched:raise RuntimeError('Alignment fault cannot acquire normal capability')
                    authority._completed=True
                    authority=KnownReadAuthority();authority.accepted_identity_raw=start['identity'][:]
                    authority.grip_permission=False;authority.reduced_channels=set();seq.authority=authority
                    report['new_normal_token_after_healthy_empty_opposition_retreat']=True
                    report['new_hand_pose_needs_current_operator_noncontact_grasp_confirmation']=True
                else:
                    hand=VisualHand(seq,seq.io,report,save,samples)
                    hand.configuration();start=hand.preflight_prepare();seq.io.finish();seq.close_hand()
                seq.hand_role='grip'
                report['startup_hand']=copy.deepcopy(start);save()
                seq.prepare_bin_route(initial)
                event=seq.wait_input(5,'await_grasp',initial,('g',))
                if (event.get('only_intended_shell_contact_and_closure_clear') is not True
                        or event.get('current_noncontact_pose_and_same_shell_scene_confirmed') is not True
                        or event.get('continuous_lift_transfer_normal_drop_authorized') is not True
                        or event.get('planned_loaded_route_min20mm_clear_confirmed') is not True):
                    raise RuntimeError('Fresh same-shell supported contact permission missing')
                if not same_pose(seq.full_arm(),initial):raise RuntimeError('Arm changed after g')
                with (folder/'current_configuration_wire.jsonl').open('a',encoding='utf8') as cwire:
                    require_current_configuration(run,cwire)
                if seq.call('Get_Collision_Stage')!=[0,8] or seq.call('Get_Self_Collision_Enable')!=[0,True]:
                    raise RuntimeError('Protection changed before grip')
                authority.grip_permission=True
                seq.hand_motion_enabled=True
                hand=factory(seq,True);hand._configuration_checked=True
                active,row=hand.close_recipe();seq.tuned_recipe=active[:]
                rows=copy.deepcopy(report['grasp_qualified_groups'])
                if len(rows)!=3:raise RuntimeError('Three healthy settled groups required')
                for sample in rows:hand.checked_row(sample,active,row)
                if any(max(r['positions'][ch] for r in rows)-min(r['positions'][ch] for r in rows)>150 for ch in range(6)):
                    raise RuntimeError('New grip not quiet; no lift')
                if not same_pose(seq.full_arm(),initial):raise RuntimeError('Arm moved during grip')
                receipt=dict(recipe=active[:],groups=rows,configuration=report['configuration'],
                    source_visual_sha256=source['archive_sha256'],current_arm=seq.full_arm(),old_fault_is_not_motion_authority=True)
                dump(folder/'new_grip_receipt.json',receipt);seq.close_hand()
                if authority._terminal_latched:raise RuntimeError('Faulted grip cannot acquire delivery capability')
                authority._completed=True
                delivery=PhaseAuthority();delivery.continuous_cycle_permission=copy.deepcopy(event)
                delivery.accepted_identity_raw=rows[-1]['identity'][:]
                delivery.tuned_grip_freshly_qualified=True;delivery.held_opposition_target=active[5]
                delivery.normal_drop_targets=seq.tuned_drop_targets[:]
                delivery.tuned_source_sha256=hashlib.sha256((folder/'new_grip_receipt.json').read_bytes()).hexdigest()
                seq.authority=delivery;seq.load_targets=active[:];seq.load_anchor=copy.deepcopy(rows[-1])
                seq.loaded_lift_speed_percent=2;seq.startup_observation_only=False
                report.update(new_grip_receipt=receipt,load_targets=active[:],load_anchor=seq.load_anchor,
                    old_fault_latch_reset=False,normal_delivery_token_from_fresh_healthy_grip=True,
                    loaded_lift_speed_percent=2,loaded_lift_performed=False)
                save();seq.deliver_held_shell(initial)
            except BaseException:
                try:seq.fault()
                except BaseException as exc:report['fault_handler_error']=repr(exc)
                raise
            finally:seq.close_hand()
    except BaseException as exc:
        report.update(phase='fault',outcome='aborted_no_further_writes',execution_ready=False)
        report['exceptions'].append(dict(error=repr(exc),traceback=traceback.format_exc()))
    finally:
        if robot is not None:
            for name in ('Arm_Socket_Close','RM_API_UnInit'):
                try:
                    value=getattr(robot,name)();report.setdefault('sdk_cleanup_results',[]).append(dict(method=name,raw_repr=repr(value)))
                    if name=='RM_API_UnInit' and (type(value) is not int or value!=0):
                        report['cleanup_errors'].append(dict(method=name,error='Uninitialization returned nonzero/unknown',raw_repr=repr(value)))
                except BaseException as exc:report['cleanup_errors'].append(dict(method=name,error=repr(exc)))
        if report['cleanup_errors']:report['outcome']='failed_cleanup'
        report['execution_ready']=False;save()
    return 0 if report.get('outcome')=='completed_school_bin_drop_feedback_and_operator_empty_confirmation' and not report['exceptions'] and not report['cleanup_errors'] else 1


if __name__=='__main__':raise SystemExit(main())
