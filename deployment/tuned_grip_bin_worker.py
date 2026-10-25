"""Fresh current held-grip qualification, then the existing supervised bin route."""
import argparse
import copy
import fcntl
import hashlib
import json
from pathlib import Path
import socket
import sys
import traceback
from known_cylinder_records import dump

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/battery_grasp/src'))
SOURCE_SHA='2480000a1d2d0670a05ce3dbbe7272dc59346d28a671a9f713e98eab571893f6'


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--folder',required=True,type=Path)
    p.add_argument('--sdk-dir',required=True,type=Path)
    p.add_argument('--color',required=True,choices=('black','red','white'))
    a=p.parse_args();folder=a.folder
    report=dict(scope='current_tuned_grip_lift_school_bin_drop_no_reclose',phase='preflight',
        outcome='starting',exceptions=[],cleanup_errors=[],arm_actuator_writes=0,hand_register_writes=0,
        configuration_writes=0,execution_ready=False,selected_color=a.color,source_fault_last_arm_state_used=False,
        loaded_lift_performed=False,load_lift_or_transfer_performed=False,object_lift=False,
        automatic_open=False,automatic_return=False,automatic_force_zero=False,
        physical_stop_verified=False,torque_release_verified=False,grip_holding_verified=False,
        vision_used=False,camera_extrinsics_used_for_motion=False,offline_tests_executed=False,
        no_prepare_descent_or_new_close=True,SDK_in_parent=False)
    robot=seq=None
    def save():dump(folder/'worker_result.json',report)
    def append(stream,row):stream.write(json.dumps(row,ensure_ascii=False)+'\n');stream.flush()
    save()
    try:
        from tuned_grip_bin_hand import TunedHand,HeldReadTransport,NormalDropTransport,RECIPE
        from school_bin_cycle_v4_sequence import SchoolBinCycle,SUCCESS
        from supported_contact_transport import PhaseAuthority
        from inspect_left_hand import connect
        from high_descent_prepare_worker import same_pose
        from pregrasp_measurement_worker import source_audit,NATIVE_SHA
        from sdk_environment import collect_sdk_environment
        from arm_protection_transport import ProtectionClient
        from taught_model_readonly import collect_model_parameters
        from battery_grasp.taught_dh_model import EXPECTED_DH
        from high_trial_tip_readonly import collect
        from school_device_identity import configuration_matches
        from school_battery_transport import WRITE
        raw=(ROOT/'deployment/grip_tune_source_081023.json').read_bytes()
        if hashlib.sha256(raw).hexdigest()!=SOURCE_SHA:raise RuntimeError('Reviewed081023 successful source bytes required')
        source=json.loads(raw)
        if (source.get('phase')!='completed' or source.get('outcome')!='supported_grip_recipe_recorded_no_lift'
                or source.get('confirmed_recipe')!=RECIPE or source.get('operator_reported_contact_fit') is not True
                or source.get('arm_actuator_writes')!=0 or source.get('hand_register_writes')!=9
                or source.get('error') or source.get('cleanup_errors') or source.get('exceptions')
                or any(v in (3,4) for v in source['final_confirmed_feedback']['status'])):
            raise RuntimeError('Successful tuned recipe provenance differs')
        baseline_raw=(ROOT/'deployment/grip_tune_source_024412.json').read_bytes()
        if hashlib.sha256(baseline_raw).hexdigest()!='d69a9e0c91d4aa34b68ab8d7ae75600d14f147135fec1b7ce7c5d36b9b0e3372':
            raise RuntimeError('Qualified original completed descent provenance differs')
        baseline=json.loads(baseline_raw);reference=baseline['arm_segments'][-1]['endpoint']
        if baseline['arm_segments'][-1]['phase']!='descending_to_supported_shell':raise RuntimeError('Missing successful descent')
        if reference!=source['qualified_reference']:raise RuntimeError('Tuning guard did not use the same accepted reference')
        profile=json.loads((ROOT/'deployment/battery_hand_verified_profile.json').read_bytes())
        if not configuration_matches(source['configuration'],profile['observed_configuration']):raise RuntimeError('Source full device profile differs')
        report.update(source_success_sha256=SOURCE_SHA,source_recipe=RECIPE[:],
            source_success_hand=source['final_confirmed_feedback'],qualified_success_reference=reference,
            source_reference_kind='original_completed_descent_plus_successful_stationary_tuning',
            source_audit=source_audit(a.sdk_dir),load_targets=RECIPE[:])
        with (ROOT/'captures/hand_stage.lock').open('a') as lock, \
             (folder/'hand_wire.jsonl').open('w',encoding='utf8') as wire, \
             (folder/'samples.jsonl').open('w',encoding='utf8') as samples, \
             (folder/'protection_wire.jsonl').open('w',encoding='utf8') as pwire, \
             (folder/'tip_wire.jsonl').open('w',encoding='utf8') as twire:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            model=collect_model_parameters(folder/'fresh_model');report['fresh_model']=model;save()
            if (not model['dh_valid'] or not model['identity_valid'] or model['errors']
                    or [model['dh_response']['joint_'+str(i)] for i in range(1,7)]!=EXPECTED_DH):
                raise RuntimeError('Fresh actual model/identity differs')
            authority=PhaseAuthority();authority.accepted_identity_raw=source['final_confirmed_feedback']['identity'][:]
            authority.tuned_source_sha256=SOURCE_SHA;authority.tuned_grip_freshly_qualified=False
            def hand_log(row):
                append(wire,row)
                if row.get('event')=='tx' and row.get('request',{}).get('command')==WRITE:
                    report['hand_register_writes']+=1;save()
            def factory(active,guarded):
                if not guarded or active.io is not None:raise RuntimeError('One guarded hand session required')
                cls=NormalDropTransport if active.hand_role=='bin_drop' else HeldReadTransport
                active.io=cls(socket.create_connection(('169.254.128.18',8080),timeout=3),hand_log,authority)
                return TunedHand(active,active.io,report,save,samples)
            robot=connect(a.sdk_dir)
            seq=SchoolBinCycle(robot,reference,profile,report,save,samples,factory,folder=folder,authority=authority,
                tip_reader=lambda:collect(lambda row:append(twire,row)),
                protection_client=ProtectionClient(lambda row:append(pwire,row)),enable_arm_protection=False,
                resume_report=None,completed_corridor=None,taught_only=True,allow_protected_opening=False)
            seq.hand_role='moving';seq.guard_target=reference['joints'][:];seq.guard_pose=reference['pose'][:]
            try:
                metadata=collect_sdk_environment(robot);report['sdk_environment']=metadata
                if (not metadata['metadata_complete'] or metadata.get('api_version')!='4.3.1'
                        or metadata.get('loaded_native',{}).get('sha256')!=NATIVE_SHA
                        or metadata.get('host_machine')!='aarch64'
                        or metadata.get('controller_run_mode_raw',{}).get('data')!=[1]):
                    raise RuntimeError('SDK identity/current real mode differs')
                arm=seq.full_arm()
                if not same_pose(arm,reference) or arm['limits']!=reference['limits']:
                    raise RuntimeError('Current arm/frames/limits moved since successful tuned grasp; no alignment')
                low,high=seq.safety_reads();seq.drive_limits=dict(minimum=low,maximum=high)
                install=report['arm_safety_reads']['Get_Install_Pose']
                if (report['fresh_protection_values']['Get_Collision_Stage']!=[0,8]
                        or report['fresh_protection_values']['Get_Self_Collision_Enable']!=[0,True]
                        or not isinstance(install,list) or len(install)!=4 or install[0]!=0
                        or any(abs(x-y)>.001 for x,y in zip(install,[0,0,45,0]))):
                    raise RuntimeError('Current protections/install differ; no settings write')
                seq.arm_state()
                # Finish every raw controller GET before opening the hand TCP.
                report['live_tip_parameters']=seq.tip_reader();save()
                if report['live_tip_parameters']!=baseline['live_tip_parameters']:
                    raise RuntimeError('Current tip speed/acceleration differs from accepted device readback')
                seq.load_targets=RECIPE[:];seq.load_anchor=copy.deepcopy(source['final_confirmed_feedback'])
                hand=factory(seq,True)
                try:
                    hand.configuration();hand.preflight_loaded(RECIPE,seq.load_anchor)
                    rows=[hand.monitor_loaded(RECIPE,seq.load_anchor) for _ in range(3)]
                    if any(max(r['positions'][ch] for r in rows)-min(r['positions'][ch] for r in rows)>150 for ch in range(6)):
                        raise RuntimeError('Current tuned grip not quiet')
                    seq.load_anchor=copy.deepcopy(rows[-1])
                    report.update(load_anchor=copy.deepcopy(seq.load_anchor),tuned_grip_fresh_groups=rows,
                        read_pacing_seconds=.3,new_timing_probe=False)
                    authority.tuned_grip_freshly_qualified=True
                finally:seq.close_hand()
                if not same_pose(seq.full_arm(),arm):raise RuntimeError('Arm changed during current grip qualification')
                seq.deliver_held_shell(arm)
            except BaseException:
                seq.fault();raise
            finally:
                seq.close_hand()
                handle,robot=robot,None
                for name in ('Arm_Socket_Close','RM_API_UnInit'):
                    try:getattr(handle,name)()
                    except BaseException as exc:report['cleanup_errors'].append(dict(method=name,error=repr(exc)))
    except BaseException as exc:
        report['exceptions'].append(dict(error=repr(exc),traceback=traceback.format_exc()))
        report.update(phase='fault',outcome='aborted_no_further_writes',execution_ready=False,
            automatic_open=False,automatic_return=False,automatic_force_zero=False)
    finally:
        if robot is not None:
            for name in ('Arm_Socket_Close','RM_API_UnInit'):
                try:getattr(robot,name)()
                except BaseException as exc:report['cleanup_errors'].append(dict(method=name,error=repr(exc)))
        if report['cleanup_errors']:report.update(outcome='failed_cleanup',execution_ready=False)
        save()
    return 0 if report.get('outcome')=='completed_school_bin_drop_feedback_and_operator_empty_confirmation' and not report['exceptions'] and not report['cleanup_errors'] else 1


if __name__=='__main__':raise SystemExit(main())
