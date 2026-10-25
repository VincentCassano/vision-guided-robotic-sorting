#!/usr/bin/env python3
"""Isolated onsite worker for one freshly taught supported low-lift cycle.

No camera capture, historical pose restoration, bus initialization, protection
setting, automatic fault opening, or automatic fault return is available here.
The sequence owns each explicit permission and records the real load outcome.
"""
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
import platform
import sys
import traceback


ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/battery_grasp/src'))

SUCCESS='completed_supported_low_lift_return_release'
SUPPORTED_ONLY='completed_supported_release_lift_not_performed'
OPENING_ONLY='completed_supported_opening_empty_return_blocked'


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--folder',required=True,type=Path)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--resume-before-grasp',type=Path)
    parser.add_argument('--sdk-dir',type=Path,
        default=Path('/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package'))
    args=parser.parse_args(argv)
    if not args.execute:parser.error('Explicit supervised low-lift execution required; no connection')
    if platform.system()!='Linux' or platform.machine()!='aarch64':
        parser.error('Reviewed onsite Linux aarch64 only; no connection')
    folder=args.folder
    folder.mkdir(parents=True,exist_ok=True)
    report=dict(schema_version=1,scope='fresh_taught_same_slot_supported_low_lift_return_release',
        timestamp_utc=datetime.now(timezone.utc).isoformat(),outcome='starting',phase='preflight',
        diagnostic_pass=False,execution_ready=False,full_pregrasp_execution_ready=False,
        sdk_in_parent=False,exceptions=[],cleanup_errors=[],arm_actuator_writes=0,
        hand_register_writes=0,controller_configuration_writes=0,hand_bus_configuration_writes=0,
        hand_register_writes_count_kind='raw_TX_attempts_not_acknowledged_or_stopped',
        hand_bus_configuration_writes_count_kind='raw_TX_attempts_not_ACK_or_physical_stop',
        hand_bus_initialization_requested=False,explicit_supported_opening_requested=True,
        arm_protection_configuration_requested=False,physical_stop_verified=False,
        torque_release_verified=False,automatic_open=False,automatic_return=False,
        automatic_grip=False,automatic_force_zero=False,object_lift=False,
        load_lift_or_transfer_performed=False,loaded_lift_performed=False,
        grip_holding_verified=False,holding_verified=False,offline_tests_executed=False,
        functional_validation='field_execution_pending',vision_used=False,
        vision_binding_scope='not_used_handeye_recalibration_deferred',
        camera_extrinsics_used_for_motion=False,previous_result_reused=False,
        historical_clearances_reused=False,automatic_old_point_alignment=False,
        automatic_lateral_motion=False,lateral_transfer_ready=False,
        hand_above_target_top_confirmed=False,autonomous_other_targets_ready=False,
        fresh_teaching_required=True,fresh_start_after_restart_allowed=True,
        onsite_hardware_stop_required_on_unexpected_contact=True,
        contact_sound_alone_aborts=False,temperature_sensor_monitoring=False)

    def save():
        temp=folder/'worker_result.tmp'
        temp.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf8')
        temp.replace(folder/'worker_result.json')

    robot=sequence=None
    fault_called=False

    def fault_once():
        nonlocal fault_called
        if sequence is None or fault_called:return
        fault_called=True
        try:
            # Inherited fault uses one Move_Stop attempt only after this run
            # attempted an arm command. No hand open/reset or return follows.
            sequence.fault()
        except BaseException as exc:
            report.setdefault('fault_errors',[]).append(dict(error=repr(exc),traceback=traceback.format_exc()))
            report.update(execution_ready=False,onsite_hardware_stop_required=True,
                automatic_open=False,automatic_return=False,automatic_force_zero=False)
            save()

    def cleanup_native():
        nonlocal robot
        if robot is None:return
        handle,robot=robot,None
        for name in ('Arm_Socket_Close','RM_API_UnInit'):
            try:
                getattr(handle,name)()
            except BaseException as exc:
                report['cleanup_errors'].append(dict(method=name,error=repr(exc),
                    traceback=traceback.format_exc()))
                fault_once()

    save()
    try:
        resume=None
        if args.resume_before_grasp:
            from supported_grasp_resume import load_resume_source
            resume=load_resume_source(args.resume_before_grasp)
            report.update(resume_before_grasp=resume,previous_result_reused=True,
                fresh_teaching_required=False,resume_source_is_not_motion_authorization=True)
            save()
        from battery_grasp.taught_dh_model import EXPECTED_DH
        from pregrasp_measurement_worker import source_audit
        from supported_contact_v6_sequence import SupportedLoadedCycle
        from supported_loaded_hand import MovingLoadedHand,SupportedOpenHand
        from supported_known_read_pacing import KnownReadHand as TrialHand
        from supported_known_read_pacing import (KnownReadTransport as LoadedCycleTransport,
            KnownReadAuthority as LoadedContactAuthority)
        from position_grasp_release_worker import ReleaseOnlyTransport
        from school_battery_transport import WRITE
        from inspect_left_hand import connect
        from arm_protection_transport import ProtectionClient
        from taught_model_readonly import collect_model_parameters
        from high_trial_tip_readonly import collect
        import fcntl
        import socket

        report['source_audit']=source_audit(args.sdk_dir)
        save()
        profile=json.loads((ROOT/'deployment/battery_hand_verified_profile.json').read_bytes())

        class MovingReadOnlyTransport(LoadedCycleTransport):
            """Final send boundary denies every write during arm load motion."""
            def exchange(self,message):
                if not isinstance(message,dict) or message.get('command')==WRITE:
                    raise RuntimeError('Loaded-motion hand monitor has no write capability')
                return super().exchange(message)

            def set_phase(self,*unused,**options):
                raise RuntimeError('Loaded-motion hand monitor cannot authorize a write phase')

            def enable_motion(self):
                raise RuntimeError('Loaded-motion hand monitor cannot enable hand writes')

            def write_group(self,*unused,**options):
                raise RuntimeError('Loaded-motion hand monitor cannot write hand targets')

        def append(stream,row):
            stream.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
            stream.flush()

        (ROOT/'captures').mkdir(exist_ok=True)
        with (ROOT/'captures/hand_stage.lock').open('a') as lock, \
             (folder/'hand_wire.jsonl').open('w',encoding='utf8') as wire, \
             (folder/'protection_wire.jsonl').open('w',encoding='utf8') as protection_wire, \
             (folder/'tip_wire.jsonl').open('w',encoding='utf8') as tip_wire, \
             (folder/'samples.jsonl').open('w',encoding='utf8') as samples:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            model=collect_model_parameters(folder/'fresh_model')
            report['fresh_model']=model
            save()
            if (not model['dh_valid'] or not model['identity_valid'] or model['errors']
                    or [model['dh_response']['joint_'+str(i)] for i in range(1,7)]!=EXPECTED_DH):
                raise RuntimeError('Fresh actual identity/DH differs; no setters or motion')

            # One normal authority per invocation. The sequence may grant a
            # separate supported-opening capability only after its own gate.
            authority=LoadedContactAuthority()

            def hand_log(row):
                append(wire,row)
                if row.get('event')=='tx' and row.get('request',{}).get('command')==WRITE:
                    report['hand_register_writes']+=1
                    save()

            def hand_factory(active,guarded):
                if not guarded or active.io is not None:
                    raise RuntimeError('Single guarded hand session required')
                role=getattr(active,'hand_role','trial')
                if role=='opening':
                    if active.release_authority is None:
                        raise RuntimeError('Explicit independent supported-opening authority missing')
                    io=ReleaseOnlyTransport(socket.create_connection(('169.254.128.18',8080),timeout=3),
                        hand_log,active.release_authority)
                    active.io=io
                    return SupportedOpenHand(active,io,report,save,samples)
                if role=='moving':
                    io=MovingReadOnlyTransport(socket.create_connection(('169.254.128.18',8080),timeout=3),
                        hand_log,authority)
                    active.io=io
                    return MovingLoadedHand(active,io,report,save,samples)
                io=LoadedCycleTransport(socket.create_connection(('169.254.128.18',8080),timeout=3),
                    hand_log,authority)
                active.io=io
                return TrialHand(active,io,report,save,samples)

            robot=connect(args.sdk_dir)
            # Fresh preflight binds the initial reference. No historical source
            # report, old pose, image, clearance, or implicit alignment is used.
            sequence=SupportedLoadedCycle(robot,None,profile,report,save,samples,hand_factory,
                folder=folder,authority=authority,
                tip_reader=lambda:collect(lambda row:append(tip_wire,row)),
                protection_client=ProtectionClient(lambda row:append(protection_wire,row)),
                enable_arm_protection=False,resume_report=None,completed_corridor=None,
                taught_only=True,allow_protected_opening=True)
            sequence.grasp_resume=resume
            failed=False
            try:
                sequence.run()
            except BaseException:
                failed=True
                fault_once()
                raise
            finally:
                try:
                    try:
                        sequence.close_hand()
                    except BaseException as exc:
                        report['cleanup_errors'].append(dict(method='close_hand',error=repr(exc),
                            traceback=traceback.format_exc()))
                        fault_once()
                        if not failed:raise
                finally:
                    # Keep diagnostic streams alive through SDK cleanup, so a
                    # cleanup fault can still log its single stop attempt.
                    cleanup_native()
    except BaseException as exc:
        fault_once()
        report['exceptions'].append(dict(error=repr(exc),traceback=traceback.format_exc()))
        report.update(outcome='aborted_no_further_writes',phase='fault',execution_ready=False,
            automatic_open=False,automatic_return=False,automatic_force_zero=False,
            onsite_hardware_stop_required=True)
    finally:
        # Also handles an exception during construction, before a sequence can
        # attempt any arm command. Normal sequence cleanup ran inside the logs.
        cleanup_native()
        if report['cleanup_errors']:
            report.update(outcome='failed_cleanup',execution_ready=False)
        save()
    if report['outcome']==SUCCESS and not report['exceptions'] and not report['cleanup_errors']:
        return 0
    if report['outcome'] in (SUPPORTED_ONLY,OPENING_ONLY) and not report['exceptions'] and not report['cleanup_errors']:
        # A supported release may finish safely while loaded motion remains
        # unperformed. The parent must preserve this distinct non-pass result.
        return 2
    return 1


if __name__=='__main__':raise SystemExit(main())
