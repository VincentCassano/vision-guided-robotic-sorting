#!/usr/bin/env python3
"""Onsite supported visual descent/grasp/release cycle, isolated native SDK."""
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
import sys
import traceback
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/battery_grasp/src'))
from battery_grasp.taught_dh_model import EXPECTED_DH
from vision_cycle_supported_release import SupportedCycle

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--folder',required=True,type=Path)
    p.add_argument('--sdk-dir',type=Path,default=Path('/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package'))
    p.add_argument('--execute',action='store_true')
    p.add_argument('--enable-arm-protection',action='store_true')
    p.add_argument('--resume-report',type=Path)
    p.add_argument('--reuse-grasp-report',type=Path)
    p.add_argument('--reuse-release-report',type=Path)
    p.add_argument('--follow-vision-report',type=Path)
    p.add_argument('--saved-goal-report',type=Path)
    p.add_argument('--clearance-return-report',type=Path)
    p.add_argument('--high-return-report',type=Path)
    p.add_argument('--resume-terminal-report',type=Path)
    p.add_argument('--confirm-world-x-10mm',action='store_true')
    p.add_argument('--confirm-groove-spacing-116mm',action='store_true')
    p.add_argument('--raise-for-vision-40mm',action='store_true')
    p.add_argument('--fixture-indexed-116mm',action='store_true')
    p.add_argument('--taught-only',action='store_true')
    p.add_argument('--initialize-hand-modbus',action='store_true')
    p.add_argument('--allow-supported-opening',action='store_true')
    args=p.parse_args(argv)
    reuse_corridor=args.reuse_grasp_report is not None or args.reuse_release_report is not None
    follow_vision=args.follow_vision_report is not None
    high_return=args.high_return_report is not None
    terminal_resume=args.resume_terminal_report is not None
    if args.saved_goal_report is not None and (not args.taught_only or not args.allow_supported_opening
            or reuse_corridor or follow_vision or args.resume_report is not None
            or args.initialize_hand_modbus or args.enable_arm_protection or args.clearance_return_report is not None or high_return or terminal_resume):
        p.error('Saved-goal automatic100 requires taught-only/supported-opening and prohibits other source/configuration modes; no connection')
    if args.clearance_return_report is not None and (not args.taught_only or not args.allow_supported_opening
            or reuse_corridor or follow_vision or args.resume_report is not None or args.saved_goal_report is not None
            or args.initialize_hand_modbus or args.enable_arm_protection or high_return or terminal_resume):
        p.error('Clearance-only requires taught-only/supported-opening and prohibits other source/configuration modes; no connection')
    if high_return and (args.taught_only or not args.allow_supported_opening or reuse_corridor or follow_vision
            or args.resume_report is not None or args.saved_goal_report is not None or args.clearance_return_report is not None
            or args.initialize_hand_modbus or args.enable_arm_protection or terminal_resume):
        p.error('High-return visual cycle prohibits other source/configuration modes and requires supported-opening; no connection')
    if terminal_resume and (args.taught_only or not args.allow_supported_opening or reuse_corridor or follow_vision
            or args.resume_report is not None or args.saved_goal_report is not None or args.clearance_return_report is not None
            or high_return or args.initialize_hand_modbus or args.enable_arm_protection):
        p.error('Terminal continuation prohibits other source/configuration modes and requires supported-opening; no connection')
    if args.confirm_world_x_10mm:
        p.error('World +X 10mm is withdrawn for the grooved fixture; use a complete left/right adjacent groove 116mm away with unchanged support height; no connection')
    if (args.reuse_grasp_report is None)!=(args.reuse_release_report is None):
        p.error('Completed corridor source reports must be paired')
    if reuse_corridor and (args.resume_report is not None or args.taught_only):
        p.error('Completed corridor reuse cannot combine with old numerical resume or fresh teaching-only mode')
    if follow_vision and (reuse_corridor or args.resume_report is not None or args.taught_only):
        p.error('Local visual follow cannot combine with source-pair reuse, numerical resume or taught-only mode')
    if follow_vision and not (args.confirm_groove_spacing_116mm and args.allow_supported_opening):
        p.error('Local visual follow requires explicit 116mm complete-adjacent-groove spacing and supported-opening authorization')
    if follow_vision and (args.initialize_hand_modbus or args.enable_arm_protection):
        p.error('Local visual follow prohibits bus/protection configuration writes; read existing accepted configuration only')
    if args.confirm_groove_spacing_116mm and not follow_vision:p.error('116mm groove-spacing confirmation requires follow source')
    if args.raise_for_vision_40mm and not follow_vision:p.error('Camera-clearance lift applies only to explicit visual-follow mode; no connection')
    if args.fixture_indexed_116mm and not follow_vision:p.error('Fixture-indexed 116mm positioning requires explicit visual-follow source, spacing and supported-opening authorization; no connection')
    if args.taught_only and args.resume_report:p.error('Taught-only must start fresh; no resume')
    folder=args.folder;folder.mkdir(parents=True,exist_ok=True)
    if not args.execute:p.error('Explicit supported visual grasp cycle required')
    report=dict(schema_version=1,scope='same_taught_shell_visual_descent_supported_grasp_release_empty_return',
        timestamp_utc=datetime.now(timezone.utc).isoformat(),outcome='starting',phase='preflight',
        diagnostic_pass=False,execution_ready=False,full_pregrasp_execution_ready=False,
        sdk_in_parent=False,exceptions=[],cleanup_errors=[],arm_actuator_writes=0,
        hand_register_writes=0,controller_configuration_writes=0,physical_stop_verified=False,
        torque_release_verified=False,automatic_open=False,automatic_return=False,
        automatic_grip=False,object_lift=False,offline_tests_executed=False,
        functional_validation='field_execution_pending',vision_binding_scope='same_current_stationary_shell_only',
        hand_above_target_top_confirmed=False,lateral_transfer_ready=False)
    report['hand_register_writes_count_kind']='raw_TX_attempts_not_acknowledged_or_stopped'
    report.update(hand_bus_initialization_requested=args.initialize_hand_modbus,hand_bus_configuration_writes=0)
    report['explicit_supported_opening_requested']=args.allow_supported_opening
    report['raise_for_vision_40mm_requested']=args.raise_for_vision_40mm
    report['fixture_indexed_116mm_requested']=args.fixture_indexed_116mm
    report['saved_goal_automatic100_requested']=args.saved_goal_report is not None
    report['clearance_return_requested']=args.clearance_return_report is not None
    report['high_return_requested']=high_return
    report['terminal_resume_requested']=terminal_resume
    report['hand_bus_configuration_writes_count_kind']='raw_TX_attempts_not_ACK_or_physical_stop'
    if args.taught_only:
        report.update(scope='fresh_taught_shell_descent_supported_grasp_release_empty_return',
            vision_binding_scope='not_used_taught_only',vision_used=False,
            camera_extrinsics_used_for_motion=False)
    elif reuse_corridor:
        report.update(scope='same_saved_corridor_shell_visual_descent_supported_grasp_release_empty_return',
            same_original_unmoved_shell_only=True,automatic_other_slot_transfer=False,
            completed_corridor_reuse_requested=True)
    elif follow_vision:
        report.update(scope='qualified_adjacent_groove_116mm_visual_follow_cycle',
            follow_vision_requested=True,measured_adjacent_groove_spacing_mm=116,
            independent_onsite_shift_direction_required=True,adjacent_groove_same_height_and_pose_required=True,
            vision_binding_scope='same_shell_complete_adjacent_groove_signed_WorldX116mm_same_height_only',
            camera_global_extrinsic_accuracy_certified=False,autonomous_other_targets_ready=False)
    if args.clearance_return_report is not None:
        report['scope']='empty_vertical_clearance_from_completed_taught_cycle'
    if high_return:
        report['scope']='same_qualified_upper_new_wrist_visual_grasp_cycle'
    if terminal_resume:
        report['scope']='continued_same_slot_visual_grasp_release_empty_return'
    def save():
        temp=folder/'worker_result.tmp'
        temp.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf8')
        temp.replace(folder/'worker_result.json')
    robot=seq=None;save()
    try:
        context=None;follow_context=None;saved_goal_context=None;clearance_context=None;high_return_context=None;terminal_context=None;sequence_class=SupportedCycle
        if terminal_resume:
            from vision_terminal_resume_cycle import TerminalResumeCycle,read_terminal_context
            from return_after_release_worker import strict_json
            if args.resume_terminal_report.resolve()!=(folder/'terminal_resume_source/worker_result.json').resolve():
                raise RuntimeError('Terminal continuation must use the exact parent-captured source bytes')
            profile=strict_json((ROOT/'deployment/battery_hand_verified_profile.json').read_bytes())
            terminal_context=read_terminal_context(folder,profile)
            sequence_class=TerminalResumeCycle
            report['terminal_resume_source_context_accepted']=True;save()
        if high_return:
            from vision_high_return_cycle import HighReturnCycle,read_high_return_context
            from return_after_release_worker import strict_json
            if args.high_return_report.resolve()!=(folder/'high_return_source/worker_result.json').resolve():
                raise RuntimeError('High-return continuation must use the exact parent-captured source bytes')
            profile=strict_json((ROOT/'deployment/battery_hand_verified_profile.json').read_bytes())
            high_return_context=read_high_return_context(folder,profile)
            sequence_class=HighReturnCycle
            report['high_return_source_context_accepted']=True;save()
        if args.clearance_return_report is not None:
            from vision_clearance_return_cycle import ClearanceReturnCycle,read_clearance_context
            from return_after_release_worker import strict_json
            if args.clearance_return_report.resolve()!=(folder/'clearance_source/worker_result.json').resolve():
                raise RuntimeError('Clearance continuation must use the exact parent-captured source bytes')
            profile=strict_json((ROOT/'deployment/battery_hand_verified_profile.json').read_bytes())
            clearance_context=read_clearance_context(folder,profile)
            sequence_class=ClearanceReturnCycle
            report['clearance_source_context_accepted']=True;save()
        if args.saved_goal_report is not None:
            from vision_saved_goal_cycle import SavedGoalCycle,read_saved_goal_context
            from return_after_release_worker import strict_json
            if args.saved_goal_report.resolve()!=(folder/'saved_goal_source/worker_result.json').resolve():
                raise RuntimeError('Saved goal must use the exact parent-captured source bytes')
            profile=strict_json((ROOT/'deployment/battery_hand_verified_profile.json').read_bytes())
            saved_goal_context=read_saved_goal_context(folder,profile)
            sequence_class=SavedGoalCycle
            report['saved_goal_source_context_accepted']=True;save()
        if follow_vision:
            from return_after_release_worker import strict_json
            from vision_follow_cycle import FollowCycle,read_follow_source
            if args.follow_vision_report.resolve()!=(folder/'follow_source/worker_result.json').resolve():
                raise RuntimeError('Visual-follow source must be the parent-captured exact source bytes')
            profile=strict_json((ROOT/'deployment/battery_hand_verified_profile.json').read_bytes())
            follow_context=read_follow_source(folder,profile)
            sequence_class=FollowCycle
            report.update(follow_source_context_accepted=True,follow_source_records=follow_context['source_records'])
            save()
        if reuse_corridor:
            from return_after_release_worker import source_context,strict_json
            for role in ('grasp','release'):
                if getattr(args,'reuse_'+role+'_report').resolve()!=(folder/'sources'/(role+'_worker_result.json')).resolve():
                    raise RuntimeError('Completed corridor must use the parent-captured exact source bytes')
            profile=strict_json((ROOT/'deployment/battery_hand_verified_profile.json').read_bytes())
            context=source_context(folder,profile)
            original=context['grasp']
            if (type(original.get('controller_configuration_writes')) is not int
                    or original['controller_configuration_writes']!=2
                    or type(original.get('hand_bus_configuration_writes')) is not int
                    or original['hand_bus_configuration_writes']!=1):
                raise RuntimeError('Original descent configuration/bus write counts differ from the reviewed source')
            report.update(source_context_accepted=True,source_records=context['source_records'])
            save()
        from pregrasp_measurement_worker import source_audit
        report['source_audit']=source_audit(args.sdk_dir);save()
        if context is not None and report['source_audit']!=context['grasp']['source_audit']:
            raise RuntimeError('Actual SDK source/native payload differs from original completed corridor; no connection')
        if follow_context is not None and report['source_audit']!=follow_context['worker']['source_audit']:
            raise RuntimeError('Actual SDK source/native payload differs from successful visual-follow baseline; no connection')
        if saved_goal_context is not None and report['source_audit']!=saved_goal_context['worker']['source_audit']:
            raise RuntimeError('Actual SDK source/native payload differs from saved taught goal; no connection')
        if clearance_context is not None and report['source_audit']!=clearance_context['worker']['source_audit']:
            raise RuntimeError('Actual SDK source/native payload differs from completed taught cycle; no connection')
        if high_return_context is not None and report['source_audit']!=high_return_context['worker']['source_audit']:
            raise RuntimeError('Actual SDK source/native payload differs from completed high-return ascent; no connection')
        if terminal_context is not None and report['source_audit']!=terminal_context['worker']['source_audit']:
            raise RuntimeError('Actual SDK source/native payload differs from completed visual descent; no connection')
        import fcntl,socket
        from inspect_left_hand import connect
        from vision_cycle_hand import CycleHand
        from vision_cycle_transport import CycleTransport
        from supported_contact_transport import PhaseAuthority
        from arm_protection_transport import ProtectionClient
        from taught_model_readonly import collect_model_parameters
        from high_trial_tip_readonly import collect
        from hand_bus_startup import HandBusStartup,REQUEST as BUS_REQUEST
        from high_descent_prepare_worker import same_pose
        # Bind the first fresh snapshot in preflight. Historical measured poses
        # and clearances do not authorize this session or cause an old-point move.
        reference=None
        def append(out,row):out.write(json.dumps(row)+'\n');out.flush()
        with (ROOT/'captures/hand_stage.lock').open('a') as lock, \
             (folder/'hand_wire.jsonl').open('w',encoding='utf8') as wire, \
             (folder/'protection_wire.jsonl').open('w',encoding='utf8') as protection_wire, \
             (folder/'tip_wire.jsonl').open('w',encoding='utf8') as tip_wire, \
             (folder/'hand_bus_wire.jsonl').open('w',encoding='utf8') as bus_wire, \
             (folder/'samples.jsonl').open('w',encoding='utf8') as samples:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            model=collect_model_parameters(folder/'fresh_model')
            report['fresh_model']=model;save()
            if (not model['dh_valid'] or not model['identity_valid'] or model['errors']
                    or [model['dh_response']['joint_'+str(i)] for i in range(1,7)]!=EXPECTED_DH):
                raise RuntimeError('Fresh actual identity/DH differs; no setters or motion')
            if context is None and follow_context is None and saved_goal_context is None and clearance_context is None and high_return_context is None and terminal_context is None:
                profile=json.loads((ROOT/'deployment/battery_hand_verified_profile.json').read_bytes())
            authority=PhaseAuthority()
            def bus_log(row):
                append(bus_wire,row)
                if row.get('event')=='tx':
                    report['controller_configuration_writes']+=1
                    report['hand_bus_configuration_writes']+=1;save()
            bus=HandBusStartup(bus_log)
            def hand_log(row):
                append(wire,row)
                if row.get('event')=='tx' and row.get('request',{}).get('command')=='write_registers':
                    report['hand_register_writes']+=1;save()
            def factory(sequence,guarded):
                if not guarded or sequence.io is not None:raise RuntimeError('Single guarded hand session required')
                if args.initialize_hand_modbus and not bus.attempted:
                    phase=report['phase'];report['phase']='initializing_hand_modbus'
                    report['hand_bus_setup']=dict(request=BUS_REQUEST,attempted=False,
                        power_changed=False,hand_register_writes=False,identity_pending=True)
                    save()
                    before=sequence.full_arm()
                    expected=dict(before,joints=sequence.guard_target,pose=sequence.guard_pose)
                    if (not same_pose(before,expected)
                            or before['frame_signatures']!=sequence.reference['frame_signatures']
                            or before['limits']!=sequence.reference['limits']):
                        raise RuntimeError('Arm/frame/limits changed before bus configuration; no setup')
                    report['hand_bus_setup']['attempted']=True;save()
                    report['hand_bus_setup']['response']=bus.configure_once()
                    after=sequence.full_arm()
                    if not same_pose(after,before) or after['limits']!=before['limits']:
                        raise RuntimeError('Arm/configuration changed during bus setup; no hand target')
                    report['hand_bus_setup'].update(outcome='controller_ACK_only_fresh_hand_identity_pending',
                        getter_support='not_assumed_legacy_controller',physical_stop_verified=False)
                    report['phase']=phase;save()
                if (args.allow_supported_opening and report['phase'] in ('preflight_release','releasing')
                        and sequence.allow_protected_opening):
                    from position_grasp_release_worker import ReleaseOnlyTransport,ReleaseOnlyHand
                    sequence.io=ReleaseOnlyTransport(socket.create_connection(('169.254.128.18',8080),timeout=3),
                        hand_log,sequence.release_authority)
                    return ReleaseOnlyHand(sequence,sequence.io,report,save,samples)
                sequence.io=CycleTransport(socket.create_connection(('169.254.128.18',8080),timeout=3),hand_log,authority)
                return CycleHand(sequence,sequence.io,report,save,samples)
            robot=connect(args.sdk_dir)
            extra={'follow_context':follow_context,'raise_for_vision_40mm':args.raise_for_vision_40mm,
                'fixture_indexed_116mm':args.fixture_indexed_116mm} if follow_vision else {}
            if saved_goal_context is not None:extra={'saved_goal_context':saved_goal_context}
            if clearance_context is not None:extra={'clearance_context':clearance_context}
            if high_return_context is not None:extra={'high_return_context':high_return_context}
            if terminal_context is not None:extra={'terminal_context':terminal_context}
            seq=sequence_class(robot,reference,profile,report,save,samples,factory,folder=folder,authority=authority,
                tip_reader=lambda:collect(lambda row:append(tip_wire,row)),
                enable_arm_protection=args.enable_arm_protection,
                resume_report=args.resume_report,
                completed_corridor=context,
                taught_only=args.taught_only,
                allow_protected_opening=args.allow_supported_opening,
                protection_client=ProtectionClient(lambda row:append(protection_wire,row)),**extra)
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
    return 0 if report['outcome'] in ('completed_visual_descent_grasp_release_empty_return',
        'completed_taught_descent_grasp_release_empty_return',
        'completed_adjacent_groove_visual_follow_grasp_release_empty_return',
        'completed_visual_grasp_release_empty_return_from_saved_terminal',
        'completed_empty_vertical_retreat_top_clearance_confirmed',
        'completed_empty_vertical_retreat_clearance_insufficient',
        'completed_empty_vertical_retreat_measurement_pending') else 1


if __name__=='__main__':raise SystemExit(main())
