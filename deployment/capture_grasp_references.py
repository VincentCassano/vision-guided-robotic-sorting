#!/usr/bin/env python3
"""Record manually positioned, NONCONTACT hover references; no device writes.

The operator moves the arm only through its existing teaching controls, then
stops before requesting a capture. This program cannot stop or release hardware.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import socket
import time
import traceback

from capture_installed_hand_geometry import _arm, _json_safe, analyze_geometry
from inspect_left_arm import snapshot
from inspect_left_hand import connect
from run_school_battery_hand import Hand
from school_battery_transport import Transport
from sdk_environment import collect_sdk_environment

ROOT = Path(__file__).resolve().parents[1]
ROLES = {'s':'safe_reference','h':'pick_hover','p':'place_hover'}
MINIMUM_REPORTED_CLEARANCE_MM = 50  # Capture protocol choice, NOT device clearance certification.


def capture_point(robot, hand, report, save, role, clearance_mm, profile, baseline, hand_factory=None):
    if role not in ROLES.values():
        raise ValueError('Unknown reference role')
    if any(row.get('role')==role for row in report.get('points',[])):
        raise ValueError('Role already recorded; no overwrite')
    if type(clearance_mm) not in (int,float) or not math.isfinite(clearance_mm) or clearance_mm < MINIMUM_REPORTED_CLEARANCE_MM:
        raise ValueError('Measured whole-hand clearance must be >=50 mm; boolean/NaN rejected')
    if not isinstance(baseline,dict) or baseline.get('diagnostic_pass') is not True:
        raise ValueError('Passed installed-hand baseline required')
    reference = _arm(baseline['arm_query_brackets']['after']['queries'])
    row = dict(role=role,accepted=False,clearance_mm=clearance_mm,
               clearance_source='operator_reported_tape_measurement_not_independently_verified',
               timestamp_utc=datetime.now(timezone.utc).isoformat(),
               started_monotonic=time.monotonic(),arm_query_brackets={},execution_ready=False)
    report.setdefault('points',[]).append(row)
    save()
    try:
        before = snapshot(robot)
        row['arm_query_brackets']['before']={'queries':before}
        save()
        before_arm = _arm(before)
        if before_arm['frame_signatures'] != reference['frame_signatures'] or before_arm['limits'] != reference['limits']:
            raise RuntimeError('Frame or joint configuration changed since accepted baseline')
        if any(value < 3 for value in before_arm['reserve']):
            raise RuntimeError('Current joints have less than 3 degrees of reserve')
        # The controller broadcasts drag-teaching acknowledgments to connected
        # clients. Never keep this strict hand connection open while an operator
        # is teaching. Create it only AFTER stopped confirmation and SDK-before.
        if hand is None:
            if hand_factory is None:
                raise ValueError('Fresh read-only hand factory required')
            hand = hand_factory()
        # Reread configuration for each NEW reference, not only the initial point.
        hand.configuration()
        if report['configuration'] != profile['observed_configuration']:
            raise RuntimeError('Device configuration differs from frozen accepted profile')
        state = hand.preflight(expected=profile['poses']['open'])
        row['hand_preflight_before'] = state
        row['hand_angle_raw'] = hand.read('reference_angle_raw',1165,6)
        row['hand_force_feedback_raw'] = hand.read('reference_force_raw',1175,5)
        final_state = hand.preflight(expected=profile['poses']['open'])
        row['hand_preflight_after'] = final_state
        after = snapshot(robot)
        row['arm_query_brackets']['after']={'queries':after}
        analysis = analyze_geometry(before,after,final_state,profile)
        row['analysis'] = analysis
        if not analysis['diagnostic_pass'] or analysis['hand_pose_classification'] != 'open':
            raise RuntimeError('Reference bracket failed: '+str(analysis['errors']))
        for queries in (before,after):
            arm = _arm(queries)
            if arm['frame_signatures'] != reference['frame_signatures'] or arm['limits'] != reference['limits']:
                raise RuntimeError('Frame or joint configuration changed since accepted baseline')
        row['accepted'] = True
    except (Exception,KeyboardInterrupt):
        row['error'] = traceback.format_exc()
        raise
    finally:
        row['finished_monotonic'] = time.monotonic()
        save()
    return row


def compile_reference_draft(points):
    """Measured poses only: do not invent a descent or infer TCP/table geometry."""
    result = dict(schema_version=1,execution_ready=False,measured_geometry_complete=False,
                  references_complete=False,waypoints=[],
                  missing=['grasp_center_tool_offset','table_and_full_hand_swept_geometry',
                           'workspace_and_arm_joint_path_validation','contact_stop_release',
                           'actual_grip_and_held_object_validation'],
                  note='Different taught wrist orientations are retained. No interpolation, descent, grasp or hardware command is generated.')
    roles = set()
    for row in points:
        if not row.get('accepted'):
            continue
        role = row['role']
        if role not in ROLES.values() or role in roles:
            raise ValueError('Unknown or duplicate accepted role')
        analysis = row['analysis']
        if analysis.get('diagnostic_pass') is not True or analysis.get('hand_pose_classification') != 'open':
            raise ValueError('Accepted point lacks passed open-hand bracket')
        clearance = row.get('clearance_mm')
        if type(clearance) not in (int,float) or not math.isfinite(clearance) or clearance < 50:
            raise ValueError('Accepted point lacks reported clearance')
        arm = analysis['current_arm']
        roles.add(role)
        result['waypoints'].append(dict(role=role,tool_world_mm=list(arm['tcp_position_mm']),
            tool_rpy_rad=list(arm['tcp_rpy_rad']),joint_degrees=list(arm['joint_degrees']),
            reported_clearance_mm=clearance,frame='World',tool='Arm_Tip'))
    result['waypoints'].sort(key=lambda row:list(ROLES.values()).index(row['role']))
    result['references_complete'] = {'safe_reference','pick_hover'} <= roles
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--confirm-empty-clear-exclusive',action='store_true')
    parser.add_argument('--confirm-onsite-stop',action='store_true')
    parser.add_argument('--baseline',type=Path,default=Path(__file__).with_name('installed_hand_baseline_20261009.json'))
    parser.add_argument('--profile',type=Path,default=Path(__file__).with_name('battery_hand_verified_profile.json'))
    parser.add_argument('--sdk-dir',type=Path,default=Path('/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package'))
    args = parser.parse_args(argv)
    if not args.confirm_empty_clear_exclusive or not args.confirm_onsite_stop:
        parser.error('Requires empty clear hand, stopped competing scripts, and onsite teaching/hardware-stop operator')
    baseline = json.loads(args.baseline.read_text(encoding='utf-8'))
    profile = json.loads(args.profile.read_text(encoding='utf-8'))
    if baseline.get('diagnostic_pass') is not True or baseline.get('read_only') is not True:
        parser.error('Passed read-only installed-hand baseline required; no connection opened')
    _arm(baseline['arm_query_brackets']['after']['queries'])
    folder = ROOT/'captures'/('grasp_references_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True,exist_ok=False)
    for name in ('wire.jsonl','samples.jsonl'):
        (folder/name).touch(exist_ok=False)
    report = dict(schema_version=1,read_only=True,hardware_writes=0,execution_ready=False,
        outcome='starting',points=[],exceptions=[],cleanup_errors=[],
        timestamp_utc=datetime.now(timezone.utc).isoformat(),
        baseline_sha256=hashlib.sha256(args.baseline.read_bytes()).hexdigest(),
        profile_sha256=hashlib.sha256(args.profile.read_bytes()).hexdigest(),
        arm_connection_strategy='one_sdk_session_hand_connection_only_during_each_confirmed_capture',
        note='Manual teaching remains operator controlled. No stopping, release or continuous collision safety is certified.')
    names=('capture_grasp_references.py','sdk_environment.py','capture_installed_hand_geometry.py','inspect_left_arm.py',
           'inspect_left_hand.py','arm_sdk_reviewed_hashes.json','run_school_battery_hand.py',
           'school_battery_transport.py','run_school_battery.py','school_battery_pose.json')
    report['source_sha256']={name:hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest() for name in names}
    def save():
        temporary=folder/'result.tmp'
        temporary.write_text(json.dumps(_json_safe(report),indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8')
        temporary.replace(folder/'result.json')
    save()
    robot,io=None,None
    try:
        import fcntl
        with (ROOT/'captures/hand_stage.lock').open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            with (folder/'wire.jsonl').open('w',encoding='utf-8') as wire, (folder/'samples.jsonl').open('w',encoding='utf-8') as samples:
                def log(row):
                    wire.write(json.dumps(row)+'\n');wire.flush()
                try:
                    robot=connect(args.sdk_dir)
                    report['sdk_environment']=collect_sdk_environment(robot)
                    save()
                    print('只读记录，不控制机械臂或手。现场使用示教器手动定位，停稳后再采集。')
                    print('保持手张开。全部手部件到电池、桌面及外部障碍物的最小净空须>=50mm。接触立即现场硬件停止。')
                    print('s=安全观察参考位；h=电池正上方悬停位；p=可选投放上方悬停位；q=保存退出。')
                    print('测整手最低部件，不测掌心；h位以电池顶面为垂直间隙基准，侧面障碍也要检查。')
                    while True:
                        command=input('现场定位并停稳后选择 s/h/p，或 q：').strip().lower()
                        if command=='q':break
                        if command not in ROLES:
                            print('请输入 s/h/p/q。');continue
                        role=ROLES[command]
                        if any(row['role']==role for row in report['points']):
                            print('此位置已记录；不覆盖。');continue
                        text=input('用卷尺量整只手到最近外部物体的最小净空（mm）；不能确认则输入q退出：').strip().lower()
                        if text=='q':break
                        try:clearance=float(text)
                        except ValueError:
                            print('需要毫米数字。');continue
                        if not math.isfinite(clearance) or clearance<50:
                            print('此步骤要求>=50mm净空，不采集这个位置。');continue
                        if input('再次确认机械臂已停稳、手仍张开、全程无接触；输入s保存，其他输入取消：').strip().lower()!='s':
                            continue
                        def fresh_hand():
                            nonlocal io
                            io=Transport(socket.create_connection(('169.254.128.18',8080),timeout=3),log)
                            return Hand(io,report,save,samples)
                        try:
                            row=capture_point(robot,None,report,save,role,clearance,profile,baseline,hand_factory=fresh_hand)
                        finally:
                            if io is not None:
                                io.disable_motion()
                                try:
                                    io.sock.close()
                                except Exception as exc:
                                    report['cleanup_errors'].append(dict(method='hand.close',role=role,error=repr(exc)))
                                io=None
                                save()
                        if report['cleanup_errors']:
                            raise RuntimeError('Hand connection cleanup failed; no further captures')
                        print('已记录',role,'TCP mm:',row['analysis']['current_arm']['tcp_position_mm'])
                        print('现在可用示教器手动定位下一个参考位，程序不发运动指令。')
                    report['draft']=compile_reference_draft(report['points'])
                    report['outcome']='references_recorded' if report['draft']['references_complete'] else 'partial_references_saved'
                finally:
                    if io is not None:
                        io.disable_motion()
                        try:io.sock.close()
                        except Exception as exc:report['cleanup_errors'].append(dict(method='hand.close',error=repr(exc)))
                        io=None
                    if robot is not None:
                        for method in ('Arm_Socket_Close','RM_API_UnInit'):
                            try:getattr(robot,method)()
                            except Exception as exc:report['cleanup_errors'].append(dict(method=method,error=repr(exc)))
                        robot=None
    except (Exception,KeyboardInterrupt) as exc:
        report['exceptions'].append(dict(error=repr(exc),traceback=traceback.format_exc()))
        report['outcome']='aborted_no_device_writes'
    finally:
        if report['cleanup_errors']:report['outcome']='failed_cleanup'
        report['execution_ready']=False
        save()
        print('SAVED:',folder,'OUTCOME:',report['outcome'])
        print('No automatic motion, return, release or stop. These are recorded references only.')
    return 0 if report['outcome']=='references_recorded' else 1


if __name__=='__main__':
    raise SystemExit(main())
