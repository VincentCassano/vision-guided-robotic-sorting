#!/usr/bin/env python3
"""Continuous accepted empty-hand cycle; optional measured observation-to-hover arm move.

No battery contact, reverse route, IK, configuration setters or hardware speed change.
The strict transport quiet/fence policy is unchanged. Onsite hardware stop remains
the contact response; normal end-of-cycle opening is not a fault release.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import logging
import math
from pathlib import Path
import socket
import time
import traceback
import zipfile

import run_arm_hand_sequence as baseline
import run_arm_hand_full_linkage as measured
from arm_protection_transport import ProtectionClient
from capture_installed_hand_geometry import _json_safe, _vector
from inspect_left_hand import connect
from continuous_hand_cycle import ContinuousHand
from run_school_battery_hand import ADAPTED_PLAN, TOL, TARGET_TOL, IDENTITY

ROOT = Path(__file__).resolve().parents[1]
LATEST_NAME = 'arm_hand_full_linkage_latest_112051.json'
LATEST_SHA = 'aabaf50192686262d64c8255d6c1a7849392d25aa477da2f6b3fbec9a31c1b73'
SOURCE_NAMES = measured.SOURCE_NAMES + ('run_arm_hand_continuous.py',
    'continuous_hand_cycle.py', LATEST_NAME)
OPEN_STEPS = [dict(phase='release_thumb_flex', address=1135, count=1, values=[0]),
    dict(phase='release_four_fingers', address=1136, count=4, values=[0]*4),
    dict(phase='return_thumb_side_palm', address=1140, count=1, values=[728])]


def accepted_completion(document, profile, hover):
    """Bind the cycle to the already completed live linkage, not another trial."""
    if (document.get('schema_version') != 1
            or document.get('outcome') != 'completed_full_linkage_open'
            or document.get('preflight_pass') is not True
            or document.get('arm_move_feedback_pass') is not True
            or document.get('empty_hand_full_linkage_feedback_pass') is not True
            or document.get('verified_hand_sequence_completed') is not True
            or document.get('hand_open_feedback_pass') is not True
            or document.get('completed_adapted_stages') != 3
            or document.get('exceptions') or document.get('cleanup_errors')
            or document.get('reference_sha256') != baseline.REFERENCE_SHA
            or document.get('configuration') != profile['observed_configuration']):
        raise ValueError('Require pinned successful112051 linkage and frozen device configuration')
    def u16(values, count):
        return (isinstance(values, list) and len(values) == count
                and all(type(v) is int and 0 <= v <= 65535 for v in values))
    def feedback(row):
        return (u16(row.get('positions'), 6) and u16(row.get('current'), 6)
                and u16(row.get('status'), 6)
                and all(v in (0, 1, 2, 5) for v in row['status'])
                and all(c <= lim for c, lim in zip(row['current'],
                    profile['observed_configuration']['current_limit_ma'])))
    moves = document.get('moves', [])
    previous = profile['poses']['open'][:]
    if len(moves) != 6:
        raise ValueError('Complete three grasp and three normal opening stages required')
    for row, step in zip(moves, ADAPTED_PLAN + OPEN_STEPS):
        channel = step['address'] - 1135
        previous[channel:channel+step['count']] = step['values']
        if (row.get('step') != step or row.get('expected') != previous
                or row.get('outcome') != 'reached_and_observed'
                or not feedback(row.get('snapshot', {}))
                or any(abs(a-b) > TOL for a, b in zip(row['snapshot']['positions'], previous))):
            raise ValueError('Successful linkage stage evidence mismatch')
    final = document.get('final_hand', {})
    if (not feedback(final) or not u16(final.get('targets'), 6)
            or not u16(final.get('identity'), 8) or final['identity'][:6] != IDENTITY
            or final['identity'][6] != 0
            or any(abs(a-b) > TARGET_TOL for a, b in zip(final['targets'], previous))
            or any(abs(a-b) > TOL for a, b in zip(final['positions'], previous))
            or final.get('speeds') != [65535]*6 or final.get('force_targets') != [0]*5):
        raise ValueError('Successful linkage did not finish at the frozen open profile')
    raw = document.get('fresh_protection_values', {})
    if raw.get('Get_Collision_Stage') != [0, 8] or raw.get('Get_Self_Collision_Enable') != [0, True]:
        raise ValueError('Require audited raw8/true protection evidence')
    last = document.get('last_arm_state', {})
    baseline.check_resumed_reference(dict(hover,
        joints=_vector(last.get('joints'), 6, 'successful endpoint joints'),
        pose=_vector(last.get('pose'), 6, 'successful endpoint pose')), hover)
    return True


def classify_endpoint(arm, observation, hover):
    _vector(arm.get('joints'), 6, 'live endpoint joints')
    _vector(arm.get('pose'), 6, 'live endpoint pose')
    matches = []
    for role, reference in (('observation', observation), ('pick_hover', hover)):
        try:
            baseline.check_resumed_reference(arm, reference)
        except (ValueError, RuntimeError):
            continue
        matches.append(role)
    if len(matches) != 1:
        raise RuntimeError('Arm is outside both accepted endpoints; no automatic alignment or return')
    return matches[0]


class ContinuousSequence(measured.FullLinkageSequence):
    def __init__(self, *args, target_reference, latest_document, **kwargs):
        super().__init__(*args, target_reference=target_reference,
                         latest_document=latest_document, **kwargs)
        self.observation_reference = self.reference
        self.hand_motion_enabled = False
        self.require_open = True
        self.start_hand_target = self.profile['poses']['open'][:]
        self.report.update(execution_scope='accepted_empty_hand_continuous_cycle',
            accepted_full_linkage_capture='arm_hand_full_linkage_20261010_112051_114028',
            prior_full_linkage_feedback_pass=True, current_role=None,
            required_speeds=[65535]*6, automatic_return=False,
            repeated_acceptance_test_required=False, timing_live_verified=False)

    def fresh_arm(self, expected_role=None):
        self.report['preflight_pass'] = False
        metadata = baseline.collect_sdk_environment(self.robot)
        self.report['sdk_environment'] = metadata
        if (metadata.get('metadata_complete') is not True
                or metadata.get('loaded_native', {}).get('sha256') != baseline.NATIVE_SHA
                or metadata.get('api_version') != '4.3.1'
                or metadata.get('host_machine') != 'aarch64'
                or metadata.get('controller_run_mode_raw', {}).get('data') != [1]
                or metadata.get('controller_software_info', {}).get('data', [{}])[0].get('product_version') != 'RM65-BI'):
            raise RuntimeError('Live SDK/controller identity or real mode differs from accepted device')
        arm = self.full_arm()
        role = classify_endpoint(arm, self.observation_reference, self.target_reference)
        if expected_role is not None and role != expected_role:
            raise RuntimeError('Arm endpoint changed during operator input; no writes')
        self.reference = self.observation_reference if role == 'observation' else self.target_reference
        self.safety_reads()
        self.settled(self.reference['joints'])
        arm = self.full_arm()
        baseline.check_resumed_reference(arm, self.reference)
        self.guard_target, self.guard_pose = arm['joints'][:], arm['pose'][:]
        self.report.update(current_role=role, arm_preflight_pass=True,
                           arm_hand_phase_pose=arm['pose'][:])
        self.save()
        return arm, role

    def fresh_hand_open(self):
        hand = self.hand_factory(self, True)
        try:
            hand.configuration()
            if self.report['configuration'] != self.profile['observed_configuration']:
                raise RuntimeError('Current hand configuration differs from frozen measured profile')
            state = hand.preflight(expected=self.start_hand_target)
            self.report['preflight_pass'] = True
            self.report['initial_hand_classification'] = 'open'
            self.report['guarded_readonly_transport_preflight_pass'] = True
            self.save()
            return state
        finally:
            self.close_hand()

    def run(self, execute):
        if not execute or self.enable_arm_protection:
            raise RuntimeError('Explicit empty-hand execution required; no protection setters')
        print('读取设备身份、臂保护和当前端点；随后核对空手配置。', flush=True)
        _, role = self.fresh_arm()
        self.fresh_hand_open()
        print('112051整体联动已通过。本入口执行已验收的3段手型和3段正常张开；中途不用逐段按键。', flush=True)
        print('接触硬物立即现场硬件急停，再安全释放。Ctrl+C锁止后续写入，无故障自动张开或回臂。', flush=True)
        if role == 'observation':
            answer = self.confirm('现场人员已守急停、空手且无人同时控制；已取走电池，整臂/手/线缆全路径无遮挡且净空>=100mm；m=一次5%移臂到已测hover，其他输入退出：').strip().lower()
            if answer != 'm':
                self.report['outcome'] = 'operator_exit_no_writes'
                return
            fresh, _ = self.fresh_arm(expected_role='observation')
            self.fresh_hand_open()
            target = self.target_reference['joints'][:]
            self.report['candidate'] = measured.make_route(fresh, self.target_reference)
            self.report['onsite_full_route_confirmation'] = True
            self.report['operator_reported_path_free'] = True
            self.report['operator_reported_battery_removed'] = True
            self.report['battery_removal_and_route_clearance_independently_detected'] = False
            self.arm_command_attempted = True
            self.report['arm_command_attempted'] = True
            self.save()
            status = self.call('Movej_Cmd', target, 5, trajectory_connect=0, r=0, block=False)
            if type(status) is not int or status != 0:
                raise RuntimeError('Movej rejected/uncertain; no retry or return')
            joints, pose = self.settled(target, travel_start=fresh['joints'])
            endpoint = self.full_arm()
            baseline.check_resumed_reference(endpoint, self.target_reference)
            self.report.update(arm_move_feedback_pass=True, hover_endpoint_feedback_pass=True,
                actual_joint_delta_degrees=[b-a for a, b in zip(fresh['joints'], joints)],
                actual_endpoint_tcp_distance_mm=math.dist(fresh['pose'][:3], pose[:3])*1000)
            self.save()
        else:
            self.report['arm_move_action'] = 'already_at_accepted_hover_no_repeat_move'
            print('当前位置已是验收后的hover；保持机械臂位置，不要求回起点重测。', flush=True)
        answer = self.confirm('现场确认臂实际停稳、空手、完整手型范围无接触且净空>=100mm、无人同时控制并全程守急停；h=连续3段抓握手型后3段正常张开，其他输入退出：').strip().lower()
        if answer != 'h':
            self.report['outcome'] = 'operator_exit_after_arm_move' if self.arm_command_attempted else 'operator_exit_no_writes'
            return
        self.report['onsite_continuous_hand_confirmation'] = True
        print('确认后核对当前臂与手参数，开始连续周期。', flush=True)
        self.fresh_arm(expected_role='pick_hover')
        hand = self.hand_factory(self, True)
        began = self.clock()
        try:
            hand.configuration()
            if self.report['configuration'] != self.profile['observed_configuration']:
                raise RuntimeError('Hand changed during confirmation; no write')
            hand.preflight(expected=self.start_hand_target)
            self.report['preflight_pass'] = True
            self.io.enable_motion()
            self.hand_motion_enabled = True
            final = hand.execute_cycle(self.start_hand_target[:])
            self.report['final_hand'] = final
        except (Exception, KeyboardInterrupt):
            self.io.disable_motion()
            raise
        finally:
            self.hand_motion_enabled = False
            self.close_hand()
            self.report['continuous_hand_phase_elapsed_seconds'] = self.clock()-began
        self.settled(self.target_reference['joints'])
        baseline.check_resumed_reference(self.full_arm(), self.target_reference)
        self.report.update(outcome='completed_continuous_hand_open',
            verified_hand_sequence_completed=True, hand_open_feedback_pass=True,
            empty_hand_full_linkage_feedback_pass=bool(self.arm_command_attempted),
            prior_full_linkage_feedback_pass=True, full_pick_path_ready=False,
            whole_route_geometry_certified=False, physical_stop_verified=False,
            torque_release_verified=False, timing_live_verified=True)
        self.save()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--confirm-empty-clear-exclusive', action='store_true')
    parser.add_argument('--confirm-onsite-stop', action='store_true')
    parser.add_argument('--sdk-dir', type=Path,
        default=Path('/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package'))
    args = parser.parse_args(argv)
    if not args.execute or not args.confirm_empty_clear_exclusive or not args.confirm_onsite_stop:
        parser.error('Requires execute, empty clear exclusive hand and onsite stop; no connection opened')
    directory = Path(__file__).resolve().parent
    def pinned(name, digest):
        path = directory/name
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            parser.error('Audited source mismatch: {}; no connection opened'.format(name))
        return json.loads(path.read_text(encoding='utf-8'))
    latest = pinned(LATEST_NAME, LATEST_SHA)
    previous = pinned(measured.LATEST_NAME, measured.LATEST_SHA)
    references = pinned('grasp_references_accepted_20261009.json', baseline.REFERENCE_SHA)
    profile_sha = references['profile_sha256']
    profile = pinned('battery_hand_verified_profile.json', profile_sha)
    try:
        safe = baseline.accepted_reference(references, profile)
        observation = measured.accepted_latest_reference(previous, safe, profile)
        hover = measured.accepted_pick_reference(references, profile, observation)
        if latest.get('profile_sha256') != profile_sha:
            raise ValueError('Successful linkage profile mismatch')
        accepted_completion(latest, profile, hover)
    except (ValueError, RuntimeError, KeyError, TypeError, IndexError) as exc:
        parser.error('Unaccepted completed linkage evidence: {}; no connection opened'.format(exc))
    folder = ROOT/'captures'/('arm_hand_continuous_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    report = dict(schema_version=1, outcome='starting', moves=[], exceptions=[], cleanup_errors=[],
        preflight_pass=False, arm_command_attempted=False, verified_hand_sequence_completed=False,
        physical_stop_verified=False, torque_release_verified=False, full_pick_path_ready=False,
        whole_route_geometry_certified=False, timestamp_utc=datetime.now(timezone.utc).isoformat(),
        latest_report_sha256=LATEST_SHA, reference_sha256=baseline.REFERENCE_SHA, profile_sha256=profile_sha,
        sdk_wire_capture_available=False, arm_protection_wire_capture_available=True,
        sdk_console_info_suppressed=True, hand_exchange_count=0, hand_exchange_elapsed_seconds=0.,
        source_sha256={name: hashlib.sha256((directory/name).read_bytes()).hexdigest() for name in SOURCE_NAMES})
    def save():
        temp = folder/'result.tmp'
        temp.write_text(json.dumps(_json_safe(report), ensure_ascii=False, indent=2,
                                   allow_nan=False)+'\n', encoding='utf-8')
        temp.replace(folder/'result.json')
    save()
    robot = sequence = None
    logger = logging.getLogger('battery_hand_readonly_sdk.robotic_arm')
    prior_level = logger.level
    try:
        import fcntl
        with (folder/'wire.jsonl').open('w', encoding='utf-8') as wire, \
             (folder/'arm_protection_wire.jsonl').open('w', encoding='utf-8') as protection_wire, \
             (folder/'samples.jsonl').open('w', encoding='utf-8') as samples, \
             (ROOT/'captures/hand_stage.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
            robot = connect(args.sdk_dir)
            logger.setLevel(logging.WARNING)
            def protection_log(row):
                protection_wire.write(json.dumps(row)+'\n'); protection_wire.flush()
            protection = ProtectionClient(protection_log)
            def factory(seq, guarded):
                if not guarded or seq.io is not None:
                    raise RuntimeError('Continuous cycle requires one guarded hand connection')
                def log(row):
                    wire.write(json.dumps(row)+'\n'); wire.flush()
                    if row.get('event') == 'tx':
                        report['hand_exchange_count'] += 1
                    if row.get('event') == 'response':
                        report['hand_exchange_elapsed_seconds'] += row.get('elapsed_s', 0.)
                seq.io = baseline.SequenceTransport(socket.create_connection(
                    ('169.254.128.18', 8080), timeout=3), log)
                return ContinuousHand(seq, seq.io, report, save, samples)
            def confirm(text):
                before = time.monotonic()
                answer = input(text)
                sequence.event('operator_input', prompt=text, value=answer,
                               wait_seconds=time.monotonic()-before)
                return answer
            sequence = ContinuousSequence(robot, observation, profile, report, save, samples,
                factory, target_reference=hover, latest_document=latest,
                confirm=confirm, protection_client=protection)
            try:
                sequence.run(True)
            except (Exception, KeyboardInterrupt):
                sequence.fault()
                raise
            finally:
                sequence.close_hand()
    except (Exception, KeyboardInterrupt) as exc:
        report['exceptions'].append(dict(error=repr(exc), traceback=traceback.format_exc()))
        report['outcome'] = 'aborted_no_further_motion'
        report['next_action'] = 'Inspect the complete saved capture; no automatic retry, return or fault release.'
        print('ABORT:', exc, '接触立即现场硬件急停；无故障自动张开/回臂。', flush=True)
    finally:
        if robot is not None:
            for name in ('Arm_Socket_Close', 'RM_API_UnInit'):
                try:
                    getattr(robot, name)()
                except Exception as exc:
                    report['cleanup_errors'].append(dict(method=name, error=repr(exc)))
        logger.setLevel(prior_level)
        if report['cleanup_errors']:
            report['outcome'] = 'failed_cleanup'
        report['run_elapsed_seconds'] = time.monotonic()-started
        save()
        print('SAVED:', folder, 'OUTCOME:', report['outcome'],
              '耗时{:.1f}s，手通信{}次'.format(report['run_elapsed_seconds'], report['hand_exchange_count']), flush=True)
        try:
            bundle = folder.with_suffix('.zip')
            with zipfile.ZipFile(bundle, 'x', zipfile.ZIP_DEFLATED) as archive:
                for item in sorted(folder.iterdir()):
                    if item.is_file():
                        archive.write(item, folder.name+'/'+item.name)
            print('完整结果已自动打包：', bundle, flush=True)
        except Exception as exc:
            print('结果目录已保存，自动打包失败：', repr(exc), flush=True)
    return 0 if report['outcome'] == 'completed_continuous_hand_open' else 1


if __name__ == '__main__':
    raise SystemExit(main())
