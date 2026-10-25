#!/usr/bin/env python3
"""Supervised empty-hand multi-joint route, settled arm, battery pose, open.

Only the two measured endpoint references are allowed. Joint interpolation and
endpoint checks are not a collision-free swept-volume certificate. An onsite
observer must inspect the entire route and operate hardware stop for contact.
No IK, teaching replay, configuration setters, speed-register writes or return.
"""
import argparse
import copy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import socket
import time
import traceback
import zipfile

import run_arm_hand_sequence as baseline
from arm_protection_transport import ProtectionClient
from capture_installed_hand_geometry import _arm, _json_safe, _vector
from inspect_left_hand import connect
from run_school_battery_hand import ADAPTED_PLAN, TOL, TARGET_TOL


ROOT = Path(__file__).resolve().parents[1]
LATEST_SHA = '86ef8fb26954b262b28177c2686f8ebf0155e9b18e20617738cad038f38c9632'
LATEST_NAME = 'arm_hand_sequence_latest_202513.json'
SOURCE_NAMES = baseline.SOURCE_NAMES + ('run_arm_hand_full_linkage.py', LATEST_NAME,
    'grasp_references_accepted_20261009.json', 'battery_hand_verified_profile.json')
STAGE_TWO = ADAPTED_PLAN[1]['values'][:]


def accepted_latest_reference(document, reference, profile):
    """Accept the pinned read-only post-fault evidence, never an arbitrary pose."""
    if (document.get('schema_version') != 1
            or document.get('outcome') != 'aborted_no_further_motion'
            or document.get('preflight_pass') is not True
            or document.get('prior_arm_move_feedback_pass') is not True
            or document.get('arm_command_attempted') is not False
            or document.get('completed_adapted_stages') != 1
            or document.get('cleanup_errors')
            or document.get('reference_sha256') != baseline.REFERENCE_SHA
            or document.get('configuration') != profile['observed_configuration']):
        raise ValueError('Require the audited202513 scope and frozen device configuration')
    moves = document.get('moves', [])
    if (len(moves) != 1 or moves[0].get('step') != ADAPTED_PLAN[1]
            or moves[0].get('expected') != STAGE_TWO):
        raise ValueError('Latest capture must contain only the accepted stage2 target')
    raw = document.get('fresh_protection_values', {})
    if raw.get('Get_Collision_Stage') != [0, 8] or raw.get('Get_Self_Collision_Enable') != [0, True]:
        raise ValueError('Latest raw controller protection evidence must be8/true')
    candidate = document.get('candidate', {})
    target = _vector(candidate.get('target_joints'), 6, 'latest arm reference')
    expected = baseline.wrist_target(reference)
    if (candidate.get('j6_delta_degrees') != 0 or candidate.get('speed_percent') != 0
            or any(abs(a-b) > .01 for a, b in zip(target, expected))):
        raise ValueError('Latest arm point differs from the accepted post-wrist reference')
    pose = _vector(document.get('arm_hand_phase_pose'), 6, 'latest arm pose')
    rows = document.get('post_fault', [])
    if len(rows) != 3:
        raise ValueError('Complete latest read-only post-fault evidence required')
    for row in rows:
        positions = _vector(row.get('positions'), 6, 'latest hand positions')
        targets = _vector(row.get('targets'), 6, 'latest hand targets')
        if (row.get('status') != [2]*6
                or any(abs(a-b) > TOL for a, b in zip(positions, STAGE_TWO))
                or any(abs(a-b) > TARGET_TOL for a, b in zip(targets, STAGE_TWO))):
            raise ValueError('Latest feedback does not reach the accepted stage2 target')
    derived = copy.deepcopy(reference)
    derived.update(joints=target, pose=pose)
    derived['reserve'] = [min(q-a, b-q) for q, a, b in zip(target,
                            reference['limits']['minimum'], reference['limits']['maximum'])]
    if any(v < 3 for v in derived['reserve']):
        raise ValueError('Latest arm point lacks3 degrees reserve')
    last = document.get('last_arm_state', {})
    baseline.check_resumed_reference(dict(derived,
        joints=_vector(last.get('joints'), 6, 'latest last joints'),
        pose=_vector(last.get('pose'), 6, 'latest last pose')), derived)
    return derived


def accepted_pick_reference(document, profile, start_reference):
    rows = [row for row in document.get('points', []) if row.get('role') == 'pick_hover']
    if len(rows) != 1 or rows[0].get('accepted') is not True:
        raise ValueError('Exactly one accepted measured pick-hover reference required')
    row = rows[0]
    clearance = row.get('clearance_mm')
    if type(clearance) not in (int, float) or not math.isfinite(clearance) or clearance < 100:
        raise ValueError('Recorded hover endpoint clearance must be>=100mm')
    analysis = row.get('analysis', {})
    if analysis.get('diagnostic_pass') is not True or analysis.get('hand_pose_classification') != 'open':
        raise ValueError('Hover endpoint lacks the accepted open-hand diagnostic bracket')
    if row['hand_preflight_after']['targets'] != profile['poses']['open']:
        raise ValueError('Hover hand differs from the frozen open profile')
    brackets = row['arm_query_brackets']
    before, after = _arm(brackets['before']['queries']), _arm(brackets['after']['queries'])
    baseline.check_reference(after, before)
    if (any(abs(a-b) > .2 for a, b in zip(before['joints'], after['joints']))
            or math.dist(before['pose'][:3], after['pose'][:3]) > .0005
            or any(abs(math.atan2(math.sin(a-b), math.cos(a-b))) > .005
                   for a, b in zip(before['pose'][3:], after['pose'][3:]))):
        raise ValueError('Hover endpoint drift exceeded its accepted stationary bracket')
    if (after['frame_signatures'] != start_reference['frame_signatures']
            or after['limits'] != start_reference['limits']
            or any(v < 3 for v in after['reserve'])):
        raise ValueError('Hover endpoint frame/limits/reserve do not match the current reference')
    make_route(start_reference, after)
    return after


def make_route(start, target):
    delta = [b-a for a, b in zip(start['joints'], target['joints'])]
    visible = [i+1 for i, v in enumerate(delta) if abs(v) >= 2]
    distance = math.dist(start['pose'][:3], target['pose'][:3])*1000
    if len(visible) < 3 or not 100 <= distance <= 200:
        raise ValueError('Require the measured visible route: at least3 joints>=2deg and TCP100..200mm')
    return dict(start_joints=start['joints'][:], target_joints=target['joints'][:],
        joint_delta_degrees=delta, joints_changing_at_least2_degrees=visible,
        endpoint_tcp_distance_mm=distance, start_tcp_pose=start['pose'][:],
        target_tcp_pose=target['pose'][:], speed_percent=5, trajectory_connect=0,
        radius=0, block=False, interpolation='controller_joint_Movej',
        endpoint_clearance_not_route_clearance=True, whole_route_geometry_certified=False,
        full_pick_path_ready=False, automatic_return=False)


class FullLinkageSequence(baseline.Sequence):
    def __init__(self, *args, target_reference, latest_document, **kwargs):
        super().__init__(*args, resume_document=latest_document, **kwargs)
        self.target_reference = target_reference
        self.start_hand_target = STAGE_TWO[:]
        self.require_open = False
        self.protection_prepared = True  # Fresh8/true required; never a setter path.
        self.drive_low = self.drive_high = None
        self.report.update(execution_scope='only_empty_hand_two_point_joint_route',
            whole_route_geometry_certified=False, full_pick_path_ready=False,
            automatic_return=False, automatic_contact_release_verified=False,
            route=make_route(self.reference, self.target_reference))

    def safety_reads(self, allow_disabled=False):
        if allow_disabled:
            raise RuntimeError('Full linkage does not authorize protection setters')
        low, high = super().safety_reads(allow_disabled=False)
        self.drive_low, self.drive_high = low[:], high[:]
        for endpoint in (self.reference, self.target_reference):
            if any(min(q-a, b-q) < 3 for q, a, b in zip(endpoint['joints'], low, high)):
                raise RuntimeError('Measured route endpoint lacks3 degrees drive reserve')
        self.report['unknowns'].append('Joint travel bounds do not certify hand/arm/cable swept geometry')
        self.save()
        return low, high

    def arm_state(self):
        joints, pose = super().arm_state()
        if self.drive_low is not None and any(min(q-a, b-q) < 3 for q, a, b in
                                                zip(joints, self.drive_low, self.drive_high)):
            raise RuntimeError('Actual arm feedback lacks3 degrees drive reserve')
        return joints, pose

    def settled(self, target, travel_start=None, timeout=90):
        begin = self.clock()
        stable_since = None
        previous = None
        stable = 0
        while self.clock()-begin < timeout:
            joints, pose = self.arm_state()
            if travel_start is not None and any(not min(a, b)-.2 <= q <= max(a, b)+.2
                    for q, a, b in zip(joints, travel_start, target)):
                raise RuntimeError('Actual joint feedback outside the confirmed two-point travel bounds')
            reached = all(abs(a-b) <= .08 for a, b in zip(joints, target))
            stationary = previous is not None and all(abs(a-b) <= .02
                            for a, b in zip(joints, previous[0]))
            stationary = stationary and math.dist(pose[:3], previous[1][:3]) <= .0002
            stationary = stationary and all(abs(math.atan2(math.sin(a-b), math.cos(a-b))) <= .002
                            for a, b in zip(pose[3:], previous[1][3:]))
            if reached and stationary:
                if stable_since is None:
                    stable_since = self.clock()
                stable += 1
                if self.clock()-stable_since >= 2 and stable >= 5:
                    self.event('full_route_arm_target_observed_stable', target=target,
                        joints=joints, pose=pose, stable_samples=stable,
                        observation_seconds=self.clock()-stable_since, physical_stop_verified=False)
                    return joints, pose
            else:
                stable_since, stable = None, 0
            previous = joints, pose
            self.pause(.1)
        raise RuntimeError('Arm route feedback timeout; hand stays disabled')

    def hand_open_check(self):
        hand = self.hand_factory(self, True)
        try:
            hand.configuration()
            if self.report['configuration'] != self.profile['observed_configuration']:
                raise RuntimeError('Hand configuration differs from frozen device profile')
            state = hand.preflight()
            allowed = (self.profile['poses']['open'],) if self.require_open else (
                self.profile['poses']['open'], STAGE_TWO)
            matches = [pose for pose in allowed if all(abs(a-b) <= TARGET_TOL
                        for a, b in zip(state['targets'], pose)) and all(abs(a-b) <= TOL
                        for a, b in zip(state['positions'], pose))]
            if len(matches) != 1:
                raise RuntimeError('Start hand must be the accepted stage2 pose or the frozen open pose')
            self.start_hand_target = matches[0][:]
            hand.preflight(expected=self.start_hand_target)
            hand.wait(self.start_hand_target, (), self.start_hand_target)
            self.report['initial_hand_classification'] = ('open' if self.start_hand_target ==
                self.profile['poses']['open'] else 'accepted_stage2')
            self.report['guarded_readonly_transport_preflight_pass'] = True
            self.save()
            return state
        finally:
            self.close_hand()

    def open_before_arm(self):
        if self.start_hand_target == self.profile['poses']['open']:
            self.report['initial_open_action'] = 'already_open_reconfirmed_without_write'
            self.require_open = True
            self.save()
            return True
        print('当前是已确认的第2段空手姿态；先正常三段张开，再移动机械臂。')
        if self.confirm('现场确认空手、张开扫掠无遮挡、臂实际停稳且守急停；r=正常张开，其他输入退出：').strip().lower() != 'r':
            self.report['outcome'] = 'operator_exit_before_arm_move'
            return False
        self.preflight()  # Fresh arm/hand/configuration after human input, no writes.
        if self.start_hand_target == self.profile['poses']['open']:
            self.report['initial_open_action'] = 'operator_opened_externally_reconfirmed_without_write'
        else:
            hand = self.hand_factory(self, True)
            try:
                hand.configuration()
                if self.report['configuration'] != self.profile['observed_configuration']:
                    raise RuntimeError('Hand configuration changed before normal opening')
                hand.preflight(expected=STAGE_TWO)
                self.io.enable_motion()
                opened = hand.release_stages(STAGE_TWO[:])
                hand.observe_pose('open_before_arm', opened)
                self.report['open_before_arm'] = hand.preflight(expected=self.profile['poses']['open'])
                self.report['initial_open_action'] = 'explicit_normal_three_stage_open'
            except (Exception, KeyboardInterrupt):
                self.io.disable_motion()
                hand.fault_details()
                raise
            finally:
                self.close_hand()
        self.require_open = True
        self.start_hand_target = self.profile['poses']['open'][:]
        self.save()
        return True

    def run(self, execute):
        if not execute or self.enable_arm_protection:
            raise RuntimeError('Full linkage requires explicit execution; no protection setters')
        self.preflight()
        if not self.open_before_arm():
            return
        route = self.report['route']
        print('多关节整体移动到已测电池悬停位：TCP端点相距约{:.1f}mm，J1/J2/J3变化约{:.1f}/{:.1f}/{:.1f}°，速度5%。'.format(
            route['endpoint_tcp_distance_mm'], *route['joint_delta_degrees'][:3]))
        print('先移臂、现场确认停稳、再执行3段空手抓电池手型及张开；结束不自动回臂。')
        print('关节插补会改变全臂/手/线缆位置；已测端点不证明中途扫掠安全。现场必须确认完整路径。')
        print('硬物接触立即硬件急停，再现场安全释放；输入时不连续监控。')
        answer = self.confirm('现场急停人员就位；输入当前完整路径及手型范围的最小净空毫米，q退出：').strip().lower()
        if answer == 'q':
            self.report['outcome'] = 'operator_exit_before_arm_move'
            return
        try:
            clearance = float(answer)
        except ValueError:
            raise RuntimeError('Need measured full route and hand swept clearance in mm')
        if not math.isfinite(clearance) or clearance < 100:
            raise RuntimeError('Full route and hand swept clearance must be>=100mm')
        self.report['fresh_whole_sweep_clearance_mm'] = clearance
        self.save()
        if self.confirm('现场已确认工作区电池及路径物品已取走，检查整臂关节插补完整路径、手及线缆无遮挡，退出示教且无人同时控制；m=执行一次整体移动，其他输入退出：').strip().lower() != 'm':
            self.report['outcome'] = 'operator_exit_before_arm_move'
            return
        self.report['onsite_full_route_confirmation'] = True
        self.report['operator_reported_battery_removed'] = True
        self.report['operator_reported_path_free'] = True
        self.report['battery_removal_and_route_clearance_independently_detected'] = False
        fresh, _ = self.preflight()
        target = self.target_reference['joints'][:]
        self.report['candidate'] = make_route(fresh, self.target_reference)
        self.arm_command_attempted = True
        self.report['arm_command_attempted'] = True
        self.save()
        status = self.call('Movej_Cmd', target, 5, trajectory_connect=0, r=0, block=False)
        if type(status) is not int or status != 0:
            raise RuntimeError('Movej rejected/uncertain; no retry or automatic return')
        joints, pose = self.settled(target, travel_start=fresh['joints'])
        endpoint = self.full_arm()
        baseline.check_reference(endpoint, self.target_reference)
        self.report.update(arm_move_feedback_pass=True,
            actual_joint_delta_degrees=[b-a for a, b in zip(fresh['joints'], joints)],
            actual_endpoint_tcp_distance_mm=math.dist(fresh['pose'][:3], pose[:3])*1000,
            hover_endpoint_feedback_pass=True)
        self.save()
        if self.confirm('臂目标反馈稳定；现场确认臂实际停止、手型扫掠空且继续守急停；h=执行3段电池手型，其他输入结束：').strip().lower() != 'h':
            self.report['outcome'] = 'operator_exit_after_arm_move'
            return
        self.settled(target)
        self.safety_reads()
        endpoint = self.full_arm()
        baseline.check_resumed_reference(endpoint, self.target_reference)
        self.guard_target, self.guard_pose = target[:], endpoint['pose'][:]
        self.report['arm_hand_phase_pose'] = endpoint['pose'][:]
        self.start_hand_target = self.profile['poses']['open'][:]
        self.save()
        self.hand_phase(target, start_index=0)
        if self.report.get('verified_hand_sequence_completed') is True:
            self.report['outcome'] = 'completed_full_linkage_open'
            self.report['empty_hand_full_linkage_feedback_pass'] = True
        else:
            self.report['empty_hand_full_linkage_feedback_pass'] = False
        self.report.update(full_pick_path_ready=False, whole_route_geometry_certified=False,
                           physical_stop_verified=False, torque_release_verified=False)
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
        parser.error('Requires execute, empty hand, exclusive control and onsite hardware stop; no connection opened')
    directory = Path(__file__).resolve().parent
    latest_path = directory/LATEST_NAME
    references_path = directory/'grasp_references_accepted_20261009.json'
    profile_path = directory/'battery_hand_verified_profile.json'
    if not latest_path.is_file() or hashlib.sha256(latest_path.read_bytes()).hexdigest() != LATEST_SHA:
        parser.error('Latest report differs from audited202513; no connection opened')
    if hashlib.sha256(references_path.read_bytes()).hexdigest() != baseline.REFERENCE_SHA:
        parser.error('Endpoint references differ from audited180315; no connection opened')
    references = json.loads(references_path.read_text(encoding='utf-8'))
    profile_sha = hashlib.sha256(profile_path.read_bytes()).hexdigest()
    if profile_sha != references.get('profile_sha256'):
        parser.error('Frozen hand profile differs from audited capture; no connection opened')
    latest = json.loads(latest_path.read_text(encoding='utf-8'))
    if latest.get('profile_sha256') != profile_sha:
        parser.error('Latest capture profile mismatch; no connection opened')
    profile = json.loads(profile_path.read_text(encoding='utf-8'))
    try:
        safe = baseline.accepted_reference(references, profile)
        reference = accepted_latest_reference(latest, safe, profile)
        hover = accepted_pick_reference(references, profile, reference)
    except (ValueError, RuntimeError, KeyError, TypeError) as exc:
        parser.error('Unaccepted endpoint/source evidence: {}; no connection opened'.format(exc))
    folder = ROOT/'captures'/('arm_hand_full_linkage_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True, exist_ok=False)
    report = dict(schema_version=1, outcome='starting', moves=[], exceptions=[], cleanup_errors=[],
        preflight_pass=False, arm_command_attempted=False, verified_hand_sequence_completed=False,
        empty_hand_full_linkage_feedback_pass=False, physical_stop_verified=False,
        torque_release_verified=False, full_pick_path_ready=False, whole_route_geometry_certified=False,
        execution_scope='only_empty_hand_two_point_joint_route', timestamp_utc=datetime.now(timezone.utc).isoformat(),
        latest_report_sha256=LATEST_SHA, reference_sha256=baseline.REFERENCE_SHA, profile_sha256=profile_sha,
        sdk_wire_capture_available=False, arm_protection_wire_capture_available=True,
        arm_log_scope='motion/state: decoded SDK; protections: separate raw TCP log',
        source_sha256={name: hashlib.sha256((directory/name).read_bytes()).hexdigest() for name in SOURCE_NAMES})
    def save():
        temporary = folder/'result.tmp'
        temporary.write_text(json.dumps(_json_safe(report), ensure_ascii=False, indent=2,
                                        allow_nan=False)+'\n', encoding='utf-8')
        temporary.replace(folder/'result.json')
    save()
    robot = sequence = None
    try:
        import fcntl
        with (folder/'wire.jsonl').open('w', encoding='utf-8') as wire, \
             (folder/'arm_protection_wire.jsonl').open('w', encoding='utf-8') as protection_wire, \
             (folder/'samples.jsonl').open('w', encoding='utf-8') as samples, \
             (ROOT/'captures/hand_stage.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
            robot = connect(args.sdk_dir)
            def protection_log(row):
                protection_wire.write(json.dumps(row)+'\n')
                protection_wire.flush()
            protection = ProtectionClient(protection_log)
            def factory(seq, guarded):
                if seq.io is not None:
                    raise RuntimeError('Hand connection already open')
                def log(row):
                    wire.write(json.dumps(row)+'\n')
                    wire.flush()
                seq.io = baseline.SequenceTransport(socket.create_connection(
                    ('169.254.128.18', 8080), timeout=3), log)
                cls = baseline.ArmGuardHand if guarded else baseline.FeedbackHand
                positional = (seq,) if guarded else ()
                hand = cls(*positional, seq.io, report, save, samples)
                if guarded:
                    seq.io.before_write = hand.guard
                return hand
            def confirm(text):
                answer = input(text)
                sequence.event('operator_input', prompt=text, value=answer)
                return answer
            sequence = FullLinkageSequence(robot, reference, profile, report, save, samples,
                factory, target_reference=hover, latest_document=latest, confirm=confirm,
                protection_client=protection)
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
        report['next_action'] = 'Use the saved full capture; no automatic return, retry or fault release.'
        print('ABORT:', exc, '接触或继续运动立即现场硬件急停；无自动张开/回臂。', flush=True)
    finally:
        if robot is not None:
            for name in ('Arm_Socket_Close', 'RM_API_UnInit'):
                try:
                    getattr(robot, name)()
                except Exception as exc:
                    report['cleanup_errors'].append(dict(method=name, error=repr(exc)))
        if report['cleanup_errors']:
            report['outcome'] = 'failed_cleanup'
        save()
        print('SAVED:', folder, 'OUTCOME:', report['outcome'], flush=True)
        try:
            bundle = folder.with_suffix('.zip')
            with zipfile.ZipFile(bundle, 'x', zipfile.ZIP_DEFLATED) as archive:
                for item in sorted(folder.iterdir()):
                    if item.is_file():
                        archive.write(item, folder.name+'/'+item.name)
            print('完整结果已自动打包：', bundle, flush=True)
        except Exception as exc:
            print('结果目录已保存，自动打包失败：', repr(exc), flush=True)
    return 0 if report['outcome'] in ('completed_full_linkage_open', 'completed_early_open') else 1


if __name__ == '__main__':
    raise SystemExit(main())
