#!/usr/bin/env python3
"""Fixed-arm, supported inert-shell contact pilot; never descends or lifts.

The 200mN thumb/index targets are manufacturer-example trial values, not
calibrated gripping-force limits. Intended shell contact uses force control;
unexpected contact requires onsite hardware stop and safe human release.
Exiting software never stops the controller or removes holding torque.
"""
import argparse
import copy
from datetime import datetime, timezone
import hashlib
import json
import logging
from pathlib import Path
import select
import socket
import sys
import time
import traceback
import zipfile

import run_arm_hand_continuous as accepted
from arm_protection_transport import ProtectionClient
from capture_installed_hand_geometry import _json_safe
from inspect_left_hand import connect
from run_school_battery_hand import ADAPTED_PLAN, TOL
from supported_contact_policy import ContactHand, PREPARE, FORCE_TARGETS, ZERO_FORCE
from supported_contact_transport import SupportedContactTransport, PhaseAuthority
from supported_contact_transport import PREPARE as PREPARE_SIGNATURE

ROOT = Path(__file__).resolve().parents[1]
LATEST_NAME = 'arm_hand_continuous_latest_120502.json'
LATEST_SHA = 'd24e097e8b5f5740f8ab3119f8731a9a0a03736f94abb5fcc8e58ef48a947e77'
PREPARE_RESUME_NAME = 'supported_contact_prepare_142650.json'
PREPARE_RESUME_SHA = '17464501011f1f341943aea7f496ba74c0fc4d5e959e5b252064033e59dac3e2'
SOURCE_NAMES = tuple(dict.fromkeys(accepted.SOURCE_NAMES + (
    'run_supported_battery_contact.py', 'supported_contact_policy.py',
    'supported_contact_transport.py', LATEST_NAME, PREPARE_RESUME_NAME)))
FORCE_PID_BLOCKS = (('p',1225), ('i',1235), ('d',1245), ('g',1255))
FORCE_SOURCE = 'https://oymotion.github.io/ROHand/SDK/ROH_SDK_CXX/'
PROTOCOL_SOURCE = 'https://oymotion.github.io/ROHand/ROH-Protocols/OHandModBusRTUProtocol_Gen2/'


def accepted_continuous_completion(document, profile, hover):
    """Reuse all accepted stage/endpoint checks without inventing an arm move."""
    cycle = document.get('continuous_hand_cycle', {})
    if (document.get('outcome') != 'completed_continuous_hand_open'
            or document.get('arm_preflight_pass') is not True
            or document.get('prior_full_linkage_feedback_pass') is not True
            or document.get('arm_command_attempted') is not False
            or document.get('profile_sha256') != profile['sha256']
            or cycle.get('completed') is not True or cycle.get('completed_stages') != 6
            or cycle.get('hardware_speed_changed') is not False):
        raise ValueError('Require the audited completed120502 continuous hover/open result')
    stage_document = copy.deepcopy(document)
    stage_document.update(outcome='completed_full_linkage_open',
        arm_move_feedback_pass=True, empty_hand_full_linkage_feedback_pass=True)
    # Only the common six-pose/open/protection/hover evidence is reused here.
    # The original document continues to report zero arm commands.
    return accepted.accepted_completion(stage_document, profile, hover)


def supported_input(prompt):
    """Nonblocking canonical-terminal input; monitoring continues until Enter."""
    if select.select([sys.stdin], [], [], 0)[0]:
        value = sys.stdin.readline()
        if not value:
            raise EOFError('Input closed while force mode was active')
        return value.strip().lower()
    return None


def reviewed_prepare_resume(document, profile):
    """Known interrupted empty prepare: ACK evidence is not settling evidence."""
    moves = document.get('moves', [])
    if (document.get('outcome') != 'aborted_no_further_writes'
            or document.get('arm_command_attempted') is not False
            or document.get('configuration') != profile['observed_configuration']
            or document.get('latest_report_sha256') != LATEST_SHA
            or document.get('force_zero_reset') is not None
            or document.get('contact_sensor_baseline') is not None
            or document.get('last_contact_snapshot') is not None
            or len(moves) != 1 or moves[0].get('step') != ADAPTED_PLAN[0]
            or moves[0].get('outcome') != 'fault_no_further_writes'
            or len(document.get('exceptions', [])) != 1 or document.get('cleanup_errors')
            or moves[0].get('error') != "ProtectionFault('Stall status5 blocks supported-contact prepare/release')"):
        raise ValueError('Only audited142650 interrupted empty prepare is resumable')
    row = document['last_snapshot']
    if (row.get('status') != [2,2,2,2,0,5]
            or any(abs(a-b) > TOL for a,b in zip(row.get('positions', []), PREPARE))
            or len(row.get('positions', [])) != 6
            or len(row.get('targets', [])) != 6
            or any(abs(a-b) > 2 for a,b in zip(row['targets'], PREPARE))
            or moves[0]['before'].get('force_targets') != list(ZERO_FORCE)):
        raise ValueError('Interrupted prepare evidence differs from audited142650')
    return True


class SupportedSequence(accepted.ContinuousSequence):
    def __init__(self, *args, authority=None, poll_input=supported_input,
                 prepare_resume_document=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.authority = authority if authority is not None else PhaseAuthority()
        self.poll_input = poll_input
        self.force_pid = None
        self.force_written = False
        self.contact_observed = False
        self.prepare_resume_document = prepare_resume_document
        if prepare_resume_document is not None:
            reviewed_prepare_resume(prepare_resume_document, self.profile)
            self.force_pid = copy.deepcopy(prepare_resume_document['force_pid_on_device'])
        self.report.update(execution_scope='fixed_hover_supported_inert_shell_thumb_index_contact',
            arm_motion_allowed=False, automatic_lift=False, automatic_descent=False,
            automatic_fault_open=False, force_targets_mn=list(FORCE_TARGETS),
            force_target_source=FORCE_SOURCE, protocol_source=PROTOCOL_SOURCE,
            force_target_is_calibrated_safe_limit=False, intended_contact='supported_inert_shell_only',
            real_battery_hold_verified=False, physical_stop_verified=False,
            torque_release_verified=False, full_pick_path_ready=False,
            repeated_acceptance_test_required=False, supported_contact_live_verified=False)
        self.report['prepare_resume_requested'] = prepare_resume_document is not None
        self.report['prepare_resume_sha256'] = PREPARE_RESUME_SHA if prepare_resume_document is not None else None

    def call(self, name, *args, **kwargs):
        if not name.startswith('Get_'):
            self.fault()
            raise RuntimeError('Supported contact entry permits arm GET calls only')
        return super().call(name, *args, **kwargs)

    def close_hand(self):
        if self.io is not None:
            current, self.io = self.io, None
            self.hand_motion_enabled = False
            current.disarm_motion()  # Normal close is not an application fault.
            try:
                current.sock.close()
            except Exception as exc:
                current.disable_motion()
                self.report.setdefault('cleanup_errors', []).append(
                    dict(method='hand.close', error=repr(exc)))
                self.save()
                raise

    def fault(self):
        # No arm motion was issued. In particular no inherited Move_Stop path.
        self.hand_motion_enabled = False
        self.authority._terminal_latched = True
        if self.io is not None:
            self.io.disable_motion()
        self.report.update(fault_latched=True, automatic_fault_open=False,
            physical_stop_verified=False, torque_release_verified=False,
            next_action='Onsite hardware stop for unexpected contact/continuing motion; '
                        'safely release with the support still carrying the test piece. '
                        'No automatic retry, force-zero, position return or arm move.')
        self.save()

    def checked_hand(self):
        hand = self.hand_factory(self, True)
        hand.configuration()
        values = {}
        for name, address in FORCE_PID_BLOCKS:
            value = hand.read('force_pid_'+name, address, 5)
            hand._u16(value, 5, 'force PID '+name)
            values[name] = value
        # Values are read from this unit; neither factory values nor tuning are
        # invented. Nonzero active P is a necessary condition, not a stability proof.
        if any(values['p'][ch] == 0 for ch in (0, 1)):
            raise RuntimeError('Active force P is zero; force control not ready; no PID setter')
        if self.force_pid is not None and self.force_pid != values:
            raise RuntimeError('Force PID changed across operator gates; no continuation')
        self.force_pid = copy.deepcopy(values)
        self.report['force_pid_on_device'] = copy.deepcopy(values)
        self.report['force_pid_raw_unit'] = 'coefficient_times_100_u16'
        self.report['force_pid_decoded'] = {name:[value/100.0 for value in rows]
                                          for name,rows in values.items()}
        self.report.setdefault('force_pid_reads', []).append(dict(
            monotonic=self.clock(), values=copy.deepcopy(values), source='on_device_readonly',
            stable_force_control_verified=False))
        self.save()
        return hand

    def critical_contact_check(self, hand, expected_force):
        limits = hand.read('current_limits_before_contact', 1095, 6)
        speeds = hand.read('speeds_before_contact', 1125, 6)
        hand._u16(limits, 6, 'current limits')
        hand._u16(speeds, 6, 'speed')
        if limits != [1299]*6 or speeds != [65535]*6:
            raise RuntimeError('Accepted current protection/speed changed; no write')
        row = hand.snapshot_contact(expected_force)
        self.report['contact_preflight'] = copy.deepcopy(row)
        self.save()
        return row

    def permit(self, phase):
        self.io.set_phase(phase, self.authority)
        self.io.enable_motion()
        self.hand_motion_enabled = True

    def monitor_release_input(self, hand, deadline_seconds=60):
        prompt = ('现场确认两指实际接触、壳体无变形、支架全程托住。60秒内输入r再回车='
                  '退出力控，核对回预抓8000后顺序张开；q=保留当前力控退出。'
                  '非预期接触/继续运动立即硬件急停：')
        print(prompt, flush=True)
        began = self.clock()
        self.event('supported_operator_wait_before', prompt=prompt)
        # Preserve complete feedback while the terminal waits for Enter. A read
        # group can take several seconds; this is not a realtime contact stop.
        while self.clock()-began < deadline_seconds:
            row = hand.snapshot_contact(FORCE_TARGETS)
            answer = self.poll_input(prompt)
            if answer is not None:
                self.event('operator_input', prompt=prompt, value=answer,
                           wait_seconds=self.clock()-began, monitored=True)
                return answer, row
        raise RuntimeError('Operator release deadline elapsed; hardware stop/safe release required')

    def normal_release(self, hand, previous_positions):
        # Only called following an explicit normal-release input, not in finally.
        self.permit('release')
        self.report['normal_release_authorized'] = True
        self.save()
        self.io.write_group(1115, list(ZERO_FORCE))
        hand.wait_release_to_prepare(previous_positions)
        hand.release_stages(list(PREPARE))
        final = hand.preflight(expected=self.profile['poses']['open'])
        self.report['final_hand'] = final
        self.io.set_phase('readonly', self.authority)
        self.hand_motion_enabled = False
        self.report['hand_open_feedback_pass'] = True
        self.save()
        return final

    def run(self, execute):
        if not execute or self.enable_arm_protection:
            raise RuntimeError('Explicit supported test-piece execution required; no arm setters')
        print('断电测试件/空壳由支架全程承载，机械臂保持已验收hover；本脚本不下降、不抬升。', flush=True)
        print('先空手预抓；再把固定支撑的壳体放进拇指与食指的空隙。只试这两指各200mN力目标。', flush=True)
        print('200mN是厂家示例试验值，非认证安全夹力。非预期硬物接触立即现场急停、安全释放。', flush=True)
        print('Ctrl+C/q/异常只禁止后续命令，不停止电机、不自动松开。', flush=True)
        action = '新鲜核对当前预抓并继续空载力零点' if self.prepare_resume_document is not None else '预抓及空载力零点'
        if self.prepare_resume_document is not None:
            print('续接142650空手预抓：不重复写预抓，不自动张开或移动机械臂。', flush=True)
        answer = self.confirm('电池及支架在手的扫掠范围外，空手、独占控制、现场守急停；p='+action+'，其他输入退出：').strip().lower()
        if answer != 'p':
            self.report['outcome'] = 'operator_exit_no_writes'
            return
        self.report['onsite_empty_preparation_confirmed'] = True
        self.fresh_arm(expected_role='pick_hover')
        hand = self.checked_hand()
        try:
            hand.authorize_empty_preparation(self.authority)
            if self.prepare_resume_document is not None:
                # Fresh mode/identity/configuration and >=3 actual groups are
                # required before importing that one historical ACK. No resend.
                hand.preflight(expected=list(PREPARE))
                row = hand.wait(list(PREPARE), (), list(PREPARE))
                self.authority._attempted_writes.add(PREPARE_SIGNATURE)
                self.authority._acknowledged_writes.add(PREPARE_SIGNATURE)
                self.report['resumed_empty_prepare'] = dict(
                    live_actual_feedback_observed=True, final=copy.deepcopy(row),
                    original_write_ack_audited=True, source_sha256=PREPARE_RESUME_SHA,
                    prepare_command_resent=False, physical_stop_verified=False)
                print('当前预抓姿态已用新鲜反馈确认；跳过重复预抓写入，继续空载清零。', flush=True)
            else:
                hand.preflight(expected=self.profile['poses']['open'])
                self.permit('prepare_empty')
                hand.write_pose(ADAPTED_PLAN[0], self.profile['poses']['open'][:])
            self.report['preflight_pass'] = True
            self.permit('reset_empty')
            self.io.write_group(1016, [1])
            self.report['force_zero_reset'] = dict(acknowledged=True, empty_hand_confirmed=True,
                sensor_calibration_verified=False, raw_ack_is_not_physical_stop=True)
            hand.sensor_baseline(samples=3)
            self.report['empty_prepare_feedback_pass'] = True
            self.save()
        finally:
            hand.end_empty_preparation()
            self.close_hand()
        print('预抓已到位。保持机械臂不动，用稳定支架把空壳抬到当前手的位置。', flush=True)
        print('壳体置于拇指/食指指面之间；支架和另外三指无遮挡。先把人手退出活动范围，再按c。', flush=True)
        answer = self.confirm('壳体固定支撑、两指空隙可接触且人手已退出、现场守急停；c=双指接触试验，r=取消试验并正常张开，q=保留预抓退出：').strip().lower()
        if answer not in ('c', 'r'):
            self.report.update(outcome='operator_exit_prepared_no_contact',
                next_action='Hand stays prepared; no contact/force command or automatic return issued.')
            return
        self.fresh_arm(expected_role='pick_hover')
        hand = self.checked_hand()
        try:
            row = self.critical_contact_check(hand, ZERO_FORCE)
            if (any(abs(a-b) > TOL for a,b in zip(row['positions'], PREPARE))
                    or any(v >= 200 for v in row['forces'])):
                raise RuntimeError('Prepare pose changed or shell already preloads force sensors; no contact write')
            if answer == 'r':
                self.normal_release(hand, row['positions'])
                self.report['outcome'] = 'completed_empty_prepare_open_no_contact'
                return
            self.report.update(supported_inert_shell_confirmed=True, support_carries_full_weight=True,
                support_independently_detected=False, operator_hands_withdrawn_confirmed=True)
            self.permit('contact')
            self.io.write_group(1115, list(FORCE_TARGETS))
            self.force_written = True
            hand.wait_contact()
            self.contact_observed = True
            print('两通道力控状态和反馈已观察到；支架继续承载，不取走支架、不抬升。', flush=True)
            answer, last = self.monitor_release_input(hand)
            if answer != 'r':
                self.report.update(outcome='operator_exit_force_mode_retained',
                    next_action='Force control remains active; software exit is not stop/release. '
                                'Onsite hardware stop and safe supported release if needed.')
                return
            self.report['onsite_contact_without_visible_damage_confirmed'] = True
            # Fresh protection/identity/limits at the release boundary. This is
            # still a loaded phase, so position-mode preflight is not reused.
            self.fresh_arm(expected_role='pick_hover')
            hand.configuration()
            for name,address in FORCE_PID_BLOCKS:
                if hand.read('force_pid_before_release_'+name,address,5) != self.force_pid[name]:
                    raise RuntimeError('Force PID changed before normal release')
            last = self.critical_contact_check(hand, FORCE_TARGETS)
            self.normal_release(hand, last['positions'])
            self.report.update(outcome='completed_supported_contact_release',
                supported_contact_live_verified=True, supported_contact_feedback_observed=True,
                real_battery_hold_verified=False, physical_stop_verified=False,
                torque_release_verified=False, automatic_lift=False, full_pick_path_ready=False)
            self.save()
        finally:
            self.close_hand()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute-supported-test-piece', action='store_true')
    parser.add_argument('--confirm-inert-supported', action='store_true')
    parser.add_argument('--confirm-exclusive-control', action='store_true')
    parser.add_argument('--confirm-onsite-stop', action='store_true')
    parser.add_argument('--resume-report', type=Path,
        help='Only the audited142650 interrupted empty-prepare result; never resumes contact')
    parser.add_argument('--sdk-dir', type=Path,
        default=Path('/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package'))
    args = parser.parse_args(argv)
    if not all((args.execute_supported_test_piece, args.confirm_inert_supported,
                args.confirm_exclusive_control, args.confirm_onsite_stop)):
        parser.error('Requires supported inert test piece, exclusive control and onsite stop; no connection opened')
    parser.error('154756 live capture exceeded the thumb position cap with near-zero force feedback. '
                 'This force-contact CLI is withdrawn; no connection opened. '
                 'Use run_supported_contact_release.py only with independent support for explicit opening.')
    directory = Path(__file__).resolve().parent
    def pinned(name, digest):
        path = directory/name
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            parser.error('Audited source mismatch: {}; no connection opened'.format(name))
        return json.loads(path.read_text(encoding='utf-8'))
    latest = pinned(LATEST_NAME, LATEST_SHA)
    completed = pinned(accepted.LATEST_NAME, accepted.LATEST_SHA)
    previous = pinned(accepted.measured.LATEST_NAME, accepted.measured.LATEST_SHA)
    references = pinned('grasp_references_accepted_20261009.json', accepted.baseline.REFERENCE_SHA)
    profile_sha = references['profile_sha256']
    profile = pinned('battery_hand_verified_profile.json', profile_sha)
    profile['sha256'] = profile_sha
    resume = None
    if args.resume_report is not None:
        path = args.resume_report
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != PREPARE_RESUME_SHA:
            parser.error('Resume report is not the audited142650 result; no connection opened')
        resume = json.loads(path.read_text(encoding='utf-8'))
    try:
        safe = accepted.baseline.accepted_reference(references, profile)
        observation = accepted.measured.accepted_latest_reference(previous, safe, profile)
        hover = accepted.measured.accepted_pick_reference(references, profile, observation)
        accepted.accepted_completion(completed, profile, hover)
        accepted_continuous_completion(latest, profile, hover)
        if resume is not None: reviewed_prepare_resume(resume, profile)
    except (ValueError, RuntimeError, KeyError, TypeError, IndexError) as exc:
        parser.error('Unaccepted successful source evidence: {}; no connection opened'.format(exc))
    folder = ROOT/'captures'/('supported_battery_contact_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    report = dict(schema_version=1, outcome='starting', moves=[], exceptions=[], cleanup_errors=[],
        preflight_pass=False, arm_command_attempted=False, hand_open_feedback_pass=False,
        physical_stop_verified=False, torque_release_verified=False, full_pick_path_ready=False,
        real_battery_hold_verified=False, supported_contact_live_verified=False,
        timestamp_utc=datetime.now(timezone.utc).isoformat(), latest_report_sha256=LATEST_SHA,
        reference_sha256=accepted.baseline.REFERENCE_SHA, profile_sha256=profile_sha,
        sdk_wire_capture_available=False, arm_protection_wire_capture_available=True,
        hand_exchange_count=0, hand_exchange_elapsed_seconds=0.,
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
            authority = PhaseAuthority()
            def factory(seq, guarded):
                if not guarded or seq.io is not None:
                    raise RuntimeError('A single guarded hand connection is required')
                def log(row):
                    wire.write(json.dumps(row)+'\n'); wire.flush()
                    if row.get('event') == 'tx': report['hand_exchange_count'] += 1
                    if row.get('event') == 'response':
                        report['hand_exchange_elapsed_seconds'] += row.get('elapsed_s', 0.)
                seq.io = SupportedContactTransport(socket.create_connection(
                    ('169.254.128.18',8080),timeout=3), log, authority)
                return ContactHand(seq, seq.io, report, save, samples)
            def confirm(prompt):
                before = time.monotonic()
                answer = input(prompt)
                sequence.event('operator_input', prompt=prompt, value=answer,
                    wait_seconds=time.monotonic()-before, monitored=False)
                return answer
            sequence = SupportedSequence(robot, observation, profile, report, save, samples,
                factory, target_reference=hover, latest_document=latest, authority=authority,
                confirm=confirm, protection_client=protection, prepare_resume_document=resume)
            try:
                sequence.run(True)
            except (Exception, KeyboardInterrupt):
                sequence.fault()
                raise
            finally:
                sequence.close_hand()
    except (Exception, KeyboardInterrupt) as exc:
        report['exceptions'].append(dict(error=repr(exc), traceback=traceback.format_exc()))
        report['outcome'] = 'aborted_no_further_writes'
        report['next_action'] = ('Unexpected contact/continuing motion: onsite hardware stop and '
            'safe supported release. No automatic retry, force-zero, open or arm return.')
        print('ABORT:', exc, '现场硬件急停并安全释放；退出不停止电机，不自动清力或张开。', flush=True)
    finally:
        if robot is not None:
            for name in ('Arm_Socket_Close', 'RM_API_UnInit'):
                try: getattr(robot, name)()
                except Exception as exc:
                    report['cleanup_errors'].append(dict(method=name, error=repr(exc)))
        logger.setLevel(prior_level)
        if report['cleanup_errors']: report['outcome'] = 'failed_cleanup'
        report['run_elapsed_seconds'] = time.monotonic()-started
        save()
        print('SAVED:', folder, 'OUTCOME:', report['outcome'], flush=True)
        try:
            bundle = folder.with_suffix('.zip')
            with zipfile.ZipFile(bundle, 'x', zipfile.ZIP_DEFLATED) as archive:
                for item in sorted(folder.iterdir()):
                    if item.is_file(): archive.write(item, folder.name+'/'+item.name)
            print('完整结果已自动打包：', bundle, flush=True)
        except Exception as exc:
            print('目录已保存，自动打包失败：', repr(exc), flush=True)
    return 0 if report['outcome'] == 'completed_supported_contact_release' else 1


if __name__ == '__main__':
    raise SystemExit(main())
