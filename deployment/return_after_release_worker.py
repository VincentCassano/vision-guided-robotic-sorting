#!/usr/bin/env python3
"""Isolated SDK: one empty World+Z return; hand and configuration are read only."""
import argparse
import copy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/battery_grasp/src'))
from battery_grasp.taught_dh_model import EXPECTED_DH
from battery_grasp.real_grasp import orientation_angle_rad
from battery_grasp.pregrasp_replay import check_segment
from high_descent_prepare_worker import same_pose
from run_arm_hand_sequence import Sequence
from vision_cycle_sequence import VisualCycle
from vision_cycle_hand import CycleHand, PREPARE
from vision_cycle_transport import CycleTransport
from school_battery_transport import MULTI, SINGLE
from arm_protection_transport import unique_object

SUCCESS = 'completed_empty_return_after_supported_release'
WAIT_PHASE = 'await_empty_return_after_release'
SAFETY_CONFIGURATION = ('Get_Joint_EN_State', 'Get_Joint_Err_Flag',
    'Get_Collision_Stage', 'Get_Self_Collision_Enable', 'Get_Joint_Drive_Min_Pos',
    'Get_Joint_Drive_Max_Pos', 'Get_Arm_Software_Version', 'Get_Install_Pose')


def strict_json(raw):
    def bad_constant(value):
        raise ValueError('Nonfinite source JSON constant: ' + value)
    return json.loads(raw, object_pairs_hook=unique_object, parse_constant=bad_constant)


def orientation_radians(a, b):
    return orientation_angle_rad(a, b)


def arm_shape(arm):
    for key in ('joints', 'pose', 'reserve'):
        value = arm[key]
        if (not isinstance(value, list) or len(value) != 6
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in value)):
            raise ValueError('Incomplete/nonfinite source arm ' + key)
    for key in ('minimum', 'maximum', 'speed', 'acceleration'):
        value = arm['limits'][key]
        if (not isinstance(value, list) or len(value) != 6
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in value)):
            raise ValueError('Incomplete source arm limits: ' + key)
    if arm['arm_error'] != 0 or arm['system_error'] != 0 or min(arm['reserve']) < 3:
        raise ValueError('Source arm error or joint reserve below3degrees')
    if any(a >= b for a, b in zip(arm['limits']['minimum'], arm['limits']['maximum'])):
        raise ValueError('Source arm limit ordering invalid')
    if set(arm['frame_signatures']) != {'work', 'tool'}:
        raise ValueError('Source frame signatures incomplete')


def prepare_readback(row):
    for key in ('targets', 'positions', 'status', 'current'):
        value = row[key]
        if (not isinstance(value, list) or len(value) != 6
                or any(type(v) is not int or not 0 <= v <= 65535 for v in value)):
            raise ValueError('Complete hand source feedback required: ' + key)
    if (any(abs(a - b) > 2 for a, b in zip(row['targets'], PREPARE))
            or any(abs(a - b) > 150 for a, b in zip(row['positions'], PREPARE))
            or any(v not in (0, 1, 2, 5) for v in row['status'])):
        raise ValueError('Source release not observed at unprotected PREPARE feedback')


def model_signature(model):
    if (model.get('dh_valid') is not True or model.get('identity_valid') is not True
            or model.get('errors') or model.get('hardware_writes') != 0
            or model.get('controller_configuration_writes') != 0):
        raise ValueError('Read-only actual source model identity/DH not valid')
    if [model['dh_response']['joint_' + str(i)] for i in range(1, 7)] != EXPECTED_DH:
        raise ValueError('Actual source DH differs from reviewed model')
    return dict(identity=model['identity_response'], dh=model['dh_response'])


def metadata_signature(metadata):
    if (metadata.get('metadata_complete') is not True or metadata.get('errors')
            or metadata.get('api_version') != '4.3.1'
            or metadata.get('host_system') != 'Linux' or metadata.get('host_machine') != 'aarch64'
            or metadata.get('controller_run_mode_raw', {}).get('data') != [1]):
        raise ValueError('Complete actual SDK/real-mode identity required')
    return {key: metadata[key] for key in ('api_version', 'host_system', 'host_machine',
        'python_pointer_bits', 'loaded_native', 'controller_software_info', 'controller_run_mode_raw')}


def source_context(folder, profile, resume_requested=False):
    parent = strict_json((folder / 'parent_result.json').read_bytes())
    documents = {}
    for role in ('grasp', 'release'):
        raw = (folder / ('sources/' + role + '_worker_result.json')).read_bytes()
        source = parent['sources'][role]
        if hashlib.sha256(raw).hexdigest() != source['sha256'] or len(raw) != source['byte_count']:
            raise ValueError('Captured source bytes changed: ' + role)
        documents[role] = strict_json(raw)
    grasp, release = documents['grasp'], documents['release']
    if (type(grasp.get('schema_version')) is not int or grasp['schema_version'] != 1
            or grasp.get('scope') != 'fresh_taught_shell_descent_supported_grasp_release_empty_return'
            or grasp.get('outcome') != 'aborted_no_further_writes' or grasp.get('cleanup_errors') != []
            or type(grasp.get('arm_actuator_writes')) is not int or grasp['arm_actuator_writes'] != 1
            or type(grasp.get('hand_register_writes')) is not int or grasp['hand_register_writes'] != 2):
        raise ValueError('Exactly one completed descent before the recorded contact-protection fault required')
    errors = grasp.get('exceptions', [])
    if len(errors) != 1:
        raise ValueError('Original grasp fault evidence incomplete')
    error = errors[0].get('error', '')
    prefix = "ProtectionFault('Current protection or unexpected force-control status: ["
    if not error.startswith(prefix) or not error.endswith("]')"):
        raise ValueError('Original fault is not the accepted position contact protection')
    status = strict_json('[' + error[len(prefix):-3] + ']')
    if (len(status) != 6 or any(type(v) is not int or v not in range(6) for v in status)
            or 3 not in status or 4 in status):
        raise ValueError('Original protection fault differs from the accepted status3 case')
    segments = grasp.get('arm_segments', [])
    if len(segments) != 1 or segments[0].get('phase') != 'descending':
        raise ValueError('Unique completed descending segment required; no old trajectory alignment')
    if (type(release.get('schema_version')) is not int or release['schema_version'] != 1
            or release.get('scope') != 'supported_position_release_only'
            or release.get('outcome') != 'completed_supported_four_channel_opening_feedback_recorded'
            or release.get('cleanup_errors') != [] or release.get('exceptions') != []
            or release.get('opening_retreat_confirmed') is not True
            or release.get('allow_protected_opening') is not True):
        raise ValueError('Successful supported four-channel opening evidence required')
    for key, expected in (('arm_actuator_writes', 0), ('arm_stop_writes', 0),
                          ('controller_configuration_writes', 0), ('hand_register_writes', 1)):
        if type(release.get(key)) is not int or release[key] != expected:
            raise ValueError('Unexpected source release write count: ' + key)
    endpoint, final_arm, upper = segments[0]['endpoint'], release['final_arm'], grasp['upper']
    for arm in (endpoint, final_arm, upper):
        arm_shape(arm)
    if (not same_pose(final_arm, endpoint) or final_arm['limits'] != endpoint['limits']
            or upper['limits'] != endpoint['limits']
            or upper['frame_signatures'] != endpoint['frame_signatures']):
        raise ValueError('Released arm differs from the completed descent or original upper frames/limits')
    prepare_readback(release['final_hand'])
    if release['final_hand']['identity'] != profile['device_identity']:
        raise ValueError('Release source hand identity differs')
    for source in (grasp, release):
        if source['configuration'] != profile['observed_configuration']:
            raise ValueError('Source full hand configuration differs from measured profile')
    if (grasp['source_audit'] != release['source_audit']
            or metadata_signature(grasp['sdk_environment']) != metadata_signature(release['sdk_environment'])
            or model_signature(grasp['fresh_model']) != model_signature(release['fresh_model'])):
        raise ValueError('Source SDK/model identity changed between descent and supported opening')
    for name in SAFETY_CONFIGURATION:
        if grasp['arm_safety_reads'][name] != release['arm_safety_reads'][name]:
            raise ValueError('Source arm protection/installation/drive configuration differs: ' + name)
    height = (upper['pose'][2] - final_arm['pose'][2]) * 1000
    if (not 30 <= height <= 120 or math.dist(upper['pose'][:2], final_arm['pose'][:2]) * 1000 > .5
            or orientation_radians(upper['pose'][3:], final_arm['pose'][3:]) > .005):
        raise ValueError('Source upper is not the accepted same-XY/wrist30..120mm World+Z corridor')
    context = dict(grasp=grasp, release=release, endpoint=endpoint, final_arm=final_arm,
        upper=upper, expected_tip=grasp['live_tip_parameters'], source_records=parent['sources'])
    if resume_requested:
        raw = (folder / 'sources/resume_return_worker_result.json').read_bytes()
        source = parent['sources']['resume_return']
        if hashlib.sha256(raw).hexdigest() != source['sha256'] or len(raw) != source['byte_count']:
            raise ValueError('Captured resume source bytes changed')
        prior = strict_json(raw)
        if (type(prior.get('schema_version')) is not int or prior['schema_version'] != 1
                or prior.get('scope') != 'explicit_empty_return_after_supported_release'
                or prior.get('outcome') != 'aborted_no_further_writes'
                or prior.get('cleanup_errors') != [] or prior.get('source_context_accepted') is not True
                or prior.get('arm_segments', []) != []):
            raise ValueError('Resume only accepts an incomplete original empty return with normal cleanup')
        for key, expected in (('arm_actuator_writes', 1), ('arm_stop_writes', 1),
                              ('hand_register_writes', 0), ('controller_configuration_writes', 0)):
            if type(prior.get(key)) is not int or prior[key] != expected:
                raise ValueError('Resume source write count differs: ' + key)
        if (len(prior.get('exceptions', [])) != 1
                or prior['exceptions'][0].get('error') != "ValueError('Wrist changed beyond0.005rad')"
                or type(prior.get('arm_stop_attempt', {}).get('result')) is not int
                or prior['arm_stop_attempt']['result'] != 0):
            raise ValueError('Resume only accepts the recorded Euler guard fault and one stop ACK0')
        for role in ('grasp', 'release'):
            if (prior['source_records'][role]['sha256'] != parent['sources'][role]['sha256']
                    or prior['source_records'][role]['byte_count'] != parent['sources'][role]['byte_count']):
                raise ValueError('Resume source refers to different grasp/release bytes')
        if (prior['original_upper'] != upper or prior['source_audit'] != grasp['source_audit']
                or prior['configuration'] != profile['observed_configuration']
                or prior['live_tip_parameters'] != context['expected_tip']
                or metadata_signature(prior['sdk_environment']) != metadata_signature(grasp['sdk_environment'])
                or model_signature(prior['fresh_model']) != model_signature(grasp['fresh_model'])
                or model_signature(prior['fresh_model_after_permission']) != model_signature(grasp['fresh_model'])):
            raise ValueError('Resume source target/model/SDK/profile/tip differs')
        for name in SAFETY_CONFIGURATION:
            if prior['arm_safety_reads'][name] != release['arm_safety_reads'][name]:
                raise ValueError('Resume source protection/installation/drive configuration differs: ' + name)
        plan = prior['plan']
        start = plan['start']
        arm_shape(start)
        if (not same_pose(start, final_arm) or start['limits'] != final_arm['limits']
                or not same_pose(prior['initial_arm'], final_arm)
                or prior['initial_arm']['limits'] != final_arm['limits']):
            raise ValueError('Resume original start differs from recorded supported opening')
        target = plan['target_sdk_pose_m_rad']
        if (not isinstance(target, list) or len(target) != 6
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in target)
                or target[:2] != start['pose'][:2] or target[3:] != start['pose'][3:]
                or abs(target[2] - upper['pose'][2]) > 1e-9
                or plan['target_world_mm'] != [v * 1000 for v in target[:3]]):
            raise ValueError('Resume original target is not the same-XY/wrist original upper-Z target')
        distance = (target[2] - start['pose'][2]) * 1000
        if (not 30 <= distance <= 120 or abs(plan['distance_mm'] - distance) > 1e-8
                or type(plan['speed_percent']) is not int or not 1 <= plan['speed_percent'] <= 5
                or type(plan['nominal_speed_mm_s']) not in (int, float)
                or not math.isfinite(plan['nominal_speed_mm_s']) or not 0 < plan['nominal_speed_mm_s'] <= 5
                or abs(plan['nominal_speed_mm_s'] - context['expected_tip']['line_speed_m_s']
                    * 1000 * plan['speed_percent'] / 100) > 1e-9
                or type(plan['arm_timeout_seconds']) not in (int, float)
                or not math.isfinite(plan['arm_timeout_seconds']) or not 0 < plan['arm_timeout_seconds'] <= 60):
            raise ValueError('Resume original distance/speed/deadline is outside bounded return')
        expected_drive = dict(minimum=release['arm_safety_reads']['Get_Joint_Drive_Min_Pos'][1],
            maximum=release['arm_safety_reads']['Get_Joint_Drive_Max_Pos'][1])
        if plan['drive_limits'] != expected_drive:
            raise ValueError('Resume original drive limits differ')
        for key in ('minimum', 'maximum'):
            values = plan['live_joint_guard_limits'][key]
            if (not isinstance(values, list) or len(values) != 6
                    or any(type(v) not in (int, float) or not math.isfinite(v) for v in values)):
                raise ValueError('Resume original branch guard is incomplete')
        if (any(a >= b for a, b in zip(plan['live_joint_guard_limits']['minimum'],
                    plan['live_joint_guard_limits']['maximum'])) or not plan.get('local_ik_rows')):
            raise ValueError('Resume original bounded branch witnesses missing')
        check_segment(start, plan)
        context.update(resume=prior, original_return_plan=plan,
            resume_stop_ACK_not_physical_stop=True, last_moving_readback_used_as_current=False)
    elif 'resume_return' in parent['sources']:
        raise ValueError('Resume bytes require explicit resume-return selection')
    return context


class ReadOnlyCycleTransport(CycleTransport):
    def enable_motion(self):
        raise RuntimeError('Empty return never enables hand motion')

    def write_group(self, *args, **kwargs):
        raise RuntimeError('Empty return has no hand write capability')

    def _validate(self, message):
        if not isinstance(message, dict) or message.get('command') not in (MULTI, SINGLE):
            raise RuntimeError('Empty return hand transport allows reviewed reads only')
        return super()._validate(message)


class EmptyReturn(VisualCycle):
    def __init__(self, *args, context, **kwargs):
        super().__init__(*args, taught_only=True, **kwargs)
        self.context = context

    def arm_state(self):
        joints, pose = super().arm_state()
        if self.context.get('original_return_plan'):
            plan = self.context['original_return_plan']
            current = dict(plan['start'], joints=joints, pose=pose)
            self.report['original_return_guard_current_readback'] = dict(
                joints=joints, pose=pose, source='this_session_current_arm_state',
                phase=self.report['phase'], physical_stop_verified=False)
            self.save()
            check_segment(current, plan)
        return joints, pose

    def full_arm(self):
        current = super().full_arm()
        plan = self.context.get('original_return_plan')
        if plan is not None:
            # Enforce the historical physical corridor as well as the new
            # current-start plan, including calculation and endpoint reads.
            # Saving precedes interpretation so an out-of-corridor raw group
            # remains available in the same capture.
            self.report['original_return_guard_full_readback'] = dict(
                current=current, source='this_session_full_arm_readback',
                phase=self.report['phase'], physical_stop_verified=False)
            self.save()
            check_segment(current, plan)
        return current

    def preflight(self, expected_arm, expected_hand, keep_hand=False):
        if keep_hand:
            raise RuntimeError('Empty return closes hand reads before every movement boundary')
        arm, tip = super().preflight(expected_arm, PREPARE)
        arm_shape(arm)
        if expected_arm is not None and arm['limits'] != expected_arm['limits']:
            raise RuntimeError('Fresh arm limits/dynamics changed; no motion')
        if tip != self.context['expected_tip']:
            raise RuntimeError('Fresh tip parameters differ from accepted descent; no motion')
        if metadata_signature(self.report['sdk_environment']) != metadata_signature(self.context['grasp']['sdk_environment']):
            raise RuntimeError('Fresh SDK/controller identity differs from source descent')
        if self.report['configuration'] != self.context['release']['configuration']:
            raise RuntimeError('Fresh full hand configuration differs from source release')
        for name in SAFETY_CONFIGURATION:
            if self.report['arm_safety_reads'][name] != self.context['release']['arm_safety_reads'][name]:
                raise RuntimeError('Fresh arm protection/installation/drive configuration changed: ' + name)
        if not self.arm_command_attempted:
            upper = self.context['upper']
            height = (upper['pose'][2] - arm['pose'][2]) * 1000
            if self.context.get('original_return_plan'):
                check_segment(arm, self.context['original_return_plan'])
            else:
                for old in (self.context['final_arm'], self.context['endpoint']):
                    if not same_pose(arm, old) or arm['limits'] != old['limits']:
                        raise RuntimeError('Current arm moved after supported opening; no old-point alignment')
                if (math.dist(upper['pose'][:2], arm['pose'][:2]) * 1000 > .5
                        or orientation_radians(upper['pose'][3:], arm['pose'][3:]) > .005):
                    raise RuntimeError('Fresh start is outside original same-XY/wrist return corridor')
            if not 30 <= height <= 120:
                raise RuntimeError('Fresh start is outside original same-XY/wrist return corridor')
        return arm, tip

    def current_quiet_groups(self, initial):
        self.report['phase'] = 'reading_current_quiet_groups'
        self.save()
        began = self.clock()
        rows = []
        while self.clock() - began < 10:
            joints, pose = self.arm_state()
            row = dict(monotonic=self.clock(), joints=joints, pose=pose)
            quiet = (not rows or max(abs(a - b) for a, b in zip(joints, rows[-1]['joints'])) <= .02
                and math.dist(pose[:3], rows[-1]['pose'][:3]) <= .0002
                and orientation_radians(pose[3:], rows[-1]['pose'][3:]) <= .002)
            if not quiet:
                rows = []
            rows.append(row)
            self.report['current_quiet_groups'] = copy.deepcopy(rows)
            self.report['quiet_feedback_is_not_physical_stop'] = True
            self.save()
            if len(rows) >= 3 and rows[-1]['monotonic'] - rows[0]['monotonic'] >= 1:
                current = self.full_arm()
                if not same_pose(current, initial) or current['limits'] != initial['limits']:
                    raise RuntimeError('Actual current arm changed during quiet readback; no return')
                if self.context.get('original_return_plan'):
                    check_segment(current, self.context['original_return_plan'])
                self.report['current_quiet_group_confirmation'] = dict(count=len(rows),
                    elapsed_seconds=rows[-1]['monotonic'] - rows[0]['monotonic'],
                    fresh_full_arm=current, physical_stop_verified=False,
                    old_last_moving_pose_used_as_current=False)
                self.save()
                return
            self.pause(.5)
        raise TimeoutError('Fresh current arm did not provide three quiet groups; no return')

    def wait_empty_permission(self):
        self.close_hand()
        self.report.update(phase=WAIT_PHASE, permission_index=1,
            SDK_queries_during_input=False, hand_socket_closed_during_input=self.io is None,
            historical_reports_do_not_confirm_current_empty_hand=True)
        self.save()
        began = self.clock()
        path = self.folder / 'permissions/1.json'
        while self.clock() - began < 600:
            if (self.folder / 'abort.json').exists():
                raise KeyboardInterrupt('Parent cancelled empty return')
            if path.exists():
                row = strict_json(path.read_bytes())
                if row.get('index') != 1 or row.get('phase') != WAIT_PHASE:
                    raise RuntimeError('Stale empty-return permission')
                self.report['operator_input'] = row
                self.save()
                if row.get('value') == 'q':
                    raise EOFError('Operator retained pose; no empty-return command')
                if (row.get('value') != 'u'
                        or row.get('shell_on_board_hand_completely_empty_and_original_corridor_clear') is not True
                        or row.get('scene_fixed_and_onsite_stop_ready') is not True
                        or row.get('onsite_actual_arm_stop_confirmed') is not True):
                    raise RuntimeError('Current physical empty hand/corridor not confirmed; no lift')
                return
            self.pause(.15)
        raise TimeoutError('Empty-return permission expired; no motion')

    def run(self):
        initial, tip = self.preflight(None, PREPARE)
        initial_hand = copy.deepcopy(self.report['last_preflight'])
        prepare_readback(initial_hand)
        if self.context.get('original_return_plan'):
            self.current_quiet_groups(initial)
        self.report.update(initial_arm=initial, initial_hand=initial_hand,
            source_records=self.context['source_records'], original_upper=self.context['upper'],
            source_role='upper_target_and_current_pose_comparison_only_not_current_state_or_confirmation',
            historical_clearances_reused=False)
        self.report['explicit_resume_return_requested'] = bool(self.context.get('original_return_plan'))
        self.report['last_moving_source_frame_used_as_current'] = False
        self.save()
        self.wait_empty_permission()
        self.report['phase'] = 'fresh_before_empty_return'
        self.save()
        from taught_model_readonly import collect_model_parameters
        model = collect_model_parameters(self.folder / 'fresh_model_after_permission')
        self.report['fresh_model_after_permission'] = model
        self.save()
        if model_signature(model) != model_signature(self.report['fresh_model']):
            raise RuntimeError('Actual controller identity/DH changed during empty-return confirmation')
        upper = self.context['upper']
        # execute_vertical owns the fresh full preflight after this permission,
        # then checks full arm/limits again immediately before its one MoveL.
        self.plan = None
        final = self.execute_vertical(initial, tip, upper['pose'][2], 'returning_empty')
        final, final_tip = self.preflight(final, PREPARE)
        if final_tip != tip:
            raise RuntimeError('Tip parameters changed after empty return')
        self.report.update(outcome=SUCCESS, phase='completed', final_arm=final,
            final_hand=copy.deepcopy(self.report['last_preflight']), hand_shape='PREPARE',
            execution_ready=False, empty_hand_confirmed_by_current_operator=True,
            hand_register_writes=0, controller_configuration_writes=0,
            object_lift_or_transfer_performed=False, full_visual_cycle_claimed=False,
            physical_stop_verified=False, torque_release_verified=False,
            functional_validation='onsite_empty_return_feedback_only')
        self.authority._completed = True
        self.save()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--folder', type=Path, required=True)
    parser.add_argument('--sdk-dir', type=Path,
        default=Path('/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package'))
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--resume-return-report', action='store_true',
        help='Use the explicitly captured prior return report; never its last moving pose as current')
    args = parser.parse_args(argv)
    if not args.execute:
        parser.error('Explicit empty return required; no connection')
    folder = args.folder
    report = dict(schema_version=1, scope='explicit_empty_return_after_supported_release',
        timestamp_utc=datetime.now(timezone.utc).isoformat(), outcome='starting', phase='preflight',
        sdk_in_parent=False, execution_ready=False, offline_tests_executed=False,
        arm_actuator_writes=0, arm_stop_writes=0, hand_register_writes=0,
        controller_configuration_writes=0, automatic_retry=False, automatic_open=False,
        automatic_return=False, object_lift=False, physical_stop_verified=False,
        torque_release_verified=False, exceptions=[], cleanup_errors=[])

    def save():
        temporary = folder / 'worker_result.tmp'
        temporary.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf8')
        temporary.replace(folder / 'worker_result.json')

    robot = sequence = None
    save()
    try:
        profile = strict_json((ROOT / 'deployment/battery_hand_verified_profile.json').read_bytes())
        context = source_context(folder, profile, args.resume_return_report)
        report['source_context_accepted'] = True
        report['source_records'] = context['source_records']
        save()
        from pregrasp_measurement_worker import source_audit
        report['source_audit'] = source_audit(args.sdk_dir)
        if report['source_audit'] != context['grasp']['source_audit']:
            raise RuntimeError('Current SDK source/native differs; no connection')
        save()
        import fcntl
        import socket
        from inspect_left_hand import connect
        from supported_contact_transport import PhaseAuthority
        from arm_protection_transport import ProtectionClient
        from taught_model_readonly import collect_model_parameters
        from high_trial_tip_readonly import collect
        with (ROOT / 'captures/hand_stage.lock').open('a') as lock, \
             (folder / 'hand_wire.jsonl').open('w', encoding='utf8') as wire, \
             (folder / 'protection_wire.jsonl').open('w', encoding='utf8') as protection_wire, \
             (folder / 'tip_wire.jsonl').open('w', encoding='utf8') as tip_wire, \
             (folder / 'samples.jsonl').open('w', encoding='utf8') as samples:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)

            def append(out, row):
                out.write(json.dumps(row) + '\n')
                out.flush()

            model = collect_model_parameters(folder / 'fresh_model')
            report['fresh_model'] = model
            save()
            if model_signature(model) != model_signature(context['grasp']['fresh_model']):
                raise RuntimeError('Fresh controller identity/DH differs from completed descent')
            authority = PhaseAuthority()

            def factory(seq, guarded):
                if not guarded or seq.io is not None:
                    raise RuntimeError('Single guarded read-only hand session required')
                seq.io = ReadOnlyCycleTransport(socket.create_connection(('169.254.128.18', 8080), timeout=3),
                    lambda row: append(wire, row), authority)
                return CycleHand(seq, seq.io, report, save, samples)

            robot = connect(args.sdk_dir)
            sequence = EmptyReturn(robot, None, profile, report, save, samples, factory,
                folder=folder, authority=authority, context=context,
                tip_reader=lambda: collect(lambda row: append(tip_wire, row)),
                enable_arm_protection=False,
                protection_client=ProtectionClient(lambda row: append(protection_wire, row)))
            try:
                sequence.run()
            except BaseException:
                sequence.fault()
                if sequence.arm_stop_attempted:
                    report['arm_stop_writes'] = 1
                    try:
                        # This diagnostic preserves actual feedback even if
                        # either movement guard is exceeded after the stop ACK.
                        current = Sequence.full_arm(sequence)
                        report['post_stop_current_arm_readback'] = dict(
                            current=current, timestamp_utc=datetime.now(timezone.utc).isoformat(),
                            read_only=True, physical_stop_verified=False,
                            torque_release_verified=False, source_last_moving_pose_reused=False)
                    except BaseException as exc:
                        report.setdefault('post_stop_readback_errors', []).append(dict(
                            error=repr(exc), traceback=traceback.format_exc(),
                            physical_stop_verified=False))
                    save()
                raise
            finally:
                sequence.close_hand()
    except EOFError as exc:
        report.update(outcome='cancelled_no_empty_return', phase='cancelled',
            execution_ready=False, cancellation_reason=str(exc))
    except BaseException as exc:
        report['exceptions'].append(dict(error=repr(exc), traceback=traceback.format_exc()))
        report.update(outcome='aborted_no_further_writes', phase='fault', execution_ready=False)
    finally:
        if robot is not None:
            for name in ('Arm_Socket_Close', 'RM_API_UnInit'):
                try:
                    getattr(robot, name)()
                except BaseException as exc:
                    report['cleanup_errors'].append(dict(method=name, error=repr(exc)))
        if report['cleanup_errors']:
            report['outcome'] = 'failed_cleanup'
        save()
    return 0 if report['outcome'] == SUCCESS else 1


if __name__ == '__main__':
    raise SystemExit(main())
