"""One scoped, onsite lift or explicitly confirmed empty-hand return.

Only this request's hand targets, World-Z +180mm MoveL, recorded-start MoveJ,
and one fault stop are writable. No configuration/reset/clear methods exist here.
"""
import argparse
import copy
import ctypes
from datetime import datetime, timezone
import hashlib
import importlib
import json
import math
from pathlib import Path
import sys
import time
import traceback
import types

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/battery_grasp/src'))
from inspect_left_arm import QUERIES, snapshot
from new_grasp_reference_worker_v1 import (SDK_DEFAULT, NATIVE_SHA, arm_from_queries,
    bracket_delta, json_evidence, source_audit, vector)
from arm_protection_transport import ProtectionClient
from manual_hand_bus_v2 import ManualHandBus
from school_device_identity import same_live_identity
from grasp_rebuild_records_v1 import arm_reference, hand_snapshot, targets
from sdk_environment import collect_sdk_environment
from visual_run_configuration_v3 import (observe_configuration, bind_or_require_frames,
    require_current_configuration)

ORDER = (5, 4, 3, 0, 1, 2)
COMMON = ('start_record', 'wrist_record', 'grip_targets', 'prepare_targets')


def encode(value):
    # Get_Install_Pose returns c_float scalars rather than Python floats.
    # Preserve their actual .value recursively before JSON or vector checks.
    if isinstance(value, ctypes._SimpleCData):
        return encode(value.value)
    if isinstance(value, ctypes.Structure):
        return {name: encode(getattr(value, name)) for name, *_ in value._fields_}
    if isinstance(value, bytes):
        return value.decode('utf8', errors='replace')
    if isinstance(value, (list, tuple, ctypes.Array)):
        return [encode(item) for item in value]
    if isinstance(value, dict):
        return {key: encode(item) for key, item in value.items()}
    return value


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_json(path):
    return json.loads(Path(path).read_bytes())


def canonical(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
        allow_nan=False).encode('utf8')).hexdigest()


def delivery(path):
    inventory = {}
    for line in path.read_text(encoding='ascii').splitlines():
        if not line.strip():
            continue
        expected, name = line.split('  ', 1)
        item = (ROOT/name).resolve()
        if (ROOT not in item.parents or name in inventory or len(expected) != 64
                or sha(item) != expected):
            raise ValueError('Delivery file differs: '+name)
        inventory[name] = expected
    if 'deployment/grasp_rebuild_motion_v1_worker.py' not in inventory:
        raise ValueError('Motion worker is missing from delivery manifest')
    return inventory


def pose_delta(a, b):
    # bracket_delta also computes SO(3), avoiding Euler wraparound comparisons.
    return bracket_delta(a, b)


def match_arm(actual, expected, label):
    delta = pose_delta(expected, actual)
    if (not delta['frame_signatures_unchanged'] or not delta['limits_unchanged']
            or max(abs(v) for v in delta['joint_degrees']) > .08
            or delta['position_distance_m'] > .0005
            or delta['orientation_angle_rad'] > .005):
        raise RuntimeError(label+': current state differs from this recorded state; '+repr(delta))
    return delta


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--manifest', required=True, type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    robot = bus = run = None
    active_motion = False
    stop_attempted = False
    external_drag = False
    mode = None
    report = dict(schema_version=1, scope='new_grasp_rebuild_180mm_lift_or_empty_return',
        timestamp_utc=datetime.now(timezone.utc).isoformat(), phase='preflight',
        outcome='not_completed', physical_object_success=False, execution_ready=False,
        arm_actuator_writes=0, hand_register_writes=0, arm_stop_writes=0,
        controller_configuration_writes=0, physical_stop_verified=False,
        old_fault_last_arm_state_used=False, offline_project_tests_executed=False,
        SDK_cleanup_finished=False, exceptions=[], cleanup_errors=[], sdk_calls=[],
        motion_requests=[], hand_commands=[], arm_samples=[],
        hand_actual_arrival_required=False, normal_status5_is_fault=False,
        tolerances=dict(joints_degrees=.08, endpoint_m=.0005,
            orientation_rad=.005, lift_xy_m=.001, ordinary_reserve_degrees=3,
            drive_reserve_degrees=3))

    def save():
        temporary = output.with_name(output.name+'.pending')
        temporary.write_bytes((json.dumps(json_evidence(report), ensure_ascii=False,
            indent=2, allow_nan=False)+'\n').encode('utf8'))
        temporary.replace(output)

    def append(kind, row):
        nonlocal external_drag
        if row.get('event') == 'external_drag_teach_notification_not_hand_ack':
            external_drag = True
        path = output.with_name(output.stem+'_'+kind+'.jsonl')
        with path.open('a', encoding='utf8', newline='\n') as stream:
            stream.write(json.dumps(json_evidence(row), ensure_ascii=False,
                allow_nan=False)+'\n')

    def phase(name):
        report['phase'] = name
        save()
        print('PHASE:', name, flush=True)

    def check_abort():
        if (output.parent/'abort.json').exists():
            report['parent_abort_request_file'] = str(output.parent/'abort.json')
            raise RuntimeError('Parent requested this action stop; no automatic continuation')

    def call(name, *values, **keywords):
        allowed = set(QUERIES) | {'Get_Joint_Drive_Min_Pos', 'Get_Joint_Drive_Max_Pos',
            'Get_Joint_EN_State', 'Get_Joint_Err_Flag', 'Get_Install_Pose',
            'Get_Joint_Current', 'Get_Joint_Temperature', 'Get_Joint_Voltage',
            'Movel_Cmd', 'Movej_Cmd', 'Move_Stop_Cmd'}
        if name not in allowed:
            raise RuntimeError('SDK capability not granted: '+name)
        row = dict(method=name, monotonic=time.monotonic(), args=values, kwargs=keywords)
        report['sdk_calls'].append(row)
        try:
            raw = getattr(robot, name)(*values, **keywords)
            row['return_type'] = type(raw).__name__
            if isinstance(raw, (tuple, list)):
                row['return_element_types'] = [type(item).__name__ for item in raw]
            row['result'] = encode(raw)
            append('sdk', row)
            return row['result']
        except BaseException as exc:
            row['error'] = repr(exc)
            append('sdk', row)
            raise

    class Reads:
        def __getattr__(self, name):
            if name not in QUERIES:
                raise AttributeError(name)
            return lambda: call(name)

    def protection(label):
        client = ProtectionClient(lambda row: append('protection', dict(label=label, **row)))
        result = {key: getattr(client, key)() for key in
            ('Get_Collision_Stage', 'Get_Self_Collision_Enable')}
        expected = {'Get_Collision_Stage': [0, 8], 'Get_Self_Collision_Enable': [0, True]}
        report[label] = result
        if result != expected:
            raise RuntimeError('Current protection must remain collision8 / selfTrue: '+repr(result))
        return result

    def good_query(name, size):
        raw = call(name)
        if (not isinstance(raw, list) or len(raw) != size+1
                or type(raw[0]) is not int or raw[0] != 0):
            raise RuntimeError('Required SDK read unavailable: '+name+' '+repr(raw))
        return raw[1:]

    def full_arm(label):
        raw = snapshot(Reads())
        report.setdefault('arm_full_queries', []).append(dict(label=label, raw=raw))
        state = arm_from_queries(raw)
        bind_or_require_frames(run, state['frame_signatures'])
        report['last_arm_state'] = copy.deepcopy(state)
        save()
        return state

    def live_arm(template):
        if external_drag:
            raise RuntimeError('External drag teaching occurred during automatic motion')
        raw = call('Get_Current_Arm_State')
        report['last_arm_raw'] = copy.deepcopy(raw)
        if (not isinstance(raw, list) or len(raw) != 5
                or type(raw[0]) is not int or raw[0] != 0):
            raise RuntimeError('Current arm communication is not qualified: '+repr(raw))
        q, p = vector(raw[1], 6, 'live joints'), vector(raw[2], 6, 'live pose')
        if any(type(v) is not int or v != 0 for v in raw[3:]):
            raise RuntimeError('Actual arm/system fault; no automatic recovery: '+repr(raw[3:]))
        result = copy.deepcopy(template)
        result.update(joints=q, pose=p, arm_error=0, system_error=0)
        result['reserve'] = [min(v-a, b-v) for v, a, b in
            zip(q, result['limits']['minimum'], result['limits']['maximum'])]
        report['last_arm_state'] = copy.deepcopy(result)
        sample = dict(monotonic=time.monotonic(), phase=report['phase'], arm=result)
        report['arm_samples'].append(sample)
        append('sdk', dict(event='fresh_arm_sample', **sample))
        return result

    def hand_row(label):
        row = bus.refresh()
        report.setdefault('hand_snapshots', []).append(dict(label=label, row=row))
        hand_snapshot(row, label)
        if row.get('configuration_matched') is not True or row.get('blockers'):
            raise RuntimeError('Current hand configuration/working reads unavailable')
        if external_drag:
            raise RuntimeError('External drag teaching observed during action stage')
        save()
        return row

    def check_targets(current, expected, label):
        if len(current) != 6 or any(abs(a-b) > 2 for a, b in zip(current, expected)):
            raise RuntimeError(label+': current target registers differ; '+repr(current))

    def hand_working(expected):
        # Use the same validated transport and live working registers. Keep all
        # six actual target/position/current/status values, including normal5.
        check_abort()
        io = bus._io
        if io is None or io.broken or bus._logging_error:
            raise RuntimeError('Current hand connection/logging is unavailable')
        row = dict(targets=io.read(1135, 6), positions=io.read(1145, 6),
            current=io.read(1105, 5)+[io.single(1110)], status=io.read(1085, 6),
            monotonic=time.monotonic())
        report['last_hand_working'] = row
        append('hand', dict(event='motion_hand_working_registers', **row))
        check_targets(row['targets'], expected, 'hand target readback')
        if (any(type(v) is not int or v not in (0, 1, 2, 5) for v in row['status'])
                or any(c < 0 or c > limit for c, limit in
                    zip(row['current'], bus._profile['current_limit_ma']))):
            raise RuntimeError('Actual hand protection/current failure; normal status5 is permitted')
        if external_drag:
            raise RuntimeError('External drag teaching observed during action stage')
        return row

    def hand_protection_read():
        # Explicit target writes need no arrival wait, but never chain a new
        # motor command across an actual protection3/4 or overcurrent report.
        io = bus._io
        state = dict(status=io.read(1085, 6),
            current=io.read(1105, 5)+[io.single(1110)], monotonic=time.monotonic())
        append('hand', dict(event='between_finger_protection_read', **state))
        report['last_hand_protection_read'] = state
        if (any(type(v) is not int or v not in (0, 1, 2, 5) for v in state['status'])
                or any(c < 0 or c > limit for c, limit in
                    zip(state['current'], bus._profile['current_limit_ma']))):
            raise RuntimeError('Actual hand protection/current failure between target commands')
        if external_drag or bus._logging_error:
            raise RuntimeError('Exclusive hand command/recording scope lost')
        return state

    def write_shape(desired, before, label):
        desired = targets(desired)
        current = targets(before)
        # The one-use bus capability permits precisely one channel/value write.
        # No actual arrival wait, stall retry or invented finger ceiling exists.
        for ch in ORDER:
            if abs(current[ch]-desired[ch]) <= 2:
                continue
            check_abort()
            if external_drag or bus._logging_error or bus._io is None:
                raise RuntimeError('Hand write authority unavailable')
            bus._serial += 1
            command = dict(label=label, channel=ch, target=desired[ch],
                monotonic=time.monotonic(), attempted=False, acknowledged=False)
            report['hand_commands'].append(command)
            bus._io.grant(ch, desired[ch], bus._serial)
            command['attempted'] = True
            report['hand_register_writes'] += 1
            bus._io.write_group(1135+ch, [desired[ch]])
            command['acknowledged'] = True
            current[ch] = desired[ch]
            readback = bus._io.read(1135, 6)
            command['target_readback'] = readback
            check_targets(readback, current, 'Each explicitly acknowledged hand command')
            append('hand', dict(event='ordered_shape_command', **command))
            hand_protection_read()
            save()
        hand_working(desired)
        report[label+'_targets'] = desired
        return desired

    def check_drive(state, drive):
        reserve = [min(v-a, b-v) for v, a, b in zip(state['joints'],
            drive['minimum'], drive['maximum'])]
        state['drive_reserve'] = reserve
        if any(v < 3 for v in reserve):
            raise RuntimeError('Current drive-limit reserve below3 degrees: '+repr(reserve))

    def require_normal(state, drive):
        check_drive(state, drive)
        if any(v < 3 for v in state['reserve']):
            raise RuntimeError('Current ordinary-limit reserve below3 degrees: '+repr(state['reserve']))

    def stable_current(label):
        before = full_arm(label+'_before')
        time.sleep(.15)
        after = full_arm(label+'_after')
        report[label+'_bracket_delta'] = match_arm(after, before, label+' stable bracket')
        return after

    def move(method, destination, beginning, drive, hand_targets, escape=None):
        nonlocal active_motion
        if method == 'Movel_Cmd':
            if mode != 'lift':
                raise RuntimeError('MoveL capability is only the new180mm lift')
            if (abs(destination[2]-beginning['pose'][2]-.180) > .000001
                    or destination[:2] != beginning['pose'][:2]
                    or destination[3:] != beginning['pose'][3:]):
                raise RuntimeError('MoveL differs from the scoped180mm World-Z lift')
        elif method == 'Movej_Cmd':
            if mode != 'return_empty' or destination != start_arm['joints']:
                raise RuntimeError('MoveJ capability is only recorded-start empty return')
        else:
            raise RuntimeError('No motion capability')
        request = dict(method=method, destination=destination, v=5,
            trajectory_connect=0, r=0, block=False, start=beginning)
        report['motion_requests'].append(copy.deepcopy(request))
        save()
        check_abort()
        # Mark before native call: an uncertain ACK still requires one Stop.
        active_motion = True
        report['arm_actuator_writes'] += 1
        raw = call(method, destination, 5, trajectory_connect=0, r=0, block=False)
        if type(raw) is not int or raw != 0:
            raise RuntimeError('Motion command rejected or ACK uncertain: '+repr(raw))
        began = time.monotonic()
        last_hand = began
        previous = None
        stable_since = None
        stable_count = 0
        while time.monotonic()-began < 120:
            check_abort()
            state = live_arm(beginning)
            check_drive(state, drive)
            if escape is None:
                require_normal(state, drive)
            else:
                for ch, (q, a, b) in enumerate(zip(state['joints'],
                        beginning['joints'], start_arm['joints'])):
                    if not min(a, b)-.1 <= q <= max(a, b)+.1:
                        raise RuntimeError('Return joint left its fresh-to-start bounded interval: J'+str(ch+1))
                    floor = escape['minimum_reserve'][ch]
                    if state['reserve'][ch] < floor:
                        raise RuntimeError('Return escape worsened ordinary-limit reserve: J'+str(ch+1))
            if method == 'Movel_Cmd':
                delta = pose_delta(beginning, state)
                if (math.dist(state['pose'][:2], beginning['pose'][:2]) > .001
                        or delta['orientation_angle_rad'] > .005
                        or not beginning['pose'][2]-.001 <= state['pose'][2] <= destination[2]+.001):
                    raise RuntimeError('Actual lift left its straight vertical pose corridor')
                distance = math.dist(state['pose'][:3], destination[:3])
                reached = distance <= .0005 and delta['orientation_angle_rad'] <= .005
            else:
                endpoint = pose_delta(start_arm, state)
                reached = (max(abs(a-b) for a, b in zip(state['joints'], destination)) <= .08
                    and endpoint['position_distance_m'] <= .0005
                    and endpoint['orientation_angle_rad'] <= .005)
            stationary = previous is not None and (
                max(abs(a-b) for a, b in zip(state['joints'], previous['joints'])) <= .02
                and math.dist(state['pose'][:3], previous['pose'][:3]) <= .0002)
            if reached and stationary:
                if stable_since is None:
                    stable_since = time.monotonic()
                stable_count += 1
            else:
                stable_since = None
                stable_count = 0
            if time.monotonic()-last_hand >= 1:
                hand_working(hand_targets)
                last_hand = time.monotonic()
            if stable_count >= 3 and time.monotonic()-stable_since >= .5:
                # Fresh full GETs catch frame/configuration changes before a
                # completed endpoint can become a reusable successful state.
                final = stable_current('motion_endpoint')
                check_drive(final, drive)
                if method == 'Movej_Cmd':
                    match_arm(final, start_arm, 'recorded-start return endpoint')
                    require_normal(final, drive)
                else:
                    expected = copy.deepcopy(beginning)
                    expected['pose'] = destination[:]
                    endpoint = pose_delta(expected, final)
                    if (endpoint['position_distance_m'] > .0005
                            or endpoint['orientation_angle_rad'] > .005
                            or final['pose'][2]-beginning['pose'][2] < .1795):
                        raise RuntimeError('Full current lift endpoint does not reach180mm')
                    require_normal(final, drive)
                active_motion = False
                return final
            previous = state
            save()
            time.sleep(.1)
        raise RuntimeError('Motion feedback did not reach its recorded endpoint within120 seconds')

    try:
        save()
        report.update(request_file=str(args.request.resolve()), request_sha256=sha(args.request))
        request = read_json(args.request)
        mode = request.get('mode')
        if mode not in ('lift', 'return_empty'):
            raise ValueError('Request mode must be lift or return_empty')
        if mode == 'lift' and request.get('operator_confirms_lift_path_and_sweep_clear') is not True:
            raise ValueError('Explicit confirmation of this180mm lift path and hand sweep is required')
        report.update(mode=mode, request_file=str(args.request.resolve()),
            request_sha256=sha(args.request), delivery_manifest_sha256=sha(args.manifest),
            delivery_source_sha256=delivery(args.manifest.resolve()))
        if request.get('manifest_sha256', report['delivery_manifest_sha256']) != report['delivery_manifest_sha256']:
            raise ValueError('Request manifest digest differs')
        report['request_common_sha256'] = canonical({key: request[key] for key in COMMON})
        start_record, wrist_record = request['start_record'], request['wrist_record']
        start_arm = arm_reference(start_record['arm'], 'recorded start', 3)
        wrist_arm = arm_reference(wrist_record['arm'], 'recorded wrist', 3)
        start_hand = hand_snapshot(start_record['hand'], 'recorded start hand')
        wrist_hand = hand_snapshot(wrist_record['hand'], 'recorded wrist hand')
        grip = targets(request['grip_targets'], 'grip targets')
        prepare = targets(request['prepare_targets'], 'prepare targets')
        if (start_arm['frame_signatures'] != wrist_arm['frame_signatures']
                or start_arm['limits'] != wrist_arm['limits']):
            raise ValueError('Start/wrist frame and joint configurations differ')
        check_targets(wrist_hand['targets'], prepare, 'recorded wrist prepare')
        previous_result = None
        if mode == 'return_empty':
            if request.get('operator_confirms_empty_original_shell_supported_and_return_sweep_clear') is not True:
                raise ValueError('Explicit empty/support/full-return-sweep confirmation is required')
            if request.get('reason') != 'cancel_before_grip':
                prior_request_path = Path(request['previous_lift_request_file']).resolve()
                prior_result_path = Path(request['previous_lift_result_file']).resolve()
                prior_request = read_json(prior_request_path)
                previous_result = read_json(prior_result_path)
                digest = sha(prior_request_path)
                if (digest != request['previous_lift_request_sha256']
                        or previous_result.get('request_sha256') != digest
                        or prior_request.get('mode') != 'lift'
                        or any(prior_request.get(key) != request[key] for key in COMMON)):
                    raise ValueError('Previous lift provenance does not match this exact cycle')
                if previous_result.get('SDK_cleanup_finished') is not True:
                    raise RuntimeError('Prior SDK cleanup is uncertain; onsite recovery required')
                report['previous_lift_provenance'] = dict(request_file=str(prior_request_path),
                    request_sha256=digest, result_file=str(prior_result_path),
                    result_sha256=sha(prior_result_path),
                    outcome=previous_result.get('outcome'), fault_last_state_used=False)
        report['sdk_provenance'] = source_audit(SDK_DEFAULT)
        # Raw configuration GETs accept a text stream, unlike ProtectionClient
        # and ManualHandBus, whose log arguments are callbacks.
        with output.with_name(output.stem+'_configuration.jsonl').open(
                'a', encoding='utf8', newline='\n') as configuration_wire:
            run = observe_configuration(configuration_wire)
        report['frozen_configuration'] = run
        protection('protection_before')
        package = types.ModuleType('battery_grasp_rebuild_motion_scoped_sdk')
        package.__path__ = [str(SDK_DEFAULT)]
        sys.modules[package.__name__] = package
        sdk = importlib.import_module(package.__name__+'.robotic_arm')
        robot = sdk.Arm.__new__(sdk.Arm)
        sdk.Arm.__init__(robot, sdk.RM65, '169.254.128.18')
        report['sdk_socket_state_raw'] = encode(robot.Arm_Socket_State())
        if type(report['sdk_socket_state_raw']) is not int or report['sdk_socket_state_raw'] != 0:
            raise RuntimeError('Current SDK socket unavailable')
        metadata = collect_sdk_environment(robot)
        report['sdk_environment'] = metadata
        if (not metadata.get('metadata_complete') or metadata.get('api_version') != '4.3.1'
                or metadata.get('loaded_native', {}).get('sha256') != NATIVE_SHA
                or metadata.get('controller_run_mode_raw', {}).get('data') != [1]):
            raise RuntimeError('Reviewed native/API/real run mode differs')
        initial = stable_current('fresh_startup')
        if (initial['frame_signatures'] != wrist_arm['frame_signatures']
                or initial['limits'] != wrist_arm['limits']):
            raise RuntimeError('Fresh frame/joint configuration differs from this cycle records')
        drive = dict(minimum=vector(good_query('Get_Joint_Drive_Min_Pos', 1)[0], 6, 'drive min'),
            maximum=vector(good_query('Get_Joint_Drive_Max_Pos', 1)[0], 6, 'drive max'))
        if any(a >= b for a, b in zip(drive['minimum'], drive['maximum'])):
            raise RuntimeError('Malformed drive limits')
        report['drive_limits'] = drive
        enabled = good_query('Get_Joint_EN_State', 1)[0]
        errors, brakes = good_query('Get_Joint_Err_Flag', 2)
        if enabled != [1]*6 or errors != [0]*6 or brakes != [0]*6:
            raise RuntimeError('Actual joint enable/error/brake state is not ready')
        install = good_query('Get_Install_Pose', 3)
        if any(abs(a-b) > .001 for a, b in zip(vector(install, 3, 'installation'), [0, 45, 0])):
            raise RuntimeError('Actual installation differs from reviewed0/45/0')
        for name in ('Get_Joint_Current', 'Get_Joint_Temperature', 'Get_Joint_Voltage'):
            report.setdefault('arm_safety_reads', {})[name] = vector(good_query(name, 1)[0], 6, name)
        bus = ManualHandBus(lambda row: append('hand', row))
        connection = bus.connect_read()
        report['hand_connect_read'] = connection
        hand = hand_snapshot(connection, 'fresh hand startup')
        if connection.get('configuration_matched') is not True or connection.get('blockers'):
            raise RuntimeError('Actual hand full configuration differs')
        if any(not same_live_identity(hand.get('identity'), saved.get('identity'))
                for saved in (start_hand, wrist_hand)):
            raise RuntimeError('Recorded/fresh static hand identity or live diagnostic differs')
        for key in ('current_limit_ma', 'speeds', 'force_targets'):
            if hand.get(key) != wrist_hand.get(key) or hand.get(key) != start_hand.get(key):
                raise RuntimeError('Recorded/fresh hand configuration differs: '+key)
        # Configuration/hand reads may take seconds; scope all later writes to
        # another healthy current bracket after those reads, not its old sample.
        now = stable_current('after_hand_connection')
        match_arm(now, initial, 'exclusive startup while hand was being read')
        initial = now
        check_abort()
        report['fresh_startup'] = dict(arm=initial, hand=connection,
            source='fresh_healthy_current_bracket_not_fault_last_state')
        if mode == 'lift':
            require_normal(initial, drive)
            report['wrist_match'] = match_arm(initial, wrist_arm, 'fresh recorded wrist')
            check_targets(hand['targets'], prepare, 'fresh pregrasp targets')
            phase('applying_grip_targets_then_lifting_180mm')
            write_shape(grip, hand['targets'], 'grip')
            # Finger target ACK/readback, not actual finger arrival, starts lift.
            fresh = live_arm(initial)
            match_arm(fresh, wrist_arm, 'wrist immediately before lift')
            require_normal(fresh, drive)
            destination = fresh['pose'][:]
            destination[2] += .180
            report['lift_start'] = fresh
            upper = move('Movel_Cmd', destination, fresh, drive, grip)
            upper_hand = hand_row('qualified upper hand')
            check_targets(upper_hand['snapshot']['targets'], grip, 'upper grip targets')
            report.update(qualified_upper=upper, qualified_upper_hand=upper_hand,
                physical_object_success=False, phase='completed',
                outcome='lift_completed_waiting_operator_result')
        else:
            check_drive(initial, drive)
            require_normal(start_arm, drive)
            if request.get('reason') == 'cancel_before_grip':
                if request.get('expected_wrist') != wrist_arm:
                    raise ValueError('Cancel return requires the exact recorded wrist expectation')
                report['return_start_match'] = match_arm(initial, wrist_arm, 'cancel return start')
                check_targets(hand['targets'], prepare, 'cancel pregrasp targets')
            elif previous_result.get('outcome') == 'lift_completed_waiting_operator_result':
                expected = previous_result.get('qualified_upper')
                if request.get('expected_upper') != expected or not isinstance(expected, dict):
                    raise ValueError('Completed return requires its exact qualified upper')
                report['return_start_match'] = match_arm(initial, expected, 'completed lift upper')
                check_targets(hand['targets'], grip, 'completed lift current grip targets')
            else:
                # A failed saved arm state is diagnostic only. Qualify the
                # actual fresh, error-free, stationary sample solely for return.
                if not previous_result.get('arm_actuator_writes'):
                    match_arm(initial, wrist_arm, 'failed-before-motion wrist')
                else:
                    delta = pose_delta(wrist_arm, initial)
                    if (math.dist(initial['pose'][:2], wrist_arm['pose'][:2]) > .001
                            or delta['orientation_angle_rad'] > .005
                            or not wrist_arm['pose'][2]-.001 <= initial['pose'][2] <= wrist_arm['pose'][2]+.181):
                        raise RuntimeError('Fresh failed-lift return state is outside its own lift corridor')
                if hand['targets'] not in (prepare, grip):
                    raise RuntimeError('Fresh failed-lift targets are neither this prepare nor grip')
                report['recovery_start_qualified_for_return_only'] = dict(arm=copy.deepcopy(initial),
                    hand=copy.deepcopy(connection), fault_last_state_used=False,
                    full_empty_return_operator_confirmation=True)
            escape = None
            if any(v < 3 for v in initial['reserve']):
                if any(v < 0 for v in initial['reserve']):
                    raise RuntimeError('Fresh return state is outside ordinary limits')
                escape = dict(minimum_reserve=[max(0, v-.05) if v < 3 else 3
                    for v in initial['reserve']],
                    fresh_start_joints=initial['joints'], recorded_target_joints=start_arm['joints'],
                    interval_tolerance_degrees=.1, allowed_only_for_this_empty_return=True)
                report['bounded_return_escape'] = escape
            phase('restoring_prepare_before_empty_return')
            write_shape(prepare, hand['targets'], 'return_prepare')
            fresh = live_arm(initial)
            match_arm(fresh, initial, 'stationary return start after preparing fingers')
            check_drive(fresh, drive)
            phase('returning_empty_to_recorded_start')
            restored = move('Movej_Cmd', start_arm['joints'][:], fresh, drive, prepare, escape)
            phase('restoring_recorded_start_hand_targets')
            write_shape(start_hand['targets'], prepare, 'recorded_start_hand')
            final_hand = hand_row('restored recorded start hand')
            check_targets(final_hand['snapshot']['targets'], start_hand['targets'], 'restored start hand')
            final_arm = stable_current('restored_start')
            match_arm(final_arm, start_arm, 'restored start')
            require_normal(final_arm, drive)
            report.update(qualified_return=final_arm, qualified_return_hand=final_hand,
                phase='completed', outcome='returned_to_recorded_start')
        save()
    except BaseException as exc:
        report['exceptions'].append(dict(error=repr(exc), traceback=traceback.format_exc(),
            phase=report.get('phase')))
        report.update(failed_phase=report.get('phase'), phase='fault',
            outcome='needs_onsite_recovery' if mode == 'return_empty' else
                'lift_not_completed_waiting_operator_recovery', physical_object_success=False)
        if active_motion and robot is not None and not stop_attempted:
            stop_attempted = True
            report['arm_stop_writes'] += 1
            try:
                report['fault_stop_raw'] = call('Move_Stop_Cmd', block=False)
            except BaseException as stop_exc:
                report['fault_stop_error'] = repr(stop_exc)
        save()
    finally:
        if bus is not None:
            try:
                report['hand_close'] = bus.close()
            except BaseException as exc:
                report['cleanup_errors'].append(dict(method='manual_hand_close', error=repr(exc)))
        if robot is not None:
            for name in ('Arm_Socket_Close', 'RM_API_UnInit'):
                try:
                    raw = encode(getattr(robot, name)())
                    report.setdefault('sdk_cleanup_results', []).append(dict(method=name, raw=raw))
                    if name == 'RM_API_UnInit' and (type(raw) is not int or raw != 0):
                        raise RuntimeError('API uninitialization returned nonzero/unknown status')
                except BaseException as exc:
                    report['cleanup_errors'].append(dict(method=name, error=repr(exc)))
            report['SDK_cleanup_finished'] = not report['cleanup_errors']
        if run is not None and (robot is None or report['SDK_cleanup_finished']):
            try:
                with output.with_name(output.stem+'_configuration.jsonl').open(
                        'a', encoding='utf8', newline='\n') as configuration_wire:
                    require_current_configuration(run, configuration_wire)
                protection('protection_after')
            except BaseException as exc:
                report['cleanup_errors'].append(dict(method='final_readonly_configuration', error=repr(exc)))
        if report['cleanup_errors'] and report.get('phase') == 'completed':
            report.update(failed_phase='final_cleanup_or_configuration', phase='fault',
                outcome='needs_onsite_recovery', physical_object_success=False)
            report.pop('qualified_upper', None)
            report.pop('qualified_upper_hand', None)
            report.pop('qualified_return', None)
            report.pop('qualified_return_hand', None)
        report['closed_timestamp_utc'] = datetime.now(timezone.utc).isoformat()
        save()
    print('SAVED:', output, 'OUTCOME:', report['outcome'], flush=True)
    return 0 if report['phase'] == 'completed' and not report['exceptions'] and not report['cleanup_errors'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
