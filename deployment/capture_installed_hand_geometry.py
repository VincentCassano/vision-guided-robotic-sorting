#!/usr/bin/env python3
"""Capture installed arm/hand references without motion or configuration writes."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import socket
import time
import traceback
from datetime import datetime, timezone

from inspect_left_arm import QUERIES, snapshot
from inspect_left_hand import connect
from run_school_battery_hand import Hand
from school_battery_transport import Transport

ROOT = Path(__file__).resolve().parents[1]
MISSING_GEOMETRY = [
    'grasp_tcp_transform_from_arm_tip',
    'little_finger_lowest_point_for_hand_and_wrist_pose',
    'whole_hand_swept_envelope',
    'support_plane_world_z_mm',
    'height_and_orientation_dependent_y_envelope',
    'contact_stop_and_release_validation',
]


def _number(value, label):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(label + ': expected finite number')
    return value


def _vector(value, size, label):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(label + ': wrong vector length')
    return [_number(v, label) for v in value]


def _words(value, size, label):
    values = _vector(value, size, label)
    if any(type(v) is not int or not 0 <= v <= 65535 for v in values):
        raise ValueError(label + ': expected unsigned register words')
    return values


def _query(queries, name, size):
    row = queries.get(name) if isinstance(queries, dict) else None
    if not isinstance(row, dict) or row.get('ok') is not True or type(row.get('status')) is not int or row['status'] != 0:
        raise ValueError(name + ': unsuccessful SDK query')
    data = row.get('data')
    if not isinstance(data, (list, tuple)) or len(data) != size:
        raise ValueError(name + ': malformed SDK return')
    return data


def _frame(value, label):
    if not isinstance(value, dict):
        raise ValueError(label + ': malformed frame')
    names = value.get('frame_name')
    name = names.get('name') if isinstance(names, dict) else None
    if not isinstance(name, str) or not name:
        raise ValueError(label + ': missing frame name')
    pose = value.get('pose')
    if not isinstance(pose, dict):
        raise ValueError(label + ': missing frame pose')
    for group, axes in (('position', ('x', 'y', 'z')), ('euler', ('rx', 'ry', 'rz'))):
        row = pose.get(group)
        if not isinstance(row, dict) or any(k not in row for k in axes):
            raise ValueError(label + ': malformed frame ' + group)
        for axis in axes:
            _number(row[axis], label + '.' + group + '.' + axis)
    # The old SDK does not promise a valid quaternion from these getters.
    # Preserve it as raw evidence, but do not derive geometry from it.
    for key in ('payload', 'x', 'y', 'z'):
        if key in value:
            _number(value[key], label + '.' + key)
    return name


def _arm(queries):
    frames = {kind: _query(queries, method, 1)[0] for kind, method in (
        ('work', 'Get_Current_Work_Frame'), ('tool', 'Get_Current_Tool_Frame'))}
    frame_names = {kind: _frame(value, kind) for kind, value in frames.items()}
    frame_signatures = {
        kind: dict(name=frame_names[kind],
                   position={axis: value['pose']['position'][axis] for axis in ('x','y','z')},
                   euler={axis: value['pose']['euler'][axis] for axis in ('rx','ry','rz')},
                   payload={key: value[key] for key in ('payload','x','y','z') if key in value})
        for kind, value in frames.items()
    }
    values = _query(queries, 'Get_Current_Arm_State', 4)
    joints = _vector(values[0], 6, 'arm joints')
    pose = _vector(values[1], 6, 'arm pose')
    for error in values[2:]:
        if type(error) is not int or error != 0:
            raise ValueError('Nonzero or malformed arm/system error: ' + repr(values[2:]))
    vectors = {}
    for key, method in (('minimum', 'Get_Joint_Min_Pos'), ('maximum', 'Get_Joint_Max_Pos'),
                        ('speed', 'Get_Joint_Speed'), ('acceleration', 'Get_Joint_Acc')):
        vectors[key] = _vector(_query(queries, method, 1)[0], 6, method)
    if any(a >= b for a, b in zip(vectors['minimum'], vectors['maximum'])):
        raise ValueError('Invalid joint limit ranges')
    if any(v <= 0 for v in vectors['speed'] + vectors['acceleration']):
        raise ValueError('Invalid joint speed/acceleration limits')
    reserve = [min(q - low, high - q) for q, low, high in
               zip(joints, vectors['minimum'], vectors['maximum'])]
    return dict(frames=frames, frame_signatures=frame_signatures, frame_names=frame_names, pose=pose, joints=joints,
                limits=vectors, reserve=reserve, arm_error=values[2], system_error=values[3])


def analyze_geometry(beforequeries, afterqueries, hand_preflight, profile):
    """Validate a stationary diagnostic bracket; never approves a trajectory."""
    result = dict(diagnostic_pass=False, execution_ready=False, errors=[],
                  missing_geometry=MISSING_GEOMETRY[:], hand_pose_classification=None)
    try:
        if not isinstance(profile, dict) or type(profile.get('schema_version')) is not int or profile['schema_version'] != 1:
            raise ValueError('Unsupported or missing profile schema')
        identity = _words(profile.get('device_identity'), 8, 'profile identity')
        limits = _vector(profile.get('current_limit_ma'), 6, 'profile current limits')
        if any(v <= 0 for v in limits):
            raise ValueError('Invalid profile current limits')
        poses = profile.get('poses')
        if not isinstance(poses, dict) or not poses:
            raise ValueError('Missing verified hand poses')
        poses = {name: _words(values, 6, 'profile pose ' + str(name)) for name, values in poses.items()}
        try:
            before = _arm(beforequeries)
        except (ValueError,TypeError,KeyError) as exc:
            raise ValueError('Before arm snapshot: '+str(exc))
        try:
            after = _arm(afterqueries)
        except (ValueError,TypeError,KeyError) as exc:
            raise ValueError('After arm snapshot: '+str(exc))
        result['frame_names'] = after['frame_names']
        result['current_arm'] = dict(tcp_position_mm=[v * 1000 for v in after['pose'][:3]],
                                    tcp_rpy_rad=after['pose'][3:], joint_degrees=after['joints'],
                                    arm_error=after['arm_error'], system_error=after['system_error'],
                                    joint_reserve_degrees=after['reserve'], joint_reserve_requirement_degrees=3,
                                    queried_joint_limits=after['limits'])
        if after['frame_names'] != {'work': 'World', 'tool': 'Arm_Tip'}:
            result['errors'].append('Unexpected active work/tool reference; expected World / Arm_Tip')
        if before['frame_signatures'] != after['frame_signatures']:
            result['errors'].append('Work/tool frame configuration changed during the bracket')
        if before['limits'] != after['limits']:
            result['errors'].append('Joint limits/speed/acceleration changed during the bracket')
        if any(v < 3 for v in before['reserve'] + after['reserve']):
            result['errors'].append('Current joints have less than 3 degrees of reserve')
        position_drift = math.sqrt(sum((a - b) ** 2 for a, b in zip(before['pose'][:3], after['pose'][:3]))) * 1000
        orientation_drift = [abs(math.atan2(math.sin(a - b), math.cos(a - b)))
                             for a, b in zip(before['pose'][3:], after['pose'][3:])]
        joint_drift = [abs(a - b) for a, b in zip(before['joints'], after['joints'])]
        result['bracket_drift'] = dict(tcp_position_mm=position_drift,
                                       tcp_orientation_rad=orientation_drift, joint_degrees=joint_drift)
        if position_drift > .5 or any(v > .005 for v in orientation_drift) or any(v > .2 for v in joint_drift):
            result['errors'].append('Arm changed during sequential diagnostic readings')
        if not isinstance(hand_preflight, dict):
            raise ValueError('Missing hand preflight')
        if _words(hand_preflight.get('identity'), 8, 'hand identity') != identity:
            result['errors'].append('Unexpected hand identity')
        status = _words(hand_preflight.get('status'), 6, 'hand status')
        if status != [2] * 6:
            result['errors'].append('Hand is not in reached state 2 on all channels')
        forces = _words(hand_preflight.get('force_targets'), 5, 'hand force targets')
        if any(forces):
            result['errors'].append('Nonzero hand force-control targets')
        targets = _words(hand_preflight.get('targets'), 6, 'hand targets')
        positions = _words(hand_preflight.get('positions'), 6, 'hand positions')
        currents = _vector(hand_preflight.get('current'), 6, 'hand current')
        speeds = _words(hand_preflight.get('speeds'), 6, 'hand speeds')
        if any(v <= 0 for v in speeds):
            result['errors'].append('Invalid hand speed setting')
        if any(v < 0 or v > limit for v, limit in zip(currents, limits)):
            result['errors'].append('Hand measured current outside recorded controller limits')
        if any(abs(a - b) > 150 for a, b in zip(targets, positions)):
            result['errors'].append('Hand targets and feedback differ by more than 150')
        matches = [name for name, pose in poses.items()
                   if all(abs(a - b) <= 2 for a, b in zip(targets, pose))
                   and all(abs(a - b) <= 150 for a, b in zip(positions, pose))]
        if len(matches) == 1:
            result['hand_pose_classification'] = matches[0]
        else:
            result['errors'].append('Hand pose does not uniquely match the verified profile')
        result['diagnostic_pass'] = not result['errors']
    except (ValueError, TypeError, KeyError, OverflowError) as exc:
        result['errors'].append(str(exc))
    return result


def _json_safe(value):
    if isinstance(value, dict):
        return {str(k): _json_safe(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json_safe(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return repr(value)
    return value


def collect_arm(robot, phase, report, save):
    report.setdefault('arm_query_brackets', {})[phase] = dict(started_monotonic=time.monotonic())
    row = report['arm_query_brackets'][phase]
    save()
    try:
        row['queries'] = snapshot(robot)
    except (Exception, KeyboardInterrupt) as exc:
        row['error'] = repr(exc)
        row['traceback'] = traceback.format_exc()
        if isinstance(exc, KeyboardInterrupt):
            raise
    finally:
        row['finished_monotonic'] = time.monotonic()
        save()
    return row.get('queries')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--confirm-stationary-exclusive', action='store_true')
    parser.add_argument('--sdk-dir', type=Path, default=Path('/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package'))
    parser.add_argument('--profile', type=Path, default=Path(__file__).with_name('battery_hand_verified_profile.json'))
    args = parser.parse_args(argv)
    if not args.confirm_stationary_exclusive:
        parser.error('Confirm arm and hand stationary, with other arm/hand control scripts stopped; no movements are requested')
    folder = ROOT / 'captures' / ('installed_hand_geometry_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True, exist_ok=False)
    report = dict(schema_version=1, read_only=True, execution_ready=False, diagnostic_pass=False,
                  outcome='starting', timestamp_utc=datetime.now(timezone.utc).isoformat(),
                  ip='169.254.128.18', sdk_directory=str(args.sdk_dir), query_methods=list(QUERIES),
                  missing_geometry=MISSING_GEOMETRY[:], exceptions=[], cleanup_errors=[],
                  units=dict(tcp_position='m', tcp_orientation='rad', joint_angles='degree'),
                  arm_connection_strategy='one_sdk_session_for_both_brackets',
                  note='One SDK session: before/hand/after sequential reads. Current joint reserve is not trajectory approval. No geometry is inferred from TCP alone.')

    def save():
        path = folder / 'result.tmp'
        path.write_text(json.dumps(_json_safe(report), indent=2, ensure_ascii=False, allow_nan=False) + '\n', encoding='utf-8')
        path.replace(folder / 'result.json')

    save()
    io = None
    robot = None
    try:
        names = ('capture_installed_hand_geometry.py', 'inspect_left_arm.py', 'inspect_left_hand.py',
                 'arm_sdk_reviewed_hashes.json', 'school_battery_transport.py',
                 'run_school_battery_hand.py', 'run_school_battery.py', 'school_battery_pose.json')
        report['source_sha256'] = {name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
                                   for name in names}
        report['profile_sha256'] = hashlib.sha256(args.profile.read_bytes()).hexdigest()
        profile = json.loads(args.profile.read_text(encoding='utf-8'))
        report['profile'] = profile
        import fcntl
        with (ROOT / 'captures' / 'hand_stage.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            robot = connect(args.sdk_dir)
            before = collect_arm(robot, 'before', report, save)
            hand_preflight = None
            with (folder / 'wire.jsonl').open('w', encoding='utf-8') as wire, (folder / 'samples.jsonl').open('w', encoding='utf-8') as samples:
                def log(row):
                    wire.write(json.dumps(row) + '\n'); wire.flush()
                try:
                    io = Transport(socket.create_connection(('169.254.128.18', 8080), timeout=3), log)
                    hand = Hand(io, report, save, samples)
                    hand.configuration()
                    hand_preflight = hand.preflight()
                    report['hand_preflight'] = hand_preflight
                    report['hand_angle_raw'] = hand.read('angle_raw', 1165, 6)
                    report['hand_force_feedback_raw'] = hand.read('force_feedback_raw', 1175, 5)
                    save()
                except (Exception, KeyboardInterrupt) as exc:
                    report['exceptions'].append(dict(phase='hand', error=repr(exc), traceback=traceback.format_exc()))
                    if isinstance(exc, KeyboardInterrupt):
                        raise
                finally:
                    if io is not None:
                        io.disable_motion()
                        try:
                            io.sock.close()
                        except Exception as exc:
                            report['cleanup_errors'].append(dict(phase='hand', method='socket.close', error=repr(exc)))
                        io = None
                    save()
            after = collect_arm(robot, 'after', report, save)
            analysis = analyze_geometry(before, after, hand_preflight, profile)
            report['analysis'] = analysis
            report['diagnostic_pass'] = analysis['diagnostic_pass'] and not report['exceptions'] and not report['cleanup_errors']
            report['outcome'] = 'read_only_complete' if report['diagnostic_pass'] else 'read_only_diagnostic_failed'
    except (Exception, KeyboardInterrupt) as exc:
        report['exceptions'].append(dict(phase='capture', error=repr(exc), traceback=traceback.format_exc()))
        report['outcome'] = 'read_only_capture_failed'
    finally:
        if robot is not None:
            for method in ('Arm_Socket_Close','RM_API_UnInit'):
                try:
                    getattr(robot,method)()
                except Exception as exc:
                    report['cleanup_errors'].append(dict(phase='sdk_session',method=method,error=repr(exc)))
            if report['cleanup_errors']:
                report['diagnostic_pass']=False
                report['outcome']='read_only_diagnostic_failed'
        report['execution_ready'] = False
        save()
        print('SAVED:', folder, 'OUTCOME:', report['outcome'], 'DIAGNOSTIC_PASS:', report['diagnostic_pass'], flush=True)
        print('READ ONLY. No arm/hand motion, frame changes, power, calibration or protection writes.', flush=True)
    return 0 if report['diagnostic_pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
