"""One onsite read-only bracket for a newly taught seated pregrasp reference.

No hand, arm-motion, reset, frame, payload or protection-setting method is called.
The parent owns its persistent UI; this isolated worker only returns a record.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import importlib
import json
import math
from pathlib import Path
import platform
import sys
import time
import traceback
import types

from inspect_left_arm import QUERIES, encode, snapshot
from arm_protection_transport import ProtectionClient

ROOT = Path(__file__).resolve().parents[1]
SDK_DEFAULT = Path('/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package')
NATIVE_SHA = '052f94a92d5c7320f227ea811916375fa0d352250329570bd50e0c7abe603ad5'
NATIVE_BYTES = 690912


def json_evidence(value):
    """Represent non-finite raw SDK fields explicitly, never guess zero."""
    if isinstance(value, dict):
        return {key: json_evidence(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_evidence(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return dict(raw_nonfinite_float=repr(value), valid_geometry_number=False)
    return value


def vector(value, size, label):
    if (not isinstance(value, (list, tuple)) or len(value) != size
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in value)):
        raise ValueError(label + ': finite vector required')
    return list(value)


def query(rows, name, size):
    row = rows.get(name, {})
    data = row.get('data')
    if (row.get('ok') is not True or type(row.get('status')) is not int
            or row['status'] != 0 or not isinstance(data, (list, tuple)) or len(data) != size):
        raise ValueError(name + ': unsuccessful or malformed query')
    return data


def arm_from_queries(rows):
    frames = {kind: query(rows, method, 1)[0] for kind, method in (
        ('work', 'Get_Current_Work_Frame'), ('tool', 'Get_Current_Tool_Frame'))}
    names, signatures = {}, {}
    for kind, value in frames.items():
        if not isinstance(value, dict):
            raise ValueError(kind + ': malformed frame')
        name = value.get('frame_name', {}).get('name')
        if not isinstance(name, str) or not name:
            raise ValueError(kind + ': missing frame name')
        names[kind] = name
        pose = value.get('pose', {})
        position = pose.get('position', {})
        euler = pose.get('euler', {})
        vector([position.get(k) for k in ('x', 'y', 'z')], 3, kind + ' frame position')
        vector([euler.get(k) for k in ('rx', 'ry', 'rz')], 3, kind + ' frame Euler')
        payload = {key: value[key] for key in ('payload', 'x', 'y', 'z') if key in value}
        vector(list(payload.values()), len(payload), kind + ' payload fields')
        signatures[kind] = dict(name=name,
            position={axis: position[axis] for axis in ('x', 'y', 'z')},
            euler={axis: euler[axis] for axis in ('rx', 'ry', 'rz')}, payload=payload)
    if names != {'work': 'World', 'tool': 'Arm_Tip'}:
        raise ValueError('Current frames must be World / Arm_Tip')
    values = query(rows, 'Get_Current_Arm_State', 4)
    joints = vector(values[0], 6, 'arm joints')
    pose = vector(values[1], 6, 'arm pose')
    if any(type(v) is not int or v != 0 for v in values[2:]):
        raise ValueError('Arm/system error: ' + repr(values[2:]))
    limits = {key: vector(query(rows, method, 1)[0], 6, method) for key, method in (
        ('minimum', 'Get_Joint_Min_Pos'), ('maximum', 'Get_Joint_Max_Pos'),
        ('speed', 'Get_Joint_Speed'), ('acceleration', 'Get_Joint_Acc'))}
    if (any(a >= b for a, b in zip(limits['minimum'], limits['maximum']))
            or any(v <= 0 for v in limits['speed'] + limits['acceleration'])):
        raise ValueError('Invalid ordinary joint limits')
    reserve = [min(q-a, b-q) for q, a, b in zip(joints, limits['minimum'], limits['maximum'])]
    if any(v < 0 for v in reserve):
        raise ValueError('Current joint outside its ordinary limits')
    return dict(frames=frames, frame_names=names, frame_signatures=signatures,
        joints=joints, pose=pose, limits=limits, reserve=reserve,
        arm_error=values[2], system_error=values[3])


def rotation(rpy):
    # Same Rz Ry Rx convention as battery_grasp.real_grasp.rotation.
    x, y, z = rpy
    cx, cy, cz = math.cos(x), math.cos(y), math.cos(z)
    sx, sy, sz = math.sin(x), math.sin(y), math.sin(z)
    return [[cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx],
            [sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx], [-sy, cy*sx, cy*cx]]


def bracket_delta(before, after):
    first, second = rotation(before['pose'][3:]), rotation(after['pose'][3:])
    trace = sum(first[i][j]*second[i][j] for i in range(3) for j in range(3))
    return dict(joint_degrees=[b-a for a, b in zip(before['joints'], after['joints'])],
        position_m=[b-a for a, b in zip(before['pose'][:3], after['pose'][:3])],
        position_distance_m=math.dist(before['pose'][:3], after['pose'][:3]),
        orientation_angle_rad=math.acos(max(-1., min(1., (trace-1.)/2.))),
        frame_signatures_unchanged=before['frame_signatures'] == after['frame_signatures'],
        limits_unchanged=before['limits'] == after['limits'])


def source_audit(sdk_dir):
    if platform.system() != 'Linux' or platform.machine() != 'aarch64':
        raise RuntimeError('Onsite Linux aarch64 worker only; no SDK loaded')
    expected = json.loads((ROOT/'deployment/arm_sdk_reviewed_hashes.json').read_bytes())
    actual = {name: hashlib.sha256((sdk_dir/name).read_bytes()).hexdigest() for name in expected}
    if not expected or actual != expected:
        raise RuntimeError('Reviewed SDK source hashes differ; no SDK loaded')
    native = sdk_dir/'libRM_Base.so'
    raw = native.read_bytes()
    if (len(raw) != NATIVE_BYTES or hashlib.sha256(raw).hexdigest() != NATIVE_SHA
            or raw[:6] != b'\x7fELF\x02\x01' or int.from_bytes(raw[18:20], 'little') != 183):
        raise RuntimeError('Reviewed aarch64 native SDK differs; no SDK loaded')
    return dict(sdk_directory=str(sdk_dir), sdk_source_sha256=actual,
        native_sha256=NATIVE_SHA, native_bytes=len(raw), host_machine=platform.machine())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--sdk-dir', type=Path, default=SDK_DEFAULT)
    args = parser.parse_args()
    output, sdk_dir = args.output.resolve(), args.sdk_dir.resolve()
    robot = None
    report = dict(schema_version=1, scope='new_seated_pregrasp_current_arm_readonly_bracket',
        timestamp_utc=datetime.now(timezone.utc).isoformat(), read_only=True,
        reference_qualified=False, reference_arm=None, execution_ready=False,
        arm_actuator_writes=0, arm_stop_writes=0, hand_register_writes=0,
        controller_configuration_writes=0, physical_stop_verified=False,
        offline_project_tests_executed=False, SDK_cleanup_finished=False,
        raw_before=None, raw_after=None, queries={}, sdk_calls=[], protection_wire=[],
        exceptions=[], cleanup_errors=[], warnings=[], outcome='reading',
        protection_ready_for_later_motion=False,
        tolerances=dict(joint_degrees=.08, position_m=.0005, orientation_rad=.005),
        role_confirmation_required_from_parent=True,
        seated_shell_or_noncontact_geometry_verified=False)

    def save():
        output.parent.mkdir(parents=True, exist_ok=True)
        pending = output.with_name(output.name+'.pending')
        pending.write_bytes((json.dumps(json_evidence(report), ensure_ascii=False, indent=2,
            allow_nan=False)+'\n').encode('utf8'))
        pending.replace(output)

    class LoggedReads:
        # Only the seven reviewed SDK getters are exposed to snapshot().
        def __getattr__(self, name):
            if name not in QUERIES:
                raise AttributeError('Reference reader permits only reviewed GET queries')
            def call():
                row = dict(method=name, started_monotonic=time.monotonic())
                report['sdk_calls'].append(row)
                try:
                    raw = getattr(robot, name)()
                    row['raw'] = encode(raw)
                    return raw
                except BaseException as exc:
                    row['error'] = repr(exc)
                    raise
                finally:row['finished_monotonic'] = time.monotonic()
            return call

    def protections(label):
        client = ProtectionClient(lambda row: report['protection_wire'].append(dict(label=label, **row)))
        rows = {}
        for name in ('Get_Collision_Stage', 'Get_Self_Collision_Enable'):
            try:rows[name] = getattr(client, name)()
            except Exception as exc:rows[name] = dict(error=repr(exc))
        return rows

    try:
        save()
        report['sdk_provenance'] = source_audit(sdk_dir)
        report['source_sha256'] = {name: hashlib.sha256((ROOT/'deployment'/name).read_bytes()).hexdigest()
            for name in ('new_grasp_reference_worker_v1.py', 'inspect_left_arm.py',
                'arm_protection_transport.py', 'arm_sdk_reviewed_hashes.json')}
        # Separate controller connections: SDK must not be active during these GETs.
        report['protection_before'] = protections('before_sdk_connection')
        save()
        package = types.ModuleType('battery_new_grasp_reference_readonly_sdk')
        package.__path__ = [str(sdk_dir)]
        sys.modules[package.__name__] = package
        sdk = importlib.import_module(package.__name__+'.robotic_arm')
        # Retain the object for finally cleanup even if __init__ fails midway.
        robot = sdk.Arm.__new__(sdk.Arm)
        sdk.Arm.__init__(robot, sdk.RM65, '169.254.128.18')
        report['sdk_socket_state_raw'] = encode(robot.Arm_Socket_State())
        if type(report['sdk_socket_state_raw']) is not int or report['sdk_socket_state_raw'] != 0:
            raise RuntimeError('Current left-arm SDK socket is not ready')
        report['raw_before'] = snapshot(LoggedReads())
        report['queries']['before'] = report['raw_before']
        save()
        report['raw_after'] = snapshot(LoggedReads())
        report['queries']['after'] = report['raw_after']
        save()
        before, after = arm_from_queries(report['raw_before']), arm_from_queries(report['raw_after'])
        delta = bracket_delta(before, after)
        report['bracket_delta'] = delta
        if (not delta['frame_signatures_unchanged'] or not delta['limits_unchanged']
                or max(abs(v) for v in delta['joint_degrees']) > .08
                or delta['position_distance_m'] > .0005 or delta['orientation_angle_rad'] > .005):
            raise RuntimeError('Current arm/frame/limits changed during reference bracket')
        report.update(reference_arm=after, reference_qualified=True,
            outcome='current_new_pregrasp_arm_reference_recorded',
            reference_source='two_healthy_current_SDK_GET_brackets_not_fault_or_historical_pose')
    except BaseException as exc:
        report.update(reference_arm=None, reference_qualified=False, outcome='reference_not_qualified')
        report['exceptions'].append(dict(error=repr(exc), traceback=traceback.format_exc()))
    finally:
        if robot is not None:
            for name in ('Arm_Socket_Close', 'RM_API_UnInit'):
                try:
                    raw = encode(getattr(robot, name)())
                    report.setdefault('sdk_cleanup_results', []).append(dict(method=name, raw=raw))
                    if name == 'RM_API_UnInit' and (type(raw) is not int or raw != 0):
                        report['cleanup_errors'].append(dict(method=name, error='Nonzero or unknown cleanup result', raw=raw))
                except BaseException as exc:
                    report['cleanup_errors'].append(dict(method=name, error=repr(exc)))
            report['SDK_cleanup_finished'] = not report['cleanup_errors']
        if report['cleanup_errors']:
            report.update(reference_arm=None, reference_qualified=False, outcome='reference_cleanup_failed')
        if 'protection_before' in report:
            if robot is None or report['SDK_cleanup_finished']:
                report['protection_after'] = protections('after_sdk_cleanup')
            else:
                report['protection_after'] = dict(skipped='SDK cleanup incomplete; no second controller connection')
            valid = True
            for label in ('protection_before', 'protection_after'):
                collision = report[label].get('Get_Collision_Stage')
                own = report[label].get('Get_Self_Collision_Enable')
                if (not isinstance(collision, list) or len(collision) != 2
                        or type(collision[0]) is not int or collision[0] != 0
                        or type(collision[1]) is not int or collision[1] not in range(9)
                        or not isinstance(own, list) or len(own) != 2
                        or type(own[0]) is not int or own[0] != 0 or type(own[1]) is not bool):
                    valid = False
                    report['warnings'].append(label + ': protection GET incomplete/malformed; no motion readiness')
            same = report['protection_before'] == report['protection_after']
            if valid and not same:
                report['warnings'].append('Protection configuration changed; arm reference alone grants no motion')
            report['protection_ready_for_later_motion'] = (valid and same and
                report['protection_after'] == {'Get_Collision_Stage': [0, 8],
                    'Get_Self_Collision_Enable': [0, True]})
            if valid and same and not report['protection_ready_for_later_motion']:
                report['warnings'].append('Protection is not stage8/selfTrue; arm reference alone grants no motion')
        report['closed_timestamp_utc'] = datetime.now(timezone.utc).isoformat()
        save()
    print('SAVED:', output, 'REFERENCE_QUALIFIED:', report['reference_qualified'], flush=True)
    return 0 if report['reference_qualified'] and not report['exceptions'] and report['SDK_cleanup_finished'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
