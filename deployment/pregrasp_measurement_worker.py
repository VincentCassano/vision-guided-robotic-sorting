#!/usr/bin/env python3
"""Isolated stationary measurement reader. Every actuator/configuration write is denied."""
import argparse
import contextlib
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import socket
import time
import traceback

from capture_installed_hand_geometry import _arm, _json_safe, _vector, _words
from inspect_left_arm import QUERIES, encode, snapshot
from inspect_left_hand import connect
from run_school_battery_hand import Hand
from school_battery_transport import MULTI, SINGLE, Transport

ROOT = Path(__file__).resolve().parents[1]
NATIVE_SHA = '052f94a92d5c7320f227ea811916375fa0d352250329570bd50e0c7abe603ad5'
NATIVE_BYTES = 690912
SDK_DEFAULT = Path('/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package')


def _deny(*args, **kwargs):
    raise RuntimeError('Measurement reader has no actuator or configuration capability')


class ReadOnlyArm:
    """Only the reviewed seven zero-argument queries and lifecycle cleanup exist."""
    __slots__ = ('__robot', '__log')

    def __init__(self, robot, log):
        object.__setattr__(self, '_ReadOnlyArm__robot', robot)
        object.__setattr__(self, '_ReadOnlyArm__log', log)

    def __getattribute__(self, name):
        if name not in QUERIES + ('Arm_Socket_Close', 'RM_API_UnInit'):
            raise AttributeError('Read-only arm method denied: ' + name)
        robot = object.__getattribute__(self, '_ReadOnlyArm__robot')
        log = object.__getattribute__(self, '_ReadOnlyArm__log')

        def call(*args, **kwargs):
            if args or kwargs:
                raise RuntimeError('Read-only SDK query takes no retry/options arguments')
            log(dict(event='sdk_call_before', method=name, monotonic=time.monotonic()))
            try:
                value = getattr(robot, name)()
                log(dict(event='sdk_call_after', method=name, value=_json_safe(encode(value)),
                         monotonic=time.monotonic()))
                return value
            except BaseException as exc:
                log(dict(event='sdk_call_error', method=name, error=repr(exc),
                         monotonic=time.monotonic()))
                raise
        return call


class ReadOnlyTransport(Transport):
    """Reject writes at the final send boundary, even if a caller flips a flag."""
    def enable_motion(self):
        _deny()

    def write_group(self, *args, **kwargs):
        _deny()

    def exchange(self, message):
        if not isinstance(message, dict) or message.get('command') not in (MULTI, SINGLE):
            _deny()
        return super().exchange(message)


class ReadOnlyHandIO:
    """Expose only register readers; Hand cannot reach the raw socket/transport."""
    __slots__ = ('__transport',)

    def __init__(self, transport):
        object.__setattr__(self, '_ReadOnlyHandIO__transport', transport)

    def __getattribute__(self, name):
        transport = object.__getattribute__(self, '_ReadOnlyHandIO__transport')
        if name in ('read', 'single', 'disable_motion'):
            return getattr(transport, name)
        if name == 'broken':
            return transport.broken
        if name in ('enable_motion', 'write_group', 'exchange'):
            return _deny
        raise AttributeError('Read-only hand capability denied: ' + name)


class MeasurementHand(Hand):
    def snapshot(self):
        # Complete one readout before interpreting status: this reader has no
        # motion capability to revoke. Protection evidence is retained even if
        # the historic movement validator would stop after the first block.
        status = self.read('status', 1085, 6)
        current = self.read('current_ma_0_to_4', 1105, 5)
        rotation = self.io.single(1110)
        self.event('read', name='rotation_current_ma', address=1110, values=[rotation])
        positions = self.read('positions', 1145, 6)
        row = dict(status=status, current=current + [rotation], positions=positions)
        self.report['last_snapshot'] = row
        self.save()
        return row


def analyze_static(before_queries, after_queries, first_hand, last_hand, profile, configuration):
    """Validate only the observed static bracket; no path/motion can be approved."""
    result = dict(static_readout_valid=False, geometry_reference_eligible=False,
                  execution_ready=False, hand_pose_classification='unknown',
                  errors=[], observations=[], controller_configuration_fully_verified=False,
                  arm_native_wire_captured=False)
    try:
        if (not isinstance(profile, dict) or type(profile.get('schema_version')) is not int
                or profile['schema_version'] != 1):
            raise ValueError('Missing reviewed hand profile')
        identity = _words(profile.get('device_identity'), 8, 'profile identity')
        poses = {name: _words(values, 6, 'profile pose')
                 for name, values in profile.get('poses', {}).items()}
        before, after = _arm(before_queries), _arm(after_queries)
        result['current_arm'] = dict(tcp_position_mm=[v * 1000 for v in after['pose'][:3]],
            tcp_rpy_rad=after['pose'][3:], joint_degrees=after['joints'],
            joint_reserve_degrees=after['reserve'], frame_signatures=after['frame_signatures'],
            limits=after['limits'], arm_error=after['arm_error'], system_error=after['system_error'])
        if after['frame_names'] != {'work': 'World', 'tool': 'Arm_Tip'}:
            result['errors'].append('Unexpected work/tool frame names')
        if before['frame_signatures'] != after['frame_signatures']:
            result['errors'].append('Work/tool frame configuration changed during bracket')
        if before['limits'] != after['limits']:
            result['errors'].append('Joint configuration changed during bracket')
        if any(v < 3 for v in before['reserve'] + after['reserve']):
            result['errors'].append('Observed joints have less than 3 degrees reserve')
        position_drift = math.sqrt(sum((a-b)**2 for a,b in zip(before['pose'][:3], after['pose'][:3]))) * 1000
        orientation_drift = [abs(math.atan2(math.sin(a-b), math.cos(a-b)))
                             for a,b in zip(before['pose'][3:], after['pose'][3:])]
        joint_drift = [abs(a-b) for a,b in zip(before['joints'], after['joints'])]
        result['bracket_drift'] = dict(tcp_position_mm=position_drift,
            tcp_orientation_rad=orientation_drift, joint_degrees=joint_drift)
        if position_drift > .5 or any(v > .005 for v in orientation_drift) or any(v > .2 for v in joint_drift):
            result['errors'].append('Arm moved during read-only bracket')
        live_limits = _vector(configuration.get('current_limit_ma'), 6, 'live current limit')
        if any(v <= 0 for v in live_limits):
            raise ValueError('Invalid live current limits')
        result['hand_configuration_matches_historical_profile'] = configuration == profile.get('observed_configuration')
        if not result['hand_configuration_matches_historical_profile']:
            result['observations'].append('Hand configuration differs from historical profile; recorded without changing it')
        if not isinstance(first_hand, dict) or not isinstance(last_hand, dict):
            raise ValueError('Missing complete hand preflight bracket')
        validated = []
        for label, hand in (('before', first_hand), ('after', last_hand)):
            if _words(hand.get('identity'), 8, label + ' identity') != identity:
                result['errors'].append('Unexpected hand identity in ' + label)
            status = _words(hand.get('status'), 6, label + ' status')
            force = _words(hand.get('force_targets'), 5, label + ' force targets')
            targets = _words(hand.get('targets'), 6, label + ' targets')
            positions = _words(hand.get('positions'), 6, label + ' positions')
            currents = _vector(hand.get('current'), 6, label + ' current')
            speeds = _words(hand.get('speeds'), 6, label + ' speeds')
            if status != [2] * 6:
                result['errors'].append('Hand status not uniformly reached; raw status is retained')
            if any(force):
                result['errors'].append('Nonzero hand force targets; existing hold may be active')
            if any(v <= 0 for v in speeds):
                result['errors'].append('Invalid current hand speed setting')
            if any(v < 0 or v > limit for v,limit in zip(currents, live_limits)):
                result['errors'].append('Current outside live controller current limits')
            if any(abs(a-b) > 150 for a,b in zip(targets,positions)):
                result['errors'].append('Target/position feedback mismatch')
            validated.append((targets, positions, speeds, force))
        if any(abs(a-b) > 2 for a,b in zip(validated[0][0], validated[1][0])):
            result['errors'].append('Hand targets changed during bracket')
        if any(abs(a-b) > 150 for a,b in zip(validated[0][1], validated[1][1])):
            result['errors'].append('Hand positions changed during bracket')
        if validated[0][2:] != validated[1][2:]:
            result['errors'].append('Hand speed/force settings changed during bracket')
        matches = [name for name,pose in poses.items()
                   if all(abs(a-b) <= 2 for targets,positions,_,_ in validated for a,b in zip(targets,pose))
                   and all(abs(a-b) <= 150 for targets,positions,_,_ in validated for a,b in zip(positions,pose))]
        if len(matches) == 1:
            result['hand_pose_classification'] = matches[0]
        else:
            result['observations'].append('Current hand shape is unclassified; no shape change was requested')
        result['static_readout_valid'] = not result['errors']
        result['geometry_reference_eligible'] = (result['static_readout_valid'] and len(matches) == 1
            and result['hand_configuration_matches_historical_profile'])
    except (ValueError, TypeError, KeyError, OverflowError, AttributeError) as exc:
        result['errors'].append(str(exc))
    return result


def source_audit(sdk_dir):
    """Audit before loading any native code or opening a device connection."""
    if platform.system() != 'Linux' or platform.machine() != 'aarch64':
        raise RuntimeError('Read-only SDK worker requires the reviewed Linux aarch64 host')
    hashes_path = Path(__file__).with_name('arm_sdk_reviewed_hashes.json')
    expected = json.loads(hashes_path.read_text(encoding='utf-8'))
    actual = {name: hashlib.sha256((sdk_dir/name).read_bytes()).hexdigest() for name in expected}
    if actual != expected:
        raise RuntimeError('SDK wrapper/log source changed; no SDK load or connection')
    native = sdk_dir / 'libRM_Base.so'
    raw = native.read_bytes()
    if (len(raw) != NATIVE_BYTES or hashlib.sha256(raw).hexdigest() != NATIVE_SHA
            or raw[:6] != b'\x7fELF\x02\x01' or int.from_bytes(raw[18:20], 'little') != 183):
        raise RuntimeError('Native SDK differs from reviewed aarch64 API 4.3.1; no connection')
    return dict(host_os=platform.system(), host_machine=platform.machine(),
                sdk_source_sha256=actual, loaded_native=dict(path=str(native), sha256=NATIVE_SHA,
                byte_count=NATIVE_BYTES, architecture='aarch64'),
                connection_constructor_review='API init/version/socket start/state only; no actuator or configuration method',
                live_controller_identity_firmware_dh_read=False)


def descriptor_state():
    rows = []
    for fd in (0,1,2):
        try:
            row = os.fstat(fd)
            rows.append(dict(fd=fd, open=True, device=row.st_dev, inode=row.st_ino, mode=row.st_mode))
        except OSError as exc:
            rows.append(dict(fd=fd, open=False, error=repr(exc)))
    return rows


def capture(args, folder, report, save):
    """One SDK session; reads before/hand/after. The parent owns the timeout."""
    import fcntl
    sdk_robot = arm = hand_transport = None
    profile_path = Path(__file__).with_name('battery_hand_verified_profile.json')
    profile_bytes = profile_path.read_bytes()
    profile = json.loads(profile_bytes.decode('utf-8'))
    report['profile_sha256'] = hashlib.sha256(profile_bytes).hexdigest()
    report['sdk_environment'] = source_audit(args.sdk_dir)
    report['source_sha256'] = {name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
        for name in ('pregrasp_measurement_worker.py', 'inspect_left_arm.py', 'inspect_left_hand.py',
                     'capture_installed_hand_geometry.py', 'school_battery_transport.py', 'run_school_battery_hand.py')}
    save()
    lock_path = ROOT/'captures'/'hand_stage.lock'
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open('a') as lock, \
         (folder/'wire.jsonl').open('w', encoding='utf-8') as wire, \
         (folder/'samples.jsonl').open('w', encoding='utf-8') as samples, \
         (folder/'sdk_calls.jsonl').open('w', encoding='utf-8') as sdk_calls:
        def append(stream, row):
            stream.write(json.dumps(_json_safe(row), allow_nan=False)+'\n'); stream.flush()
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        report['exclusive_lock_acquired'] = True
        save()
        try:
            report['sdk_constructor_attempted'] = True
            save()
            sdk_robot = connect(args.sdk_dir)
            report['sdk_session_opened'] = True
            arm = ReadOnlyArm(sdk_robot, lambda row: append(sdk_calls, row))
            report['arm_before'] = snapshot(arm)
            save()
            # Invalid arm data prevents a second device connection; it never
            # triggers a motion/reconfiguration attempt or SDK retry.
            _arm(report['arm_before'])
            hand_transport = ReadOnlyTransport(socket.create_connection(('169.254.128.18',8080), timeout=3),
                                               lambda row: append(wire,row))
            hand = MeasurementHand(ReadOnlyHandIO(hand_transport), report, save, samples, confirm=_deny)
            try:
                hand.configuration()
            except RuntimeError as exc:
                # Baseline mismatch is evidence, not a reason to modify this
                # device. Read remaining current telemetry exactly once.
                report['configuration_validation_error'] = repr(exc)
                save()
                if hand_transport.broken or not report.get('configuration', {}).get('current_limit_ma'):
                    raise
            try:
                report['hand_preflight_before'] = hand.preflight()
                report['hand_angle_raw'] = hand.read('angle_raw', 1165, 6)
                report['hand_force_feedback_raw'] = hand.read('force_feedback_raw', 1175, 5)
                report['hand_preflight_after'] = hand.preflight()
            except Exception:
                if report.get('last_preflight'):
                    report['incomplete_hand_preflight'] = report['last_preflight']
                raise
            finally:
                save()
        except (Exception, KeyboardInterrupt) as exc:
            report['exceptions'].append(dict(phase='capture', error=repr(exc), traceback=traceback.format_exc()))
            save()
        finally:
            if hand_transport is not None:
                try:
                    hand_transport.disable_motion()
                    hand_transport.sock.close()
                except Exception as exc:
                    report['cleanup_errors'].append(dict(method='hand_socket.close',error=repr(exc)))
                save()
            if arm is not None:
                report['arm_after'] = snapshot(arm)
                save()
                probes = report.setdefault('sdk_cleanup_descriptor_probes', [])
                probes.append(dict(stage='before_cleanup', descriptors=descriptor_state()))
                save()
                for method in ('Arm_Socket_Close', 'RM_API_UnInit'):
                    try:
                        getattr(arm,method)()
                    except Exception as exc:
                        report['cleanup_errors'].append(dict(method=method,error=repr(exc)))
                    probes.append(dict(stage='after_'+method, descriptors=descriptor_state()))
                    save()
        report['analysis'] = analyze_static(report.get('arm_before'), report.get('arm_after'),
            report.get('hand_preflight_before'), report.get('hand_preflight_after'), profile,
            report.get('configuration', {}))
        report['static_readout_valid'] = (report['analysis']['static_readout_valid']
            and not report['exceptions'] and not report['cleanup_errors'])
        report['geometry_reference_eligible'] = (report['static_readout_valid']
            and report['analysis']['geometry_reference_eligible'])
        report['outcome'] = ('current_static_readout_recorded' if report['static_readout_valid']
                             else 'current_static_readout_incomplete_or_unusable')
        save()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--sdk-dir', type=Path, default=SDK_DEFAULT)
    parser.add_argument('--role', choices=('current_static',), default='current_static')
    args = parser.parse_args(argv)
    folder = args.output.resolve()
    folder.mkdir(parents=True, exist_ok=True)
    if (folder/'result.json').exists():
        parser.error('Output already contains a worker result; do not overwrite evidence')
    report = dict(schema_version=1, role=args.role, timestamp_utc=datetime.now(timezone.utc).isoformat(),
        read_only=True, hardware_writes=0, arm_actuator_writes=0, hand_register_writes=0,
        execution_ready=False, static_readout_valid=False, geometry_reference_eligible=False,
        exclusive_lock_acquired=False, sdk_constructor_attempted=False, sdk_session_opened=False,
        outcome='starting', exceptions=[], cleanup_errors=[], query_methods=list(QUERIES),
        sdk_directory=str(args.sdk_dir), ip='169.254.128.18', parent_owns_timeout_seconds=120,
        automatically_return_to_historical_anchor=False, current_state_source='fresh_stationary_readout',
        arm_raw_protocol_scope='Encoded SDK query return values; native TCP byte frames unavailable from this wrapper',
        note='Historical hand shape is only classified. Unknown shape does not request motion. '
             'Recorded static feedback never authorizes descent, hold, release or return.')

    def save():
        temp = folder/'result.tmp'
        temp.write_text(json.dumps(_json_safe(report), ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
        temp.replace(folder/'result.json')

    save()
    try:
        # Python SDK console output is saved here. The parent must also retain
        # OS-level stdout/stderr for native output and abnormal process exits.
        with (folder/'sdk_stdout.log').open('w', encoding='utf-8') as out, \
             (folder/'sdk_stderr.log').open('w', encoding='utf-8') as err, \
             contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            capture(args, folder, report, save)
    except (Exception, KeyboardInterrupt) as exc:
        report['exceptions'].append(dict(phase='worker', error=repr(exc), traceback=traceback.format_exc()))
        report['outcome'] = 'current_static_worker_failed_no_actuator_writes'
        report['static_readout_valid'] = False
        report['geometry_reference_eligible'] = False
    finally:
        report['execution_ready'] = False
        save()
    return 0 if report['static_readout_valid'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
