#!/usr/bin/env python3
"""Isolated controller session factory; no accepted physical route in v1."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import platform
import traceback

from pregrasp_controller_adapter import ControllerAdapter, require_physical_route


def run_session(plan, sdk_dir, folder, report, save):
    # These imports and the first connection are strictly AFTER physical-route
    # validation in main. The parent never imports the native SDK.
    import fcntl
    import socket
    from run_localized_pregrasp import load_provenance, close_capture_resources
    from run_arm_hand_sequence import Sequence
    from continuous_hand_cycle import ContinuousHand
    from supported_contact_transport import SupportedContactTransport, PhaseAuthority
    from arm_protection_transport import ProtectionClient
    root = Path(__file__).resolve().parents[1]
    prior, _, profile, _, _, _ = load_provenance()
    first = plan['route_start']
    current = plan['current_pose']
    reference = dict(joints=first['joint_degrees'],
        pose=[v/1000 for v in first['tool_world_mm']]+first['tool_rpy_rad'],
        limits=current['limits'], frame_signatures=current['frame_signatures'],
        reserve=[min(q-a,b-q) for q,a,b in zip(first['joint_degrees'],
            current['limits']['minimum'], current['limits']['maximum'])])
    authority = PhaseAuthority()
    robot = sequence = backend = None

    class PrepareOnly(SupportedContactTransport):
        def set_phase(self, phase, token):
            if phase != 'prepare_empty':
                self.latch_fault('Pregrasp only; no grip, release or configuration phase')
                raise RuntimeError('Only PREPARE is permitted')
            return super().set_phase(phase, token)

    class SessionSequence(Sequence):
        def close_hand(self):
            if self.io is not None:
                io, self.io = self.io, None
                self.hand_motion_enabled = False
                io.disarm_motion()  # Clean close must not latch the shared token.
                try:
                    io.sock.close()
                except BaseException:
                    io.disable_motion()
                    raise

    def append(out, row):
        out.write(json.dumps(row)+'\n')
        out.flush()

    with (root/'captures/hand_stage.lock').open('a') as lock, \
         (folder/'hand_wire.jsonl').open('w', encoding='utf-8') as wire, \
         (folder/'arm_protection_wire.jsonl').open('w', encoding='utf-8') as protection_wire, \
         (folder/'samples.jsonl').open('w', encoding='utf-8') as samples:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        def factory(seq, guarded):
            if not guarded or seq.io is not None:
                raise RuntimeError('Single guarded session required')
            seq.io = PrepareOnly(socket.create_connection(('169.254.128.18',8080), timeout=3),
                                 lambda row:append(wire,row), authority)
            return ContinuousHand(seq, seq.io, report, save, samples)
        try:
            report['device_connection_attempted'] = True
            save()
            robot = prior.connect(sdk_dir)
            report['device_connection_opened'] = True
            save()
            sequence = SessionSequence(robot, reference, profile, report, save, samples, factory,
                protection_client=ProtectionClient(lambda row:append(protection_wire,row)))
            sequence.hand_motion_enabled = False
            def permit():
                sequence.io.set_phase('prepare_empty', authority)
                sequence.io.enable_motion()
                sequence.hand_motion_enabled = True
            backend = ControllerAdapter(sequence, permit)
            backend.fresh_preflight(plan)
            backend.prepare()
            report['prepare_feedback_observed'] = True
            save()
            for command in plan['command_specification']:
                observed = backend.command(command)
                report.setdefault('segments', []).append(dict(command=command, feedback=observed))
                save()
            report['outcome'] = 'pregrasp_feedback_observed_no_grip_or_lift'
        except BaseException:
            if backend is not None:
                backend.fault()
            raise
        finally:
            close_capture_resources(sequence, robot, report)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--packet', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--sdk-dir', type=Path, default=Path('/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package'))
    args = p.parse_args(argv)
    args.output.mkdir(parents=True, exist_ok=False)
    report = dict(schema_version=1, timestamp_utc=datetime.now(timezone.utc).isoformat(),
        outcome='starting', exceptions=[], cleanup_errors=[], device_connection_opened=False,
        device_connection_attempted=False,
        physical_stop_verified=False, torque_release_verified=False, automatic_return=False,
        automatic_release=False, automatic_grip=False, automatic_lift=False)
    def save():
        tmp = args.output/'result.tmp'
        tmp.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        tmp.replace(args.output/'result.json')
    try:
        plan = require_physical_route(args.packet.read_bytes())
        if platform.system() != 'Linux' or platform.machine() != 'aarch64':
            raise RuntimeError('Reviewed onsite Linux aarch64 required')
        run_session(plan, args.sdk_dir, args.output, report, save)
    except BaseException as exc:
        report['exceptions'].append(dict(error=repr(exc), traceback=traceback.format_exc()))
        report['outcome'] = 'refused_before_device_connection' if not report['device_connection_attempted'] else 'aborted_no_further_writes'
        print('ABORT:', exc)
    finally:
        save()
    return 0 if report['outcome'] == 'pregrasp_feedback_observed_no_grip_or_lift' else 2


if __name__ == '__main__':
    raise SystemExit(main())
