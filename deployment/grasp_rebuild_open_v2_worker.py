#!/usr/bin/env python3
"""Isolated empty-hand worker: four flex channels to8000 once, no arm motion."""
import argparse
import copy
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/battery_grasp/src'))
from battery_grasp.taught_dh_model import EXPECTED_DH
from high_descent_prepare_worker import HighSequence, same_pose
from vision_cycle_hand import CycleHand, PREPARE
from vision_cycle_transport import CycleTransport
from supported_contact_transport import WritePhase
from school_battery_transport import WRITE
from run_school_battery_hand import TOL, TARGET_TOL

SUCCESS = 'completed_empty_hand_position_release_to_PREPARE'
OPENING_SUCCESS = 'completed_empty_hand_four_channel_prepare_feedback_recorded'
WAIT_PHASE = 'await_supported_position_release'
SAFETY_CONFIGURATION = ('Get_Joint_EN_State', 'Get_Joint_Err_Flag',
    'Get_Collision_Stage', 'Get_Self_Collision_Enable', 'Get_Joint_Drive_Min_Pos',
    'Get_Joint_Drive_Max_Pos', 'Get_Arm_Software_Version', 'Get_Install_Pose')


def loaded_bounds(hand, row, expected_targets=None):
    """Current empty hand only: one decreasing four-flex target; no old recipe."""
    if not hand.sequence.allow_protected_opening:
        raise RuntimeError('Explicit empty-hand four-channel opening required')
    targets=row['targets'];positions=row['positions']
    if any(v<8000 for v in targets[:4]+positions[:4]):
        raise RuntimeError('Target8000 would add closure to a current flex position/target; no write')
    if expected_targets is not None:
        if (any(abs(a-b)>TARGET_TOL for a,b in zip(targets[:4],expected_targets[:4]))
                or targets[4:]!=expected_targets[4:]):
            raise RuntimeError('Current targets changed during confirmation; no write')
    anchor=getattr(hand.sequence,'empty_open_current_anchor',None)
    if anchor is None:
        hand.sequence.empty_open_current_anchor=copy.deepcopy(row)
        hand.report['empty_open_current_anchor']=copy.deepcopy(row)
        hand.report['reference_recipe_required']=False
        hand.report['untouched_channels_reference']='fresh current targets/positions, not65535 or prior opposition'
        hand.save()
    elif (any(abs(a-b)>TARGET_TOL for a,b in zip(targets[:4],anchor['targets'][:4]))
            or targets[4:]!=anchor['targets'][4:]
            or any(abs(a-b)>TOL for a,b in zip(positions,anchor['positions']))):
        raise RuntimeError('Current hand changed after initial read; restart without writes')


class ReleaseOnlyHand(CycleHand):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.opening_attempted = False
        self.write_status3_channels = set()
        self.still_existing_status3_channels = set()
        self.write_anchor = None

    def check_status(self, row):
        if not self.sequence.allow_protected_opening:
            self._protection('Explicit empty-only opening mode required')
        status = row['status']
        if any(value == 4 for value in status):
            self._protection('Force-control status4 is never allowed in opening-only recovery')
        channels = {channel for channel, value in enumerate(status) if value == 3}
        if self.opening_attempted:
            if not channels <= self.still_existing_status3_channels:
                self._protection('New current-protection status3 appeared after opening-only target')
            self.still_existing_status3_channels.intersection_update(channels)
            self.report.setdefault('opening_status_observations', []).append(dict(
                phase='after_single_opening_attempt', status=status[:],
                existing_status3_channels=sorted(channels), protection_cleared_claimed=False))
        else:
            if any(channel >= 4 for channel in channels):
                self._protection('Protection on held little/rotation channel blocks opening-only recovery')
            loaded_bounds(self, row)
            self.report.setdefault('opening_status_observations', []).append(dict(
                phase='readonly_before_opening', status=status[:],
                existing_status3_channels=sorted(channels), opening_direction_checked=True,
                actuator_write_authorized_by_this_read=False, protection_cleared_claimed=False))
        self.save()

    def observe_opening(self, anchor):
        began = self.clock()
        previous = None
        quiet_since = None
        groups = 0
        minimum_observed = anchor['positions'][:4]
        while self.clock() - began < 30:
            row = self.snapshot()
            if self.clock() - began >= 30:
                break
            if any(abs(value - 8000) > TARGET_TOL for value in row['targets'][:4]):
                raise RuntimeError('Opening target changed outside this single attempt')
            if row['targets'][4:] != anchor['targets'][4:]:
                raise RuntimeError('Unwritten held-channel target changed after opening command')
            if any(abs(a - b) > TOL for a, b in zip(row['positions'][4:], anchor['positions'][4:])):
                raise RuntimeError('Unwritten little/rotation position moved outside observed hold tolerance')
            if any(not 8000 - TOL <= position <= initial + TOL
                    for position, initial in zip(row['positions'][:4], anchor['positions'][:4])):
                raise RuntimeError('Opening feedback left actual-start to8000 withdrawal bounds')
            if any(position > minimum + TOL
                    for position, minimum in zip(row['positions'][:4], minimum_observed)):
                raise RuntimeError('Finger reclosed beyond opening observation tolerance; no next target')
            minimum_observed = [min(position, minimum)
                for position, minimum in zip(row['positions'][:4], minimum_observed)]
            reached = all(abs(position - 8000) <= TOL for position in row['positions'][:4])
            quiet = (previous is not None and all(abs(a - b) <= TOL
                for a, b in zip(row['positions'], previous['positions'])))
            if reached and quiet:
                if quiet_since is None:
                    quiet_since = self.clock()
                groups += 1
                if groups >= 2 and self.clock() - quiet_since >= 1:
                    self._critical_configuration()
                    if self.clock() - began >= 30:
                        break
                    self.report['opening_feedback'] = dict(final=row,
                        commanded_channels=[0, 1, 2, 3], untouched_channels=[4, 5],
                        protection_cleared_claimed=False, torque_release_verified=False,
                        initial_status3_channels=sorted(self.write_status3_channels),
                        final_status3_channels=[ch for ch, value in enumerate(row['status']) if value == 3])
                    self.report['opening_retreat_confirmed'] = True
                    self.save()
                    return row
            else:
                groups = 0
                quiet_since = None
            previous = row
        self.report.update(opening_retreat_confirmed=False,
            opening_unconfirmed_reason='Single target did not reach observed8000 withdrawal within30seconds',
            protection_cleared_claimed=False)
        self.save()
        self._disable()
        raise TimeoutError('Opening-only retreat unconfirmed; no retry/reset/additional target')

    def release_prepare(self, expected):
        self._critical_configuration()
        row = self.snapshot()
        loaded_bounds(self, row, expected)
        self.guard()
        if not self.sequence.hand_motion_enabled or not self.io.motion_enabled:
            raise RuntimeError('Supported-position release not authorized')
        if self.sequence.allow_protected_opening:
            self.write_status3_channels = {ch for ch, value in enumerate(row['status']) if value == 3}
            self.still_existing_status3_channels = self.write_status3_channels.copy()
            self.write_anchor = copy.deepcopy(row)
            self.report['opening_write_before'] = dict(snapshot=copy.deepcopy(row),
                status3_channels=sorted(self.write_status3_channels),
                address=1135, count=4, values=[8000] * 4,
                untouched_channels=[4, 5], protection_cleared_claimed=False)
            self.save()
            self.opening_attempted = True
            self.io.write_group(1135, [8000] * 4)
            return self.observe_opening(row)
        raise RuntimeError('No six-channel PREPARE capability in this empty opening entry')


class ReleaseOnlyTransport(CycleTransport):
    """Preserve strict framing/read fences; add no previous-close fiction."""
    def set_phase(self, phase, authority):
        if (authority is not self._phase_authority or phase != 'release_loaded_position'
                or self.terminal_latched or self.broken or self._completed
                or self._phase is not WritePhase.READONLY
                or getattr(authority, 'loaded_release_started', False)
                or authority._attempted_writes):
            self.latch_fault('Only one explicit loaded-position release phase is permitted')
            raise RuntimeError('Loaded-position release phase refused')
        authority.loaded_release_started = True
        self._phase = WritePhase.RELEASE
        self._phase_index = 0
        self.log(dict(event='loaded_position_release_phase', phase=phase,
            no_prior_close_claimed=True, force_writes=False, arm_motion=False))

    def _validate(self, message):
        if message.get('command') != WRITE:
            return super()._validate(message)
        protected = getattr(self._phase_authority, 'allow_protected_opening', False)
        if not protected:raise RuntimeError('Only explicit four-channel empty opening is permitted')
        values = (8000, 8000, 8000, 8000)
        exact = dict(command=WRITE, port=1, address=1135, num=len(values),
            data=[byte for value in values for byte in (value >> 8, value & 255)], device=2)
        if (message != exact or any(type(message.get(name)) is not int
                for name in ('port', 'address', 'num', 'device'))
                or not isinstance(message.get('data'), list)
                or any(type(byte) is not int for byte in message['data'])
                or self._phase is not WritePhase.RELEASE or not self.motion_enabled
                or self.terminal_latched or self._completed or self._phase_index != 0
                or not getattr(self._phase_authority, 'loaded_release_started', False)):
            raise ValueError('Only exact single PREPARE supported-release write allowed')
        key = ('release_loaded_position', 0, (1135, values))
        if key in self._attempted_writes:
            raise RuntimeError('Supported-release target already attempted; no retry')
        return key


class ReleaseOnlySequence(HighSequence):
    def __init__(self, *args, allow_protected_opening=False, **kwargs):
        super().__init__(*args, **kwargs)
        self.allow_protected_opening = allow_protected_opening

    def fault(self):
        """This entry never issues arm stop/movement or automatic hand recovery."""
        self.hand_motion_enabled = False
        self.authority._terminal_latched = True
        if self.io is not None:
            self.io.disable_motion()
        self.report.update(execution_ready=False, phase='blocked',
            automatic_open=False, automatic_return=False, automatic_force_zero=False,
            onsite_hardware_stop_required=True)
        self.save()

    def model_read(self):
        from taught_model_readonly import collect_model_parameters
        name = 'fresh_model_before_release' if self.reference is not None else 'fresh_model'
        model = collect_model_parameters(self.folder / name)
        self.report[name] = model
        self.save()
        if (not model['dh_valid'] or not model['identity_valid'] or model['errors']
                or [model['dh_response']['joint_' + str(index)] for index in range(1, 7)] != EXPECTED_DH):
            raise RuntimeError('Fresh device identity/DH mismatch; no position target')

    def loaded_preflight(self, expected_arm=None, expected_targets=None):
        from sdk_environment import collect_sdk_environment
        from pregrasp_measurement_worker import NATIVE_SHA
        self.model_read()
        metadata = collect_sdk_environment(self.robot)
        self.report['sdk_environment'] = metadata
        self.save()
        if (not metadata['metadata_complete'] or metadata.get('api_version') != '4.3.1'
                or metadata.get('loaded_native', {}).get('sha256') != NATIVE_SHA
                or metadata.get('host_machine') != 'aarch64'
                or metadata.get('controller_run_mode_raw', {}).get('data') != [1]):
            raise RuntimeError('Native identity or actual real-mode mismatch; no position target')
        arm = self.full_arm()
        if expected_arm is None:
            if self.reference is not None:
                raise RuntimeError('Current loaded arm reference can only be bound once')
            self.reference = arm
            self.report.update(startup_reference=arm,
                reference_source='this_session_fresh_loaded_pose', historical_pose_reused=False)
        elif not same_pose(arm, expected_arm) or arm['limits'] != expected_arm['limits']:
            raise RuntimeError('Arm/frame/limits changed during support confirmation; no target')
        self.guard_target, self.guard_pose = arm['joints'][:], arm['pose'][:]
        low, high = self.safety_reads()
        self.drive_limits = dict(minimum=low, maximum=high)
        install = self.report['arm_safety_reads']['Get_Install_Pose']
        if (not isinstance(install, list) or len(install) != 4 or install[0] != 0
                or any(abs(a - b) > .001 for a, b in zip(install[1:], [0, 45, 0]))):
            raise RuntimeError('Actual installation differs from accepted mount; no target')
        if any(min(q - a, b - q) < 3 for q, a, b in zip(arm['joints'], low, high)):
            raise RuntimeError('Current drive reserve below3degrees; no target')
        hand = self.hand_factory(self, True)
        try:
            hand.configuration()
            critical = hand._critical_configuration()
            row = dict(hand.snapshot(), **critical)
            stable_rows = [copy.deepcopy(row)]
            if self.allow_protected_opening:
                second = dict(hand.snapshot(), **critical)
                loaded_bounds(hand, second, row['targets'])
                if any(abs(a - b) > TOL for a, b in zip(row['positions'], second['positions'])):
                    raise RuntimeError('Two complete loaded groups are not observed quiet; no write')
                stable_rows.append(copy.deepcopy(second))
                row = second
                self.report.setdefault('loaded_quiet_group_checks', []).append(dict(
                    phase=self.report['phase'], snapshots=stable_rows,
                    observed_position_span_within150=True, physical_stop_verified=False,
                    protection_cleared_claimed=False))
            self.report['last_loaded_preflight'] = copy.deepcopy(row)
            self.save()
            loaded_bounds(hand, row, expected_targets)
            self.report.update(loaded_partial_position_allowed=True,
                no_target_position_equality_required_for_contact=True, last_loaded_preflight=row)
            self.save()
            return arm, row, hand
        except BaseException:
            self.close_hand()
            raise

    def permission(self):
        self.report.update(phase=WAIT_PHASE, permission_index=1,
            SDK_queries_during_input=False, hand_socket_closed_during_input=self.io is None)
        self.save()
        began = self.clock()
        path = self.folder / 'permissions/1.json'
        while self.clock() - began < 600:
            if (self.folder / 'abort.json').exists():
                raise KeyboardInterrupt('Parent cancelled supported release')
            if path.exists():
                row = json.loads(path.read_text(encoding='utf8'))
                if row.get('index') != 1 or row.get('phase') != WAIT_PHASE:
                    raise RuntimeError('Stale supported-release permission')
                self.report['operator_input'] = row
                self.save()
                if row.get('value') == 'q':
                    raise EOFError('Operator retained loaded pose; no write')
                if row.get('value') != 'r' or row.get('empty_hand_and_opening_clear') is not True:
                    raise RuntimeError('Empty hand/opening clearance not confirmed')
                return
            self.pause(.15)
        raise TimeoutError('Supported-release permission expired; no position target')

    def run(self):
        arm, initial, _ = self.loaded_preflight()
        safety_before = copy.deepcopy(self.report['arm_safety_reads'])
        self.close_hand()
        self.permission()
        self.report['phase'] = 'fresh_before_release'
        self.save()
        _, fresh, hand = self.loaded_preflight(arm, initial['targets'])
        try:
            if any(self.report['arm_safety_reads'][name] != safety_before[name]
                   for name in SAFETY_CONFIGURATION):
                raise RuntimeError('Actual arm safety configuration changed during confirmation; no release')
            if (self.folder / 'abort.json').exists():
                raise KeyboardInterrupt('Parent cancelled before target')
            self.io.set_phase('release_loaded_position', self.authority)
            self.io.enable_motion()
            self.hand_motion_enabled = True
            self.report.update(phase='releasing_position',
                empty_hand_and_opening_confirmed_by_operator=True, execution_ready=True)
            self.save()
            final = hand.release_prepare(fresh['targets'])
            hand.configuration()
            final_arm = self.full_arm()
            if not same_pose(final_arm, arm) or final_arm['limits'] != arm['limits']:
                raise RuntimeError('Arm/frames/limits changed during hand release')
            outcome = OPENING_SUCCESS if self.allow_protected_opening else SUCCESS
            self.report.update(outcome=outcome, phase='completed', final_hand=final,
                final_arm=final_arm, hand_shape='four_channels8000_others_preserved' if self.allow_protected_opening else 'PREPARE', physical_stop_verified=False,
                torque_release_verified=False, execution_ready=False,
                arm_motion_performed=False, object_lift_or_transfer_performed=False,
                functional_validation='onsite_opening_feedback_only',
                protection_cleared_claimed=False)
            self.authority._completed = True
            self.save()
        finally:
            self.close_hand()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--folder', type=Path, required=True)
    parser.add_argument('--sdk-dir', type=Path,
        default=Path('/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package'))
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--allow-protected-opening', action='store_true')
    args = parser.parse_args(argv)
    if not args.execute or not args.allow_protected_opening:
        parser.error('Explicit empty-hand four-channel withdrawal required; no connection')
    folder = args.folder
    report = dict(schema_version=1, scope='empty_hand_four_channel_prepare_only',
        timestamp_utc=datetime.now(timezone.utc).isoformat(), outcome='starting', phase='preflight',
        sdk_in_parent=False, arm_actuator_writes=0, arm_stop_writes=0,
        hand_register_writes=0, controller_configuration_writes=0,
        automatic_open=False, automatic_return=False, automatic_force_zero=False,
        physical_stop_verified=False, torque_release_verified=False,
        execution_ready=False, offline_tests_executed=False, exceptions=[], cleanup_errors=[],
        allow_protected_opening=args.allow_protected_opening,
        protection_cleared_claimed=False)

    def save():
        temp = folder / 'worker_result.tmp'
        temp.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf8')
        temp.replace(folder / 'worker_result.json')

    robot = sequence = None
    save()
    try:
        from pregrasp_measurement_worker import source_audit
        report['source_audit'] = source_audit(args.sdk_dir)
        save()
        import fcntl
        import socket
        from inspect_left_hand import connect
        from supported_contact_transport import PhaseAuthority
        from arm_protection_transport import ProtectionClient
        authority = PhaseAuthority()
        authority.allow_protected_opening = args.allow_protected_opening
        profile = json.loads((ROOT / 'deployment/battery_hand_verified_profile.json').read_bytes())
        with (ROOT / 'captures/hand_stage.lock').open('a') as lock, \
             (folder / 'hand_wire.jsonl').open('w', encoding='utf8') as wire, \
             (folder / 'protection_wire.jsonl').open('w', encoding='utf8') as protection_wire, \
             (folder / 'samples.jsonl').open('w', encoding='utf8') as samples:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)

            def append(out, row):
                out.write(json.dumps(row) + '\n')
                out.flush()

            def hand_log(row):
                append(wire, row)
                if row.get('event') == 'tx' and row.get('request', {}).get('command') == WRITE:
                    report['hand_register_writes'] += 1
                    save()

            def factory(seq, guarded):
                if not guarded or seq.io is not None:
                    raise RuntimeError('Single guarded hand release session required')
                seq.io = ReleaseOnlyTransport(socket.create_connection(('169.254.128.18', 8080), timeout=3),
                    hand_log, authority)
                return ReleaseOnlyHand(seq, seq.io, report, save, samples)

            robot = connect(args.sdk_dir)
            sequence = ReleaseOnlySequence(robot, None, profile, report, save, samples, factory,
                folder=folder, authority=authority, tip_reader=None, enable_arm_protection=False,
                allow_protected_opening=args.allow_protected_opening,
                protection_client=ProtectionClient(lambda row: append(protection_wire, row)))
            try:
                sequence.run()
            except BaseException:
                sequence.fault()
                raise
            finally:
                sequence.close_hand()
    except EOFError as exc:
        report.update(outcome='cancelled_no_position_write', phase='cancelled',
            execution_ready=False, cancellation_reason=str(exc))
    except BaseException as exc:
        report['exceptions'].append(dict(error=repr(exc), traceback=traceback.format_exc()))
        snapshot = report.get('last_snapshot', {})
        if report['hand_register_writes'] == 0 and any(value in (3, 4) for value in snapshot.get('status', [])):
            outcome = 'blocked_active_protection_no_position_write'
            report['block_reason_zh'] = ('空手张开恢复前检未通过；完整状态3/4仍记录，未发送松指目标。'
                if args.allow_protected_opening else '本次新鲜完整读数仍有状态3/4，未发送松指目标。')
        elif report.get('opening_retreat_confirmed') is False and report['hand_register_writes']:
            outcome = 'opening_retreat_unconfirmed_after_single_write'
        else:
            outcome = 'blocked_no_position_write' if report['hand_register_writes'] == 0 else 'aborted_after_single_release_attempt'
        report.update(outcome=outcome, phase='blocked', execution_ready=False)
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
    return 0 if report['outcome'] in (SUCCESS, OPENING_SUCCESS) else 1


if __name__ == '__main__':
    raise SystemExit(main())
