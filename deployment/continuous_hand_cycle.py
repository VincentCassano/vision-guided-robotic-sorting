#!/usr/bin/env python3
"""Continuous, previously accepted EMPTY-hand poses with sequential feedback.

This module changes software observation overhead, never hardware speed or
protection. Three complete in-tolerance feedback groups over at least one
second are an observation criterion, not proof of physical stop or release.
The caller retains onsite hardware-stop supervision and raw transport logging.
"""
import copy

from hand_feedback_policy import FeedbackHand
from run_arm_hand_sequence import ArmGuardHand
from run_school_battery_hand import ADAPTED_PLAN, IDENTITY, TOL, TARGET_TOL


RELEASE_PLAN = (
    dict(phase='release_thumb_flex', address=1135, count=1, values=[0]),
    dict(phase='release_four_fingers', address=1136, count=4, values=[0]*4),
    dict(phase='return_thumb_side_palm', address=1140, count=1, values=[728]),
)
CONFIG_LENGTHS = dict(identity=8, calibration_end_absolute=6,
    calibration_start_absolute=6, thumb_presets_absolute=3,
    pid_p=6, pid_i=6, pid_d=6, pid_g=6, current_limit_ma=6,
    stall_speed=6, stall_current_ma=6, stall_stop_ms=6, stall_retry_ms=6)
ALLOWED_STEPS = {(s['phase'], s['address'], s['count'], tuple(s['values']))
                 for s in list(ADAPTED_PLAN)+list(RELEASE_PLAN)}


class ContinuousHand(FeedbackHand):
    """One authorized cycle, without artificial pauses or repeated target writes."""
    def __init__(self, sequence, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.sequence = sequence
        self.expected_arm = sequence.guard_target[:]
        self.expected_pose = sequence.guard_pose[:]
        self.last_feedback = None
        self._configuration_checked = False
        self._executing = False
        self._fault_latched = False
        self.report.setdefault('moves', [])
        self.report['required_speeds'] = [65535]*6
        self.report['feedback_policy'].update(
            name='continuous_actual_position_three_groups',
            observed_settling_seconds=1.0, minimum_complete_groups=3,
            feedback_read_order=[1000, 1085, 1105, 1145, 1110, 1135],
            no_automatic_fault_open=True, no_speed_writes=True,
            physical_stop=False, continuous_execution_only=True)
        self.report['hand_feedback_policy'] = self.report['feedback_policy']

    def _disable(self):
        self.last_feedback = None
        self._fault_latched = True
        self.sequence.hand_motion_enabled = False
        self.io.disable_motion()

    def guard(self):
        """Use exactly the baseline arm joint/translation/orientation limits."""
        try:
            ArmGuardHand.guard(self)
        except BaseException:
            self._disable()
            raise

    def _identity_checked(self, identity):
        self._u16(identity, 8, 'identity')
        if (identity[:6] != IDENTITY or identity[6] != 0
                or identity != self.sequence.profile['device_identity']):
            raise RuntimeError('Hand identity or sub-exception changed; no next write')

    def configuration(self):
        """Full measured configuration at entry and exit, never at every pose."""
        self._configuration_checked = False
        try:
            super().configuration()
            configuration = self.report['configuration']
            for name, length in CONFIG_LENGTHS.items():
                self._u16(configuration[name], length, name)
            self._identity_checked(configuration['identity'])
            if configuration != self.sequence.profile['observed_configuration']:
                raise RuntimeError('Full hand configuration differs from accepted profile')
            self._configuration_checked = True
            self.report.setdefault('configuration_checks', []).append(dict(
                monotonic=self.clock(), values=copy.deepcopy(configuration),
                phase='cycle_exit' if self._executing else 'entry', matched=True))
            self.save()
        except BaseException:
            self._disable()
            raise

    def configure_checked(self):
        self.configuration()
        return self.report['configuration']

    def _critical_configuration(self):
        if not self._configuration_checked:
            raise RuntimeError('Full accepted configuration required before hand feedback')
        identity = self.read('identity_before_pose', 1000, 8)
        self._identity_checked(identity)
        limits = self.read('current_limits_before_pose', 1095, 6)
        speeds = self.read('speeds_before_pose', 1125, 6)
        forces = self.read('force_targets_before_pose', 1115, 5)
        for values, length, name in ((limits, 6, 'current limit'),
                                    (speeds, 6, 'speed'), (forces, 5, 'force target')):
            self._u16(values, length, name)
        if limits != [1299]*6 or limits != self.sequence.profile['current_limit_ma']:
            raise RuntimeError('Live current protection changed; no next write')
        if speeds != [65535]*6:
            raise RuntimeError('Live recorded speeds changed; no next write')
        if any(forces):
            raise RuntimeError('Force-control target must be zero; no next write')
        return dict(speeds=speeds, force_targets=forces, current_limit_ma=limits)

    def snapshot(self):
        """One complete legal group; strict Transport still owns framing/fences."""
        began = self.clock()
        try:
            identity = self.read('identity_feedback', 1000, 8)
            self._identity_checked(identity)
            status = self.read('status', 1085, 6)
            if (len(status) != 6 or any(type(v) is not int or v not in range(6)
                                        for v in status)):
                self._protection('Invalid status feedback: '+str(status))
            if any(v in (3, 4) for v in status):
                self._protection('Current protection or unexpected force-control status: '+str(status))
            current = self.read('current_ma_0_to_4', 1105, 5)
            positions = self.read('positions', 1145, 6)
            rotation_began = self.clock()
            rotation_current = self.io.single(1110)
            self.event('read', name='rotation_current_ma', address=1110,
                       values=[rotation_current], started_monotonic=rotation_began)
            targets = self.read('targets_during_motion', 1135, 6)
            currents = current+[rotation_current]
            for values, length, name in ((currents, 6, 'current'),
                                        (positions, 6, 'position'), (targets, 6, 'target')):
                self._u16(values, length, name)
            # Guard after the whole group, never twice around each register.
            self.guard()
            limits = self.report['configuration']['current_limit_ma']
            if any(limit <= 0 or c > limit for c, limit in zip(currents, limits)):
                self._protection('Measured current exceeds live controller current limit')
            row = dict(identity=identity, status=status, current=currents,
                positions=positions, targets=targets, reads_are_sequential=True,
                started_monotonic=began, finished_monotonic=self.clock(),
                physical_stop_verified=False, torque_release_verified=False)
            self.report['last_snapshot'] = row
            self.last_feedback = row
            if 5 in status:
                self.report['status5_warning_count'] = self.report.get('status5_warning_count', 0)+1
                self.event('status5_warning', snapshot=row,
                    message='Status5 alone is not a completion/abort decision or proven firmware bug',
                    no_retry_or_target_write=True)
            self.save()
            return row
        except BaseException:
            self._disable()
            raise

    def _at_expected(self, row, expected):
        self._u16(expected, 6, 'expected target')
        if any(abs(a-b) > TARGET_TOL for a, b in zip(row['targets'], expected)):
            raise RuntimeError('Target changed outside this script: '+str(row['targets']))
        if any(abs(a-b) > TOL for a, b in zip(row['positions'], expected)):
            raise RuntimeError('Actual position is not within previous target tolerance')

    def preflight(self, expected=None, start_open=False):
        """Fresh checks after any manual wait; no cached feedback is trusted here."""
        try:
            critical = self._critical_configuration()
            row = dict(self.snapshot(), **critical)
            actual_expected = row['targets'] if expected is None else expected
            self._at_expected(row, actual_expected)
            if start_open:
                self._at_expected(row, self.sequence.profile['poses']['open'])
            self.report['last_preflight'] = row
            self.last_feedback = row
            self.save()
            return row
        except BaseException:
            self._disable()
            raise

    def wait(self, expected, active_channels, previous, min_seconds=1.0):
        try:
            return self._wait_checked(expected, active_channels, previous, min_seconds)
        except BaseException:
            self._disable()
            raise

    def _wait_checked(self, expected, active_channels, previous, min_seconds):
        self._u16(expected, 6, 'expected target')
        self._u16(previous, 6, 'previous target')
        if any(type(c) is not int or not 0 <= c <= 5 for c in active_channels):
            raise RuntimeError('Invalid active channel')
        speeds = self.report['required_speeds']
        self._u16(speeds, 6, 'speed')
        if speeds != [65535]*6:
            raise RuntimeError('Required speed profile changed')
        began = self.clock()
        travel = max((abs(expected[c]-previous[c])/speeds[c]
                      for c in active_channels), default=0)
        deadline = began+travel+30
        stable = 0
        settled_since = None
        window_minimum = window_maximum = None
        while self.clock() < deadline:
            row = self.snapshot()
            positions = row['positions']
            if any(abs(a-b) > TARGET_TOL for a, b in zip(row['targets'], expected)):
                self.event('target_changed_during_motion', expected=expected, actual=row['targets'])
                raise RuntimeError('Target changed outside this script during motion')
            for ch in range(6):
                if ch not in active_channels and abs(positions[ch]-previous[ch]) > TOL:
                    self.event('inactive_position_drift', channel=ch,
                               expected=previous[ch], actual=positions[ch])
                    raise RuntimeError('Inactive finger position drift exceeds tolerance: '+str(ch))
                if not max(0, min(previous[ch], expected[ch])-TOL) <= positions[ch] <= min(
                        65535, max(previous[ch], expected[ch])+TOL):
                    raise RuntimeError('Position outside commanded travel: '+str(positions))
            observed = row['finished_monotonic']
            reached = all(abs(a-b) <= TOL for a, b in zip(positions, expected))
            if reached:
                if settled_since is None:
                    settled_since, stable = observed, 1
                    window_minimum = positions[:]
                    window_maximum = positions[:]
                else:
                    stable += 1
                    window_minimum = [min(a,b) for a,b in zip(window_minimum, positions)]
                    window_maximum = [max(a,b) for a,b in zip(window_maximum, positions)]
            else:
                settled_since, stable = None, 0
                window_minimum = window_maximum = None
            if (observed <= deadline and reached and stable >= 3
                    and observed-settled_since >= 1.0
                    and observed-began >= min_seconds):
                self.event('actual_position_reached_and_observed', expected=expected,
                    positions=positions, status=row['status'], elapsed_seconds=observed-began,
                    settling_seconds=observed-settled_since, stable_samples=stable,
                    position_window_minimum=window_minimum,
                    position_window_maximum=window_maximum,
                    position_window_span=[b-a for a,b in zip(window_minimum, window_maximum)],
                    reads_are_sequential=True, physical_stop_verified=False,
                    torque_release_verified=False)
                return row
        raise RuntimeError('Pose timeout; no automatic return or repeated write')

    def write_pose(self, step, previous):
        """One allowed target write, with a fresh boundary check and feedback."""
        row = None
        try:
            if self._fault_latched:
                raise RuntimeError('Hand fault is latched; no further target write')
            signature = (step['phase'], step['address'], step['count'], tuple(step['values']))
            if signature not in ALLOWED_STEPS:
                raise RuntimeError('Only accepted battery/open steps are permitted')
            self._u16(previous, 6, 'previous target')
            before = self.preflight(expected=previous)
            expected = previous[:]
            ch = step['address']-1135
            expected[ch:ch+step['count']] = step['values']
            self._u16(expected, 6, 'command target')
            row = dict(step=copy.deepcopy(step), expected=expected[:], outcome='pending',
                       before=before, physical_stop_verified=False)
            self.report['moves'].append(row)
            self.save()
            self.guard()
            if (not self.sequence.hand_motion_enabled
                    or not getattr(self.io, 'motion_enabled', False)):
                raise RuntimeError('Explicit hand motion authorization required')
            self.event('pose_write_before', address=step['address'], values=step['values'])
            self.io.write_group(step['address'], step['values'])
            row['snapshot'] = self.wait(expected, tuple(range(ch, ch+step['count'])), previous)
            row['outcome'] = 'reached_and_observed'
            self.report.setdefault('pose_observations', []).append(dict(
                phase=step['phase'], expected=expected[:], before=before,
                after=row['snapshot'], reads_are_sequential=True,
                physical_stop_verified=False, torque_release_verified=False))
            self.save()
            return expected
        except BaseException as exc:
            if row is not None:
                row.update(outcome='fault_no_further_writes', error=repr(exc))
            self._disable()
            self.save()
            raise

    def execute_cycle(self, previous):
        """Caller has freshly configured/checked and explicitly enabled writes.

        No input gates occur inside the three accepted grasp and three release
        stages. Only feedback from this uninterrupted cycle is reused in the
        report and as the next travel baseline; preflight always reads anew.
        """
        if self._executing:
            raise RuntimeError('Continuous cycle is already running')
        self._executing = True
        try:
            self._u16(previous, 6, 'initial target')
            if previous != self.sequence.profile['poses']['open']:
                raise RuntimeError('Continuous battery cycle must start at accepted open target')
            if self.last_feedback is None:
                raise RuntimeError('Fresh caller preflight required before continuous cycle')
            self._at_expected(self.last_feedback, previous)
            self.report['continuous_hand_cycle'] = dict(started_monotonic=self.clock(),
                completed_stages=0, automatic_normal_open=True,
                no_automatic_fault_open=True, hardware_speed_changed=False,
                physical_stop=False, physical_stop_verified=False,
                torque_release_verified=False, status5_firmware_bug_confirmed=False)
            for index, step in enumerate(list(ADAPTED_PLAN)+list(RELEASE_PLAN)):
                print('手部 {}/6：{}'.format(index+1, step['phase']), flush=True)
                previous = self.write_pose(step, previous)
                self.report['continuous_hand_cycle']['completed_stages'] = index+1
                if index < len(ADAPTED_PLAN):
                    self.report['completed_adapted_stages'] = index+1
                self.save()
            # One full end comparison and final diagnostic group replaces the
            # legacy two preflights and angle/force reads after every pose.
            self.configuration()
            angles = self.read('angle_feedback_raw_u16', 1165, 6)
            forces = self.read('force_feedback_raw', 1175, 5)
            self._u16(angles, 6, 'angle')
            self._u16(forces, 5, 'force feedback')
            final = self.preflight(expected=self.sequence.profile['poses']['open'])
            self.report['final_hand'] = final
            self.report['pose_observations'].append(dict(phase='final_open',
                expected=previous[:], angle_raw_u16=angles, force_feedback_raw=forces,
                after=final, reads_are_sequential=True,
                physical_stop_verified=False, torque_release_verified=False))
            self.report['continuous_hand_cycle'].update(completed=True,
                finished_monotonic=self.clock())
            self.report.update(hand_open_feedback_pass=True,
                verified_hand_sequence_completed=True, physical_stop_verified=False,
                torque_release_verified=False)
            self.save()
            return final
        except BaseException as exc:
            self._disable()
            self.report.setdefault('continuous_hand_cycle', {}).update(
                completed=False, error=repr(exc), no_automatic_fault_open=True)
            self.save()
            raise
        finally:
            self._executing = False
            # An old group is never an entry permit after returning to human
            # confirmation or another phase; caller must preflight again.
            self.last_feedback = None
