"""Supported, de-energized battery contact observation with the arm held still.

This policy issues no force/reset/release/arm commands. The dedicated caller
owns explicit motion and release authorization. Sampled force observations do
not certify grip strength, physical stopping, sensor calibration or safe lift.
"""
import copy
import math

from continuous_hand_cycle import ContinuousHand, RELEASE_PLAN
from run_school_battery_hand import ADAPTED_PLAN, TOL, TARGET_TOL


PREPARE = tuple(ADAPTED_PLAN[0]['values'])
FORCE_TARGETS = (200, 200, 0, 0, 0)
ZERO_FORCE = (0, 0, 0, 0, 0)
SWEEP_CAP = (15000, 40000, 35000, 40000, 65535, 65535)
TRIAL_FORCE_MN = 200
ACTIVE_CHANNELS = (0, 1)


class ContactHand(ContinuousHand):
    """Contact-specific feedback; position completion is not grip completion."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._release_transition_in_progress = False
        self._normal_release_ready = False
        self._release_index = 0
        self._empty_preparation_authorized = False
        self.report['supported_contact_policy'] = dict(
            force_targets_mn=list(FORCE_TARGETS), active_channels=list(ACTIVE_CHANNELS),
            trial_force_setpoint_mn=TRIAL_FORCE_MN,
            trial_force_source='manufacturer_CXX_example_trial_choice_not_load_rating',
            force_sensor_calibration_verified=False, safe_grip_verified=False,
            physical_stop_verified=False, torque_release_verified=False,
            automatic_lift=False, automatic_normal_open=False,
            no_automatic_fault_open=True, minimum_complete_groups=3,
            observation_seconds=1.0, timeout_seconds=30.0,
            approved_position_sweep_caps=list(SWEEP_CAP),
            inactive_position_reference=list(PREPARE), fault_latched=False)
        self.report['hand_feedback_policy'] = self.report['supported_contact_policy']

    def _disable(self):
        # The inherited latch also blocks an accidental inherited position write.
        try:
            super()._disable()
        finally:
            self._normal_release_ready = False
            self._empty_preparation_authorized = False
            self.report.setdefault('supported_contact_policy', {}).update(
                fault_latched=True, safe_grip_verified=False,
                no_automatic_fault_open=True)
            self.save()

    def snapshot(self):
        """Only explicitly empty preparation reuses the accepted warning5 policy."""
        try:
            self._ready()
            row = super().snapshot()
            if self._empty_context():
                forces = self.read('empty_prepare_force_mode', 1115, 5)
                self._u16(forces, 5, 'empty prepare force targets')
                if any(forces):
                    raise RuntimeError('Empty preparation became force-controlled; no continuation')
                row['force_targets'] = forces
            elif 5 in row['status']:
                self._protection('Stall status5 blocks supported-contact prepare/release')
            return row
        except BaseException:
            self._disable()
            raise

    def _empty_context(self):
        authority = getattr(self.sequence, 'authority', None)
        return bool(self._empty_preparation_authorized and authority is not None
                    and authority is getattr(self.io, '_phase_authority', None)
                    and not authority._terminal_latched and not authority._completed
                    and not getattr(self.sequence, 'force_written', False)
                    and (1115, FORCE_TARGETS) not in authority._attempted_writes
                    and getattr(self.io, 'phase', None) in ('readonly', 'prepare_empty', 'reset_empty'))

    def authorize_empty_preparation(self, authority):
        """Entry-owned p permission; zero force alone never establishes empty hand."""
        try:
            self._ready()
            if (authority is not getattr(self.sequence, 'authority', None)
                    or authority is not getattr(self.io, '_phase_authority', None)
                    or self.report.get('onsite_empty_preparation_confirmed') is not True
                    or getattr(self.sequence, 'force_written', False)
                    or (1115, FORCE_TARGETS) in authority._attempted_writes
                    or authority._terminal_latched
                    or authority._completed
                    or getattr(self.io, 'phase', None) not in ('readonly', 'prepare_empty', 'reset_empty')
                    or self._empty_preparation_authorized):
                raise RuntimeError('Fresh entry-owned empty preparation confirmation required')
            self._empty_preparation_authorized = True
            self.report['empty_prepare_status5_policy'] = dict(
                authorized=True, empty_hand_confirmed_by_operator=True,
                status5_is_warning_only=True, actual_position_completion_still_required=True,
                source='accepted112051_and120502_empty_position_policy',
                applies_to_loaded_contact_or_release=False, firmware_bug_confirmed=False)
            self.save()
        except BaseException:
            self._disable()
            raise

    def end_empty_preparation(self):
        self._empty_preparation_authorized = False
        self.report.setdefault('empty_prepare_status5_policy', {})['authorized'] = False
        self.save()

    def _ready(self):
        if self._fault_latched:
            raise RuntimeError('Contact fault is latched; no retry or release from this policy')
        if not self._configuration_checked:
            raise RuntimeError('Full accepted hand configuration required before contact feedback')
        limits = self.report.get('configuration', {}).get('current_limit_ma')
        self._u16(limits, 6, 'current limit')
        if limits != [1299]*6 or limits != self.sequence.profile['current_limit_ma']:
            raise RuntimeError('Accepted current protection changed; no contact continuation')

    def _expected(self, expected_force, expected_position_targets):
        self._u16(expected_force, 5, 'expected force targets')
        self._u16(expected_position_targets, 6, 'expected position targets')
        if tuple(expected_force) not in (ZERO_FORCE, FORCE_TARGETS):
            raise RuntimeError('Only zero or the accepted 200mN thumb/index trial is permitted')
        if tuple(expected_position_targets) != PREPARE:
            raise RuntimeError('Contact must retain the accepted prepare position targets')

    def snapshot_contact(self, expected_force, expected_position_targets=PREPARE,
                         release_transition=False):
        """Read one complete contact group and check mode, targets and travel.

        Status4 is permitted only on explicitly force-controlled thumb/index.
        Transient0/1/2 can be sampled while entering contact; completion requires
        both active channels at4. Every3/5 or unexpected4 remains blocking.
        """
        began = self.clock()
        try:
            self._ready()
            self._expected(expected_force, expected_position_targets)
            if release_transition and (not self._release_transition_in_progress
                                       or tuple(expected_force) != ZERO_FORCE):
                raise RuntimeError('Force-zero transition may only be observed by release wait')
            identity = self.read('contact_identity', 1000, 8)
            self._identity_checked(identity)
            status = self.read('contact_status', 1085, 6)
            self._u16(status, 6, 'status')
            if any(v not in range(6) for v in status):
                self._protection('Invalid contact status feedback')
            empty_only = (self._empty_context()
                          and tuple(expected_force) == ZERO_FORCE and not release_transition)
            if any(v == 3 or (v == 5 and not empty_only) for v in status):
                self._protection('Contact protection/stall status; no further writes')
            if any(v == 4 and (ch not in ACTIVE_CHANNELS
                              or not (expected_force[ch] or release_transition))
                   for ch, v in enumerate(status)):
                self._protection('Unexpected force-control channel; no further writes')
            current = self.read('contact_current_ma_0_to_4', 1105, 5)
            positions = self.read('contact_positions', 1145, 6)
            rotation_current = self.io.single(1110)
            self.event('read', name='contact_rotation_current_ma', address=1110,
                       values=[rotation_current])
            targets = self.read('contact_position_targets', 1135, 6)
            force_targets = self.read('contact_force_targets', 1115, 5)
            forces = self.read('contact_force_feedback_raw', 1175, 5)
            currents = current + [rotation_current]
            for values, count, name in ((currents, 6, 'current'),
                    (positions, 6, 'position'), (targets, 6, 'position target'),
                    (force_targets, 5, 'force target'), (forces, 5, 'force feedback')):
                self._u16(values, count, name)
            self.guard()
            if any(c > 1299 for c in currents):
                self._protection('Measured current exceeds live controller current limit')
            if force_targets != list(expected_force):
                raise RuntimeError('Force target/mode changed outside this contact trial')
            if any(abs(a-b) > TARGET_TOL for a, b in zip(targets, expected_position_targets)):
                raise RuntimeError('Position target changed during contact')
            if any(not 0 <= p <= min(65535, cap+TOL)
                   for p, cap in zip(positions, SWEEP_CAP)):
                raise RuntimeError('Actual contact position exceeds accepted sweep cap')
            if any(positions[ch] < PREPARE[ch]-TOL for ch in ACTIVE_CHANNELS):
                raise RuntimeError('Active finger moved outward below accepted prepare aperture')
            if any(abs(positions[ch]-PREPARE[ch]) > TOL for ch in range(2, 6)):
                raise RuntimeError('Uncommanded finger position drift during contact')
            row = dict(identity=identity, status=status, current=currents,
                positions=positions, targets=targets, force_targets=force_targets,
                forces=forces, force_feedback_raw=forces[:], reads_are_sequential=True,
                started_monotonic=began, finished_monotonic=self.clock(),
                physical_stop_verified=False, torque_release_verified=False,
                safe_grip_verified=False)
            if empty_only and 5 in status:
                self.report['status5_warning_count'] = self.report.get('status5_warning_count', 0)+1
                self.event('empty_prepare_status5_warning', snapshot=row,
                           physical_stop_verified=False, firmware_bug_confirmed=False)
            self.report['last_contact_snapshot'] = copy.deepcopy(row)
            self.last_feedback = row
            self.event('contact_feedback_group', snapshot=row)
            self.save()
            return row
        except BaseException:
            self._disable()
            raise

    def sensor_baseline(self, samples=3):
        """Observe raw offsets without any sensor-zero or force writes."""
        try:
            if type(samples) is not int or samples < 3:
                raise RuntimeError('At least three fresh sensor baseline groups required')
            rows = []
            for _ in range(samples):
                row = self.snapshot_contact(ZERO_FORCE)
                if any(v >= TRIAL_FORCE_MN for v in row['forces']):
                    raise RuntimeError('Raw force baseline already reaches trial threshold; no force reset')
                if any(abs(a-b) > TOL for a, b in zip(row['positions'], PREPARE)):
                    raise RuntimeError('Sensor baseline requires actual accepted prepare pose')
                rows.append(row)
            self.report['contact_sensor_baseline'] = dict(
                rows=copy.deepcopy(rows), below_trial_threshold=True,
                sensor_zero_written_by_policy=False, force_sensor_calibration_verified=False,
                safe_grip_verified=False,
                raw_minimum=[min(row['forces'][ch] for row in rows) for ch in range(5)],
                raw_maximum=[max(row['forces'][ch] for row in rows) for ch in range(5)],
                sampled_raw_span=[max(row['forces'][ch] for row in rows)
                                  - min(row['forces'][ch] for row in rows) for ch in range(5)],
                contact_threshold_relaxed=False)
            self.save()
            return rows
        except BaseException:
            self._disable()
            raise

    def wait_contact(self, expected_force=FORCE_TARGETS,
                     expected_position_targets=PREPARE, timeout=30, min_seconds=1.0):
        """Observe both active force modes/thresholds for >=3 groups and >=1s."""
        try:
            self._ready()
            self._normal_release_ready = False
            self._expected(expected_force, expected_position_targets)
            if tuple(expected_force) != FORCE_TARGETS:
                raise RuntimeError('Contact completion requires the accepted thumb/index force trial')
            if (type(timeout) not in (int, float) or not math.isfinite(timeout)
                    or not 0 < timeout <= 30 or type(min_seconds) not in (int, float)
                    or not math.isfinite(min_seconds) or min_seconds < 1):
                raise RuntimeError('Contact observation requires >=1s and a bounded <=30s deadline')
            began = self.clock()
            deadline = began+timeout
            stable = 0
            settled_since = None
            while self.clock() < deadline:
                row = self.snapshot_contact(expected_force, expected_position_targets)
                observed = row['finished_monotonic']
                reached = all(row['status'][ch] == 4 and row['forces'][ch] >= TRIAL_FORCE_MN
                              for ch in ACTIVE_CHANNELS)
                if reached:
                    if settled_since is None:
                        settled_since, stable = observed, 1
                    else:
                        stable += 1
                else:
                    settled_since, stable = None, 0
                if (observed <= deadline and reached and stable >= 3
                        and observed-settled_since >= min_seconds):
                    self.report['supported_contact_observation'] = dict(
                        feedback_threshold_observed=True, stable_samples=stable,
                        observed_seconds=observed-settled_since,
                        elapsed_seconds=observed-began, final=copy.deepcopy(row),
                        safe_grip_verified=False, real_battery_hold_verified=False,
                        physical_stop_verified=False, torque_release_verified=False,
                        automatic_lift=False, automatic_normal_open=False)
                    self.save()
                    return row
            raise RuntimeError('Contact feedback timeout; no force increase, retry or automatic release')
        except BaseException:
            self._disable()
            raise

    def wait_release_to_prepare(self, previous_positions, timeout=30):
        """After caller-authorized force-zero, observe return to prepare.

        Force-zero exits force mode. Return to the retained position targets is
        observed here, not assumed from that ACK. Active status4 is pending,
        never completion; no force/reset/open command is sent by this method.
        """
        try:
            self._ready()
            self._normal_release_ready = False
            self._u16(previous_positions, 6, 'previous contact positions')
            if (type(timeout) not in (int, float) or not math.isfinite(timeout)
                    or not 0 < timeout <= 30):
                raise RuntimeError('Release transition needs a bounded <=30s deadline')
            if (any(not 0 <= p <= min(65535, cap+TOL)
                    for p, cap in zip(previous_positions, SWEEP_CAP))
                    or any(previous_positions[ch] < PREPARE[ch]-TOL
                           for ch in ACTIVE_CHANNELS)
                    or any(abs(previous_positions[ch]-PREPARE[ch]) > TOL
                           for ch in range(2, 6))):
                raise RuntimeError('Previous contact positions exceed accepted travel bounds')
            self._release_transition_in_progress = True
            began = self.clock()
            deadline = began+timeout
            stable = 0
            settled_since = None
            while self.clock() < deadline:
                row = self.snapshot_contact(ZERO_FORCE, release_transition=True)
                if any(not max(0, min(a, b)-TOL) <= p <= min(65535, max(a, b)+TOL)
                       for p, a, b in zip(row['positions'], previous_positions, PREPARE)):
                    raise RuntimeError('Release return leaves previous-to-prepare travel bounds')
                observed = row['finished_monotonic']
                reached = (4 not in row['status'] and all(abs(a-b) <= TOL
                           for a, b in zip(row['positions'], PREPARE)))
                if reached:
                    if settled_since is None:
                        settled_since, stable = observed, 1
                    else:
                        stable += 1
                else:
                    settled_since, stable = None, 0
                if (observed <= deadline and reached and stable >= 3
                        and observed-settled_since >= 1.0):
                    self._normal_release_ready = True
                    self._release_index = 0
                    self.report['contact_release_to_prepare'] = dict(
                        observed=True, stable_samples=stable,
                        observed_seconds=observed-settled_since,
                        final=copy.deepcopy(row), physical_stop_verified=False,
                        torque_release_verified=False)
                    self.save()
                    return row
            raise RuntimeError('Force-zero return-to-prepare timeout; no automatic open')
        except BaseException:
            self._disable()
            raise
        finally:
            self._release_transition_in_progress = False

    def write_pose(self, step, previous):
        """Prepare is inherited; opening requires the observed release gate."""
        preparing = step == ADAPTED_PLAN[0]
        releasing = (self._normal_release_ready and self._release_index < 3
                     and step == RELEASE_PLAN[self._release_index])
        if not preparing and not releasing:
            self._disable()
            raise RuntimeError('Contact policy permits prepare or gated normal release only')
        result = super().write_pose(step, previous)
        if releasing:
            self._release_index += 1
            if self._release_index == 3:
                self._normal_release_ready = False
        return result

    def execute_cycle(self, previous):
        self._disable()
        raise RuntimeError('Supported contact cannot execute the empty-hand automatic-open cycle')
