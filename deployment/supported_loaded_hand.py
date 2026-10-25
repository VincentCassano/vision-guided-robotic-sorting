"""Supported contact freeze, read-only loaded observation, supported opening.

Position feedback is not a force measurement or a guarantee of holding a load.
All transport writes remain fenced by the entry's phase capability. No class
clears a protection, resets the device, changes limits, or retries a target.
"""
import copy

from run_school_battery_hand import ProtectionFault, TOL, TARGET_TOL
from supported_loaded_transport import FIRST_CLOSE, FREEZE_PHASE
from vision_cycle_hand import CycleHand, PREPARE


class SupportedStatus3(ProtectionFault):
    """Validated status3 provenance, never an authorization to move or open."""
    def __init__(self, status, feedback, origin):
        super().__init__('Current protection during supported close: '+str(status))
        self.status = copy.deepcopy(status)
        self.feedback = copy.deepcopy(feedback)
        self.origin = origin


def _remember_protection(hand, row=None, **details):
    row = copy.deepcopy(hand.report.get('last_snapshot', {}) if row is None else row)
    entry = dict(phase=hand.report.get('phase'), feedback=row, **details)
    # Expose the exact status origin separately when a short read, rather than
    # another complete group, detected the protection before the freeze.
    entry['status'] = copy.deepcopy(details.get('fresh_status', row.get('status', [])))
    rows = hand.report.setdefault('loaded_protection_rows', [])
    rows.append(entry)
    hand.sequence.loaded_protection_rows = rows
    hand.save()


def _dynamic_bounds(row, targets, anchor=None, reached=True):
    if (not isinstance(targets, (list, tuple)) or len(targets) != 6
            or any(type(v) is not int or not 0 <= v <= 65535 for v in targets)):
        raise ValueError('Six integer loaded targets are required')
    if any(not 8000 <= value <= cap + TARGET_TOL
            for value, cap in zip(targets[:4], FIRST_CLOSE[1][:4])):
        raise RuntimeError('Loaded flex targets outside accepted withdrawal/recipe bounds')
    if list(targets[4:]) != [65535, 65535] or row['targets'][4:] != [65535, 65535]:
        raise RuntimeError('Held little/rotation targets must remain65535')
    if any(abs(a - b) > TARGET_TOL for a, b in zip(row['targets'], targets)):
        raise RuntimeError('Loaded targets changed outside this run')
    if any(not 8000 - TOL <= value <= cap + TOL
            for value, cap in zip(row['positions'][:4], FIRST_CLOSE[1][:4])):
        raise RuntimeError('Loaded actual fingers left the accepted supported sweep')
    if any(abs(value - 65535) > TOL for value in row['positions'][4:]):
        raise RuntimeError('Held little/rotation actual orientation changed')
    if reached and any(abs(a - b) > TOL for a, b in zip(row['positions'], targets)):
        raise RuntimeError('Loaded position target is not observed within150')
    if anchor is not None:
        if (not isinstance(anchor, dict) or len(anchor.get('positions', [])) != 6
                or len(anchor.get('targets', [])) != 6):
            raise ValueError('Complete loaded hand anchor is required')
        if any(abs(a - b) > TARGET_TOL for a, b in zip(anchor['targets'], targets)):
            raise RuntimeError('Loaded anchor target differs from this run')
        if any(abs(a - b) > TOL for a, b in zip(row['positions'], anchor['positions'])):
            raise RuntimeError('Loaded finger position drifted beyond150 from its anchor')


class TrialHand(CycleHand):
    """One school close and one target reduction per contacting flex channel."""
    def read_feedback_group(self):
        # All six existing reads and every base-class validation are retained.
        # Status is last, so the complete group itself supplies the pre-freeze
        # status; no repeated six-word read/identity fence delays that decision.
        began = self.clock()
        identity = self.read('identity_feedback', 1000, 8)
        targets = self.read('targets_during_motion', 1135, 6)
        current = self.read('current_ma_0_to_4', 1105, 5)
        positions = self.read('positions', 1145, 6)
        rotation_began = self.clock()
        rotation_current = self.io.single(1110)
        self.event('read', name='rotation_current_ma', address=1110,
            values=[rotation_current], started_monotonic=rotation_began)
        status_began = self.clock()
        status = self.read('status', 1085, 6)
        self.report['feedback_policy']['feedback_read_order'] = [1000, 1135, 1105, 1145, 1110, 1085]
        return dict(identity=identity, status=status, current=current+[rotation_current],
            positions=positions, targets=targets, reads_are_sequential=True,
            started_monotonic=began, finished_monotonic=self.clock(),
            final_status_started_monotonic=status_began, status_is_last_group_read=True,
            physical_stop_verified=False, torque_release_verified=False)

    def check_status(self, row):
        status = row['status']
        if 3 in status and 4 not in status and all(status[ch] != 3 for ch in (4, 5)):
            _remember_protection(self, row, origin='validated_complete_group')
            try:
                self._protection('Current protection during supported close: '+str(status))
            except ProtectionFault as exc:
                raise SupportedStatus3(status, row, 'validated_complete_group') from exc
        return super().check_status(row)

    def snapshot(self):
        try:
            return super().snapshot()
        except SupportedStatus3:
            raise
        except ProtectionFault:
            _remember_protection(self)
            raise

    def close_trial(self):
        if not self._configuration_checked:
            self.configuration()
        critical = self._critical_configuration()
        before = self.snapshot()
        self._at_expected(before, PREPARE)
        self.sequence.authority.loaded_configuration = copy.deepcopy(self.report['configuration'])
        self.io.set_phase('close', self.sequence.authority)
        self.io.enable_motion()
        self.sequence.hand_motion_enabled = True
        expected = list(FIRST_CLOSE[1])
        self.report.update(grasp_target_steps_planned=1,
            contact_freeze_is_not_force_control=True,
            maximum_contact_reductions=4, maximum_reductions_per_channel=1,
            contact_reduction_scope='only_new_status5_channels_0_to_3',
            no_additional_school_index_close=True)
        self.event('supported_trial_close_before', address=1135, values=expected)
        self.save()
        self.io.write_group(1135, expected)
        self.sequence.load_targets = expected[:]
        self.report['load_targets'] = expected[:]
        self.save()
        began = self.clock()
        quiet_since, groups = None, 0
        low = high = None
        while self.clock() - began < 30:
            row = dict(self.snapshot(), **critical)
            self.check_targets(row, expected)
            _dynamic_bounds(row, expected, reached=False)
            continued = [ch for ch in self.sequence.authority.frozen_channels
                if row['positions'][ch] > expected[ch] + TOL]
            if continued:
                self.event('continued_closure_after_contact_reduction', channels=continued,
                    snapshot=row, targets=expected[:], hardware_stop_required=True)
                self._disable()
                raise RuntimeError('Finger continued closing after target reduction; hardware stop, no retry')
            first_contact = self.sequence.authority.observe_first_close_feedback(row)
            if first_contact:
                channels = self.sequence.authority.pending_freeze_channels
                frozen = expected[:]
                for ch in channels:
                    frozen[ch] = min(row['positions'][ch], expected[ch])
                if any(value < 8000 for value in frozen[:4]):
                    self._disable()
                    raise RuntimeError('First contact too early for withdrawal-only supported opening')
                if row.get('status_is_last_group_read') is not True:
                    self._disable()
                    raise RuntimeError('Contact freeze requires status-last complete feedback')
                immediate = row['status'][:]
                self._u16(immediate, 6, 'pre-freeze status')
                self.report['contact_status_before_freeze'] = dict(
                    values=immediate[:], source_group=copy.deepcopy(row),
                    origin='last_read_of_same_complete_group',
                    extra_status_read_omitted=True, reads_are_sequential=True)
                self.save()
                if any(value not in range(6) for value in immediate):
                    self._protection('Invalid immediate pre-freeze status')
                if any(value in (3, 4) for value in immediate):
                    self._protection('Protection in final group status forbids contact freeze')
                self.guard()
                authorized = dict(row, status_immediately_before_freeze=immediate[:])
                self.io.set_phase(FREEZE_PHASE, self.sequence.authority,
                    approved_values=frozen, snapshot=authorized)
                self.io.enable_motion()
                self.event('supported_contact_freeze_before', address=1135, values=frozen,
                    channels=list(channels),
                    source_group=row, latest_status=immediate,
                    holding_force_verified=False, physical_stop_verified=False)
                self.report['contact_freeze'] = dict(values=frozen[:], source_group=copy.deepcopy(row),
                    channels=list(channels),
                    latest_status=immediate[:], no_added_closure=True,
                    final_status_to_write_request_s=self.clock()-row['finished_monotonic'],
                    timing_scope='before_transport_quiet_wait_not_wire_or_motor_stop',
                    physical_stop_verified=False, holding_force_verified=False)
                self.save()
                self.io.write_group(1135, frozen)
                self.report.setdefault('contact_freezes', []).append(copy.deepcopy(self.report['contact_freeze']))
                # Preserve the acknowledged dynamic target even if the next
                # complete group raises protection before settling. It is the
                # scope for independent supported withdrawal, not a holding
                # acceptance or a replacement for the fault's actual row.
                self.sequence.load_targets = frozen[:]
                expected = frozen[:]
                self.sequence.load_anchor = dict(copy.deepcopy(row), targets=frozen[:])
                self.report['load_targets'] = frozen[:]
                self.report['load_anchor'] = copy.deepcopy(self.sequence.load_anchor)
                self.save()
                quiet_since, groups = None, 0
                low = high = None
                continue
            positions = row['positions']
            if low is None:
                low, high = positions[:], positions[:]
            low = [min(a, b) for a, b in zip(low, positions)]
            high = [max(a, b) for a, b in zip(high, positions)]
            reached = all(abs(a-b) <= TOL for a,b in zip(positions, expected))
            quiet = max(b-a for a,b in zip(low, high)) <= TOL
            if reached and quiet:
                if quiet_since is None:
                    quiet_since = self.clock()
                groups += 1
                if groups >= 3 and self.clock()-quiet_since >= 1:
                    row = dict(row, **self._critical_configuration())
                    self.report['loaded_contact_settling'] = dict(targets=expected[:],
                        final=copy.deepcopy(row), complete_groups=groups, position_span_within150=True,
                        frozen_channels=sorted(self.sequence.authority.frozen_channels),
                        physical_stop_verified=False, holding_force_verified=False,
                        opposed_contact_requires_onsite_confirmation=True)
                    self.sequence.load_targets = expected[:]
                    self.sequence.load_anchor = copy.deepcopy(row)
                    self.report.update(load_targets=expected[:], load_anchor=copy.deepcopy(row))
                    self.save()
                    return row
            else:
                low, high = positions[:], positions[:]
                quiet_since, groups = None, 0
        self._disable()
        raise TimeoutError('No supported contact/reached observation; no retry or lift')


class MovingLoadedHand(TrialHand):
    """Read-only loaded observation; the sequence owns the live arm corridor."""
    def guard(self):
        try:
            self.sequence.arm_state()
        except BaseException:
            self._disable()
            raise

    def write_pose(self, *args, **kwargs):
        raise RuntimeError('Moving loaded hand is read only')

    def close_trial(self):
        raise RuntimeError('Moving loaded hand cannot close or freeze')

    def release_prepare(self, *args, **kwargs):
        raise RuntimeError('Moving loaded hand cannot release')

    def preflight_loaded(self, targets, anchor=None, allow_existing_protection=False):
        if allow_existing_protection:
            raise RuntimeError('A loaded arm move cannot allow an existing hand protection')
        if self.io.motion_enabled:
            raise RuntimeError('Loaded motion observer must have a read-only hand socket')
        if not self._configuration_checked:
            self.configuration()
        critical = self._critical_configuration()
        row = dict(self.snapshot(), **critical)
        _dynamic_bounds(row, targets, anchor, reached=True)
        self.loaded_critical = critical
        self.report['last_loaded_preflight'] = copy.deepcopy(row)
        self.save()
        return row

    def monitor_loaded(self, targets, anchor):
        if self.io.motion_enabled:
            raise RuntimeError('Loaded motion observation cannot enable hand writes')
        row = dict(self.snapshot(), **getattr(self, 'loaded_critical', {}))
        _dynamic_bounds(row, targets, anchor, reached=True)
        self.report['last_loaded_monitor'] = copy.deepcopy(row)
        self.save()
        return row

    def check_current(self, targets, anchor):
        return self.monitor_loaded(targets, anchor)


class SupportedOpenHand(TrialHand):
    """Independent four-word withdrawal, including recorded existing status3."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.allow_existing_protection = False
        self.existing_status3 = set()
        self.opening_attempted = False
        self.loaded_targets = None
        self.loaded_anchor = None

    def check_status(self, row):
        status = row['status']
        if any(value == 4 for value in status):
            self._protection('Force-control status4 blocks supported opening')
        channels = {ch for ch, value in enumerate(status) if value == 3}
        if channels:
            if (not self.allow_existing_protection or not channels <= self.existing_status3
                    or any(ch >= 4 for ch in channels)):
                self._protection('New/held-channel current protection blocks supported opening')
        self.existing_status3.intersection_update(channels)
        self.report.setdefault('loaded_opening_status_observations', []).append(dict(
            status=status[:], retained_status3=sorted(self.existing_status3),
            opening_attempted=self.opening_attempted, protection_cleared_claimed=False))
        self.save()

    def preflight_loaded(self, targets, anchor=None, allow_existing_protection=False):
        if self.loaded_targets is not None:
            raise RuntimeError('Supported opening reference can only be bound once')
        self.allow_existing_protection = bool(allow_existing_protection)
        if self.allow_existing_protection:
            records = getattr(self.sequence, 'loaded_protection_rows',
                self.report.get('loaded_protection_rows', []))
            if records:
                status = records[-1].get('status', records[-1].get('feedback', {}).get('status', []))
                if len(status) != 6 or any(type(v) is not int or v not in range(6) for v in status):
                    raise RuntimeError('Recorded protection provenance is incomplete')
                if 4 in status or any(status[ch] == 3 for ch in (4, 5)):
                    raise RuntimeError('Recorded force/held-channel protection forbids supported opening')
                self.existing_status3 = {ch for ch, value in enumerate(status) if value == 3}
        if not self._configuration_checked:
            self.configuration()
        critical = self._critical_configuration()
        row = dict(self.snapshot(), **critical)
        _dynamic_bounds(row, targets, reached=False)
        if any(value < 8000 for value in row['positions'][:4]):
            raise RuntimeError('Supported opening target8000 would add actual flex closure')
        if anchor is not None:
            # Contact can be partial before a fault. Targets, not an old
            # all-position-equality assumption, bind this withdrawal scope.
            if any(abs(a - b) > TARGET_TOL for a, b in zip(anchor['targets'], targets)):
                raise RuntimeError('Supported opening anchor targets changed')
        self.loaded_targets = list(targets)
        self.loaded_anchor = copy.deepcopy(row)
        self.report['last_loaded_preflight'] = copy.deepcopy(row)
        self.save()
        return row

    def monitor_loaded(self, targets, anchor):
        row = self.snapshot()
        _dynamic_bounds(row, targets, reached=False)
        if anchor is not None and any(abs(a - b) > TOL
                for a, b in zip(row['positions'], anchor['positions'])):
            raise RuntimeError('Supported opening preparation not observed quiet')
        return row

    def release_prepare(self):
        if self.loaded_targets is None or self.loaded_anchor is None or self.opening_attempted:
            raise RuntimeError('Single freshly bound supported opening required')
        self._critical_configuration()
        row = self.snapshot()
        _dynamic_bounds(row, self.loaded_targets, reached=False)
        if any(value < 8000 for value in row['positions'][:4]):
            raise RuntimeError('Supported opening target8000 would add actual flex closure')
        if not self.sequence.hand_motion_enabled or not self.io.motion_enabled:
            raise RuntimeError('Independent supported opening capability is not enabled')
        self.loaded_anchor = copy.deepcopy(row)
        self.report['opening_write_before'] = dict(snapshot=copy.deepcopy(row),
            address=1135, count=4, values=[8000] * 4, untouched_channels=[4, 5],
            status3_channels=sorted(self.existing_status3), protection_cleared_claimed=False)
        self.save()
        self.opening_attempted = True
        self.io.write_group(1135, [8000] * 4)
        began = self.clock()
        quiet_since = None
        groups = 0
        previous = None
        minima = row['positions'][:4]
        while self.clock() - began < 30:
            current = self.snapshot()
            if (any(abs(value - 8000) > TARGET_TOL for value in current['targets'][:4])
                    or current['targets'][4:] != self.loaded_targets[4:]):
                raise RuntimeError('Supported opening target changed outside the single write')
            if any(abs(a - b) > TOL for a, b in zip(current['positions'][4:], row['positions'][4:])):
                raise RuntimeError('Unwritten little/rotation orientation changed during opening')
            if any(not 8000 - TOL <= value <= initial + TOL
                    for value, initial in zip(current['positions'][:4], row['positions'][:4])):
                raise RuntimeError('Opening left the actual-start withdrawal sweep')
            if any(value > minimum + TOL for value, minimum in zip(current['positions'][:4], minima)):
                raise RuntimeError('Finger reclosed during supported opening')
            minima = [min(a, b) for a, b in zip(minima, current['positions'][:4])]
            reached = all(abs(value - 8000) <= TOL for value in current['positions'][:4])
            quiet = previous is not None and all(abs(a - b) <= TOL
                for a, b in zip(current['positions'], previous['positions']))
            if reached and quiet:
                if quiet_since is None:
                    quiet_since = self.clock()
                groups += 1
                if groups >= 3 and self.clock() - quiet_since >= 1:
                    critical = self._critical_configuration()
                    current = dict(current, **critical)
                    self.report['opening_feedback'] = dict(final=copy.deepcopy(current),
                        commanded_channels=[0, 1, 2, 3], untouched_channels=[4, 5],
                        final_status3_channels=sorted(self.existing_status3),
                        protection_cleared_claimed=False, torque_release_verified=False)
                    self.report['opening_retreat_confirmed'] = True
                    self.save()
                    return current
            else:
                quiet_since, groups = None, 0
            previous = current
        self._disable()
        raise TimeoutError('Supported opening unconfirmed; no retry/reset/automatic arm motion')
