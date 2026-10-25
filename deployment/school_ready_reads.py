"""Known register ranges, with per-run read-to-read pacing qualification.

100ms is a candidate host interval, NOT a manufacturer minimum. It is used
only after three complete successful read-only PREPARE groups on this run.
Every write and first read after a write retain the established300ms interval.
All six identity/target/current/position/status reads and existing guards stay.
"""
import copy

from school_battery_transport import MULTI, SINGLE, WRITE
from supported_contact_transport import WritePhase
from school_startup_contact_transport import LoadedCycleTransport, LoadedContactAuthority, FIRST_CLOSE_KEY, IDENTITY
from school_startup_hand import TrialHand
from run_school_battery_hand import TOL, TARGET_TOL
from school_startup_pose_hand import PREPARE

READ_GAP = .100
WRITE_BOUNDARY_GAP = .300


class KnownReadAuthority(LoadedContactAuthority):
    def __init__(self):
        super().__init__()
        self.read_pacing_qualified = False


class KnownReadTransport(LoadedCycleTransport):
    def __init__(self, sock, log, phase_authority):
        super().__init__(sock, log, phase_authority)
        self.read_probe_active = False
        self.identity_verified_this_socket = False

    def pre_send_quiet_seconds(self, message):
        if (message.get('command') in (MULTI, SINGLE)
                and self.previous is not None and self.previous[0] in (MULTI, SINGLE)
                and self.identity_verified_this_socket and not self.terminal_latched
                and (self.read_probe_active or self._phase_authority.read_pacing_qualified)):
            return READ_GAP
        return WRITE_BOUNDARY_GAP

    def exchange(self, message):
        value = super().exchange(message)
        if message.get('command') == MULTI and (message.get('address'), message.get('num')) == (1000, 8):
            if value != IDENTITY:
                self.latch_fault('Full current hand identity/sub-exception mismatch')
                if value[:6] == IDENTITY[:6] and value[6:] == [1,0]:
                    raise RuntimeError('ROH_SUB_EXCEPTION=1: initialization-not-ready diagnostic; no writes. Readiness changed after startup gate; normal operation is not retried.')
                raise RuntimeError('Full current hand identity/sub-exception mismatch: '+str(value))
            self.identity_verified_this_socket = True
        return value

    def begin_readonly_probe(self, authority):
        if (authority is not self._phase_authority or self._phase is not WritePhase.READONLY
                or self.motion_enabled or self.broken or self.terminal_latched
                or FIRST_CLOSE_KEY in authority._attempted_writes
                or not self.identity_verified_this_socket or self.read_probe_active):
            self.latch_fault('Read pacing probe outside a healthy preclose readonly phase')
            raise RuntimeError('Readonly PREPARE pacing qualification required')
        self.read_probe_active = True
        self.log(dict(event='known_read_pacing_probe_begin', candidate_read_to_read_seconds=READ_GAP,
            write_boundary_seconds=WRITE_BOUNDARY_GAP, motion_enabled=False,
            manufacturer_minimum_interval_claimed=False))

    def end_readonly_probe(self):
        self.read_probe_active = False

    def set_phase(self, phase, authority, approved_values=None, snapshot=None):
        if phase == 'close' and not authority.read_pacing_qualified:
            self.latch_fault('Read pacing not qualified before first close')
            raise RuntimeError('Complete readonly pacing qualification required before close')
        return super().set_phase(phase, authority, approved_values=approved_values, snapshot=snapshot)


class KnownReadHand(TrialHand):
    def qualify_read_pacing(self):
        authority = self.sequence.authority
        if authority.read_pacing_qualified:
            return
        if not self._configuration_checked:
            self.configuration()
        self._critical_configuration()
        before = self.snapshot()
        self._at_expected(before, PREPARE)
        rows = []
        self.io.begin_readonly_probe(authority)
        try:
            for _ in range(3):
                row = self.snapshot()
                self._at_expected(row, PREPARE)
                rows.append(copy.deepcopy(row))
        finally:
            # A negative read already latches the normal transport; never
            # retry it at a slower rate or use it to authorize a close.
            self.io.end_readonly_probe()
        after = self.snapshot()
        self._at_expected(after, PREPARE)
        evidence = [before] + rows + [after]
        if any(max(row['positions'][ch] for row in evidence)
                - min(row['positions'][ch] for row in evidence) > TOL for ch in range(6)):
            raise RuntimeError('PREPARE moved during readonly pacing qualification; no close')
        self.guard()
        authority.read_pacing_qualified = True
        self.report['known_read_pacing_qualification'] = dict(passed=True, groups=rows,
            before=before, after=after, actuator_writes_for_probe=0,
            read_to_read_seconds=READ_GAP, write_and_postwrite_read_seconds=WRITE_BOUNDARY_GAP,
            same_run_only=True, first_read_on_each_socket_seconds=WRITE_BOUNDARY_GAP,
            each_new_socket_requires_exact_identity=True, no_reserved_registers=True,
            manufacturer_minimum_interval_claimed=False, loaded_pacing_guaranteed=False)
        self.io.log(dict(event='known_read_pacing_qualified', complete_groups=3,
            read_to_read_seconds=READ_GAP, write_boundary_seconds=WRITE_BOUNDARY_GAP,
            actual_group_seconds=[row['finished_monotonic']-row['started_monotonic'] for row in rows]))
        self.save()

    def close_trial(self):
        self.qualify_read_pacing()
        return super().close_trial()


    def preflight(self, expected=None, start_open=False):
        if not getattr(self.sequence, 'startup_observation_only', False):
            return super().preflight(expected=expected, start_open=start_open)
        # Read-only startup observation is distinct from reaching a command.
        # No target match or observed quietness grants arm/hand motion here.
        if start_open or self.io.motion_enabled or self.sequence.hand_motion_enabled:
            raise RuntimeError('Startup observer cannot run with motion enabled')
        try:
            critical = self._critical_configuration()
            rows = []
            for _ in range(3):
                row = dict(self.snapshot(), **critical)
                # STATUS0/1 are opening/closing observations, not run modes.
                # Snapshot still blocks3/4, checks actual current and guards arm.
                # Original position/target stability checks below remain.
                self.sequence.startup_stalled_channels.update(
                    ch for ch,v in enumerate(row['status']) if v == 5)
                if expected is not None and any(abs(a-b) > TARGET_TOL
                        for a,b in zip(row['targets'], expected)):
                    raise RuntimeError('Startup target changed during confirmation')
                rows.append(copy.deepcopy(row))
            if any(max(r['positions'][ch] for r in rows)-min(r['positions'][ch] for r in rows) > TOL
                    for ch in range(6)):
                raise RuntimeError('Startup positions changed; no preparation write')
            if any(abs(a-b) > TARGET_TOL for r in rows[1:]
                    for a,b in zip(r['targets'], rows[0]['targets'])):
                raise RuntimeError('Startup targets changed; no preparation write')
            anchor = getattr(self.sequence, 'startup_hand_observation', None)
            if anchor is not None and any(any(abs(a-b) > TOL for a,b in
                    zip(r['positions'], anchor['positions'])) or any(abs(a-b) > TARGET_TOL
                    for a,b in zip(r['targets'], anchor['targets'])) for r in rows):
                raise RuntimeError('Hand changed after startup confirmation; restart required')
            if anchor is None:
                self.sequence.startup_hand_observation = copy.deepcopy(row)
            self.report['startup_observed_stalled_channels'] = sorted(self.sequence.startup_stalled_channels)
            self.report['startup_hand_observation'] = dict(groups=rows,
                old_targets_reached=False, position_tolerance=TOL,
                target_tolerance=TARGET_TOL, motion_authorized=False,
                physical_stop_verified=False, torque_release_verified=False)
            self.report['last_preflight'] = row
            self.last_feedback = row
            self.save()
            return row
        except BaseException:
            self._disable()
            raise

    def prepare_empty_start(self, previous):
        # h is an explicit empty-hand/full-sweep clearance permission.
        # Observe afresh before authorizing one exact new PREPARE command.
        try:
            if not self.sequence.startup_observation_only or self._fault_latched:
                raise RuntimeError('Only one nonfault startup preparation is allowed')
            before = self.preflight(expected=previous)
            for ch, status in enumerate(before['status']):
                if (status == 5 or ch in self.sequence.startup_stalled_channels) and (PREPARE[ch] > before['targets'][ch]
                        or PREPARE[ch] > before['positions'][ch] + TOL):
                    raise RuntimeError('Startup PREPARE would add travel on stalled channel ' + str(ch))
            self.guard()
            self.report['startup_prepare_before'] = dict(snapshot=copy.deepcopy(before),
                targets=PREPARE[:], one_attempt=True, no_protection_reset=True,
                previous_target_is_not_position_anchor=True)
            self.save()
            self.io.set_phase('prepare_empty', self.sequence.authority)
            self.io.enable_motion()
            self.sequence.hand_motion_enabled = True
            self.io.write_group(1135, PREPARE)
            # The travel envelope starts at actual positions, not the old
            # unachieved target. Post-command position/target tolerances stay.
            final = self.wait(PREPARE, tuple(range(6)), before['positions'])
            self.sequence.startup_observation_only = False
            self.sequence.startup_normalized = True
            self.report['startup_prepare_feedback'] = copy.deepcopy(final)
            self.report['startup_normalized'] = True
            self.save()
            return self.preflight(expected=PREPARE)
        except BaseException:
            self._disable()
            raise
