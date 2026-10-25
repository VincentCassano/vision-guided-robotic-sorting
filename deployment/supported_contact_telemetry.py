"""Four live contact reads, with unchanged300ms pacing and explicit provenance.

Identity is checked in full immediately before close and again before accepting
the grasp. Each contact group freshly reads ALL targets, currents, positions and
statuses. Documented reserved tail slots distinguish response lengths; their
values are logged, never interpreted or written. No global read/write allowlist
or hardware protection is changed.
"""
import copy
import math
import time

from school_battery_transport import MULTI, WRITE
from supported_loaded_transport import (
    LoadedCycleTransport, LoadedContactAuthority, FIRST_CLOSE_KEY, FREEZE_PHASE, IDENTITY)
from supported_loaded_hand import TrialHand, _dynamic_bounds
from run_school_battery_hand import TOL, TARGET_TOL

BLOCKS = ((1135, 9), (1105, 8), (1145, 7), (1085, 6))
EXTRA_READS = frozenset(BLOCKS[:3])


def validate_contact_source(authority, row):
    anchor = authority.contact_identity_anchor
    ack = authority.contact_close_ack_monotonic
    if (not authority.contact_layout_probed or not isinstance(anchor, dict)
            or row.get('identity_source') != 'validated_full_group_immediately_before_close'
            or row.get('identity_read_this_group') is not False
            or row.get('identity_source_finished_monotonic') != anchor.get('finished_monotonic')
            or row.get('identity') != IDENTITY or anchor.get('identity') != IDENTITY
            or type(ack) not in (int, float) or not math.isfinite(ack)
            or not anchor['finished_monotonic'] <= ack <= row['started_monotonic']
            or not 0 <= row['finished_monotonic']-row['started_monotonic'] <= 3):
        raise RuntimeError('Contact feedback provenance/timing invalid; no target reduction')
    blocks = row.get('contact_live_blocks')
    if not isinstance(blocks, list) or len(blocks) != 4:
        raise RuntimeError('Four fresh live blocks required; no target reduction')
    previous_end = row['started_monotonic']
    for block, (address, count), field in zip(blocks, BLOCKS,
            ('targets', 'current', 'positions', 'status')):
        if (block.get('address') != address or block.get('count') != count
                or len(block.get('raw_values', [])) != count
                or block['raw_values'][:6] != row.get(field)
                or any(type(v) is not int or not 0 <= v <= 65535 for v in block['raw_values'])
                or not previous_end <= block['started_monotonic'] <= block['finished_monotonic']
                or block['finished_monotonic'] > row['finished_monotonic']):
            raise RuntimeError('Contact live block differs from its raw source')
        previous_end = block['finished_monotonic']


class ContactTelemetryAuthority(LoadedContactAuthority):
    def __init__(self):
        super().__init__()
        self.contact_layout_probed = False
        self.contact_identity_anchor = None
        self.contact_close_ack_monotonic = None

    def observe_first_close_feedback(self, snapshot):
        validate_contact_source(self, snapshot)
        return super().observe_first_close_feedback(snapshot)


class ContactTelemetryTransport(LoadedCycleTransport):
    def _validate(self, message):
        if isinstance(message, dict) and message.get('command') == MULTI and (
                message.get('address'), message.get('num')) in EXTRA_READS:
            if (set(message) != {'command', 'port', 'address', 'num', 'device'}
                    or any(type(message.get(k)) is not int for k in ('port','address','num','device'))
                    or message['port'] != 1 or message['device'] != 2):
                raise ValueError('Only exact hand telemetry reads allowed')
            return None
        return super()._validate(message)

    def read(self, address, count):
        if (address, count) not in EXTRA_READS:
            return super().read(address, count)
        if type(address) is not int or type(count) is not int:
            self.latch_fault('Invalid telemetry read types')
            raise ValueError('Integer telemetry address/count required')
        if self.previous == (MULTI, count):
            if self.read(1000, 8) != IDENTITY:
                self.latch_fault('Telemetry identity fence mismatch')
                raise RuntimeError('Telemetry identity fence mismatch')
        return self.exchange(dict(command=MULTI, port=1, address=address, num=count, device=2))

    def set_phase(self, phase, authority, approved_values=None, snapshot=None):
        if phase == 'close' and not authority.contact_layout_probed:
            self.latch_fault('Read-only telemetry layout not confirmed before close')
            raise RuntimeError('Telemetry layout must pass before any close')
        if phase == FREEZE_PHASE:
            validate_contact_source(authority, snapshot)
        return super().set_phase(phase, authority, approved_values=approved_values, snapshot=snapshot)

    def exchange(self, message):
        value = super().exchange(message)
        if (message.get('command') == WRITE and getattr(self, '_cycle_phase', None) == 'close'
                and FIRST_CLOSE_KEY in self._phase_authority._acknowledged_writes):
            self._phase_authority.contact_close_ack_monotonic = time.monotonic()
        return value


class ContactTelemetryHand(TrialHand):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.full_feedback_only = False

    def read_feedback_group(self):
        authority = self.sequence.authority
        if (self.full_feedback_only or FIRST_CLOSE_KEY not in authority._acknowledged_writes):
            row = super().read_feedback_group()
            if FIRST_CLOSE_KEY not in authority._acknowledged_writes:
                # Base snapshot still validates this group and arm/current/status
                # before super.close_trial can authorize the close.
                authority.contact_identity_anchor = copy.deepcopy(row)
            return row
        began = self.clock()
        blocks = []
        self.report['contact_feedback_read_in_progress'] = dict(
            started_monotonic=began, completed=False, completed_blocks=[],
            last_snapshot_may_precede_this_close=not self.report.get('contact_feedback_after_close_available', False))
        self.save()
        for address, count in BLOCKS:
            started = self.clock()
            raw = self.read('live_contact_block', address, count)
            self._u16(raw, count, 'contact block')
            blocks.append(dict(address=address, count=count, raw_values=raw[:],
                started_monotonic=started, finished_monotonic=self.clock()))
            self.report['contact_feedback_read_in_progress']['completed_blocks'] = copy.deepcopy(blocks)
            self.save()
        row = dict(identity=authority.contact_identity_anchor['identity'][:],
            identity_source='validated_full_group_immediately_before_close',
            identity_read_this_group=False,
            identity_source_finished_monotonic=authority.contact_identity_anchor['finished_monotonic'],
            targets=blocks[0]['raw_values'][:6], current=blocks[1]['raw_values'][:6],
            positions=blocks[2]['raw_values'][:6], status=blocks[3]['raw_values'][:6],
            contact_live_blocks=blocks, reads_are_sequential=True,
            started_monotonic=began, finished_monotonic=self.clock(),
            final_status_started_monotonic=blocks[-1]['started_monotonic'], status_is_last_group_read=True,
            physical_stop_verified=False, torque_release_verified=False)
        validate_contact_source(authority, row)
        self.report['contact_feedback_read_in_progress'].update(completed=True,
            finished_monotonic=row['finished_monotonic'])
        self.report['contact_feedback_after_close_available'] = True
        self.report['feedback_policy'].update(contact_read_order=list(BLOCKS),
            contact_identity_source=row['identity_source'], contact_identity_read_each_group=False,
            all_six_targets_currents_positions_statuses_live=True,
            contact_group_host_idle_seconds=1.2, group_deadline_seconds=3,
            group_deadline_is_not_physical_stop_guarantee=True,
            reserved_tails_logged_not_interpreted=True)
        return row

    def probe_layout(self):
        authority = self.sequence.authority
        if authority.contact_layout_probed:
            return
        if FIRST_CLOSE_KEY in authority._attempted_writes:
            raise RuntimeError('Telemetry layout cannot be probed after a close attempt')
        if not self._configuration_checked:
            self.configuration()
        self._critical_configuration()
        before = self.snapshot()
        blocks = []
        for address, count in BLOCKS[:3]:
            started = self.clock()
            raw = self.read('readonly_contact_layout_probe', address, count)
            self._u16(raw, count, 'telemetry layout')
            blocks.append(dict(address=address, count=count, raw_values=raw[:],
                started_monotonic=started, finished_monotonic=self.clock()))
        after = self.snapshot()
        self._at_expected(before, [8000,8000,8000,8000,65535,65535])
        self._at_expected(after, [8000,8000,8000,8000,65535,65535])
        if (before['identity'] != IDENTITY or after['identity'] != IDENTITY
                or any(abs(a-b)>TARGET_TOL for a,b in zip(blocks[0]['raw_values'][:6], after['targets']))
                or any(v>limit for v,limit in zip(blocks[1]['raw_values'][:6], self.report['configuration']['current_limit_ma']))
                or any(abs(a-b)>TOL for a,b in zip(blocks[2]['raw_values'][:6], after['positions']))):
            raise RuntimeError('Extended live block differs from fresh baseline; no close')
        self.guard()
        authority.contact_layout_probed = True
        self.report['contact_telemetry_layout_probe'] = dict(before=before, blocks=blocks, after=after,
            passed=True, actuator_writes_for_probe=0, reserved_tail_values_ignored=True)
        self.save()

    def close_trial(self):
        # Same physical setup and g permission; the compatibility probe is
        # read-only and happens BEFORE the first potentially loaded write.
        self.probe_layout()
        self.report['contact_feedback_after_close_available'] = False
        self.save()
        last = super().close_trial()
        self.full_feedback_only = True
        # super.close_trial just rechecked full critical configuration; do not
        # repeat those queries while holding. This group re-reads identity.
        final = self.snapshot()
        self.check_targets(final, self.sequence.load_targets)
        _dynamic_bounds(final, self.sequence.load_targets, last, reached=True)
        self.sequence.load_anchor = copy.deepcopy(final)
        self.report['load_anchor'] = copy.deepcopy(final)
        self.report['contact_full_feedback_after_settling'] = dict(snapshot=copy.deepcopy(final),
            identity_read_this_group=True, passed=True, required_before_any_lift=True)
        self.save()
        return final
