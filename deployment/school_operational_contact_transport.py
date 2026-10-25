"""Per-channel supported contact reduction with acknowledged-frame pacing.

A position freeze is neither a brake nor a holding-force guarantee. This
capability cannot survive an application/communication protection latch and
does not reset a hand fault. A separately authorized supported opening uses
the existing opening-only transport, never this close/freeze capability.
"""
import copy
import math
from school_device_identity import configuration_matches, same_live_identity

from school_battery_transport import WRITE
from supported_contact_transport import PhaseAuthority, WritePhase
from school_startup_cycle_transport import CycleTransport


FIRST_CLOSE = (1135, (15000, 30000, 28000, 20000, 65535, 65222))
FIRST_CLOSE_KEY = ('close', 0, FIRST_CLOSE)
FREEZE_PHASE = 'freeze_supported_contact'
IDENTITY = [512, 769, 151, 8193, 290, 2, 0, 0]
# This is the already frozen actual-device profile, not a new manufacturer
# default. The caller must also match its full freshly read configuration.
EXPECTED_CONFIGURATION = {
    'identity': IDENTITY,
    'calibration_end_absolute': [21539, 24972, 25993, 24169, 24866, 18946],
    'calibration_start_absolute': [3039, 2472, 3093, 2469, 2616, 2386],
    'thumb_presets_absolute': [2386, 18026, 18946],
    'pid_p': [25000] * 6,
    'pid_i': [200] * 6,
    'pid_d': [25000] * 6,
    'pid_g': [100] * 6,
    'current_limit_ma': [1299] * 6,
    'stall_speed': [16] * 6,
    'stall_current_ma': [200] * 6,
    'stall_stop_ms': [300] * 6,
    'stall_retry_ms': [500] * 6,
}


def _u16(values, length, name):
    if (not isinstance(values, (list, tuple)) or len(values) != length
            or any(type(v) is not int or not 0 <= v <= 65535 for v in values)):
        raise ValueError('Invalid supported-contact ' + name)


def _grasp_permission(authority):
    permission = getattr(authority, 'loaded_grasp_permission', None)
    if (not isinstance(permission, dict)
            or type(permission.get('index')) is not int
            or permission['index'] != 5 or permission.get('value') != 'g'
            or permission.get('only_intended_shell_contact_and_closure_clear') is not True
            or permission.get('freeze_supported_contact_while_shell_supported') is not True):
        raise RuntimeError('This run needs explicit g support/contact-freeze permission')
    return permission


def _complete_feedback(snapshot, configuration, targets):
    if not isinstance(snapshot, dict):
        raise ValueError('A complete current supported-contact group is required')
    if not configuration_matches(configuration, EXPECTED_CONFIGURATION):
        raise RuntimeError('Full actual hand configuration changed; no contact freeze')
    for name, count in (('identity', 8), ('status', 6), ('current', 6),
                        ('positions', 6), ('targets', 6), ('speeds', 6),
                        ('force_targets', 5), ('current_limit_ma', 6)):
        _u16(snapshot.get(name), count, name)
    if not same_live_identity(snapshot['identity'], configuration['identity']) or any(v not in range(6) for v in snapshot['status']):
        raise RuntimeError('Hand identity or status changed; no contact freeze')
    if (snapshot.get('reads_are_sequential') is not True
            or snapshot['speeds'] != [65535] * 6 or any(snapshot['force_targets'])
            or snapshot['current_limit_ma'] != [1299] * 6
            or any(c > limit for c, limit in zip(snapshot['current'], snapshot['current_limit_ma']))):
        raise RuntimeError('Live hand mode/current/configuration forbids contact freeze')
    start, end = snapshot.get('started_monotonic'), snapshot.get('finished_monotonic')
    if (type(start) not in (int, float) or type(end) not in (int, float)
            or not math.isfinite(start) or not math.isfinite(end) or end < start):
        raise ValueError('Complete feedback timing is required')
    status_start = snapshot.get('final_status_started_monotonic')
    if (snapshot.get('status_is_last_group_read') is not True
            or type(status_start) not in (int, float) or not math.isfinite(status_start)
            or not start <= status_start <= end):
        raise ValueError('Contact freeze requires the final status read of this complete group')
    _u16(targets, 6, 'acknowledged active targets')
    if any(abs(a - b) > 2 for a, b in zip(snapshot['targets'], targets)):
        raise RuntimeError('Targets differ from this run acknowledged close/freeze')
    if any(p > cap + 150 for p, cap in zip(snapshot['positions'], FIRST_CLOSE[1])):
        raise RuntimeError('Actual finger position escaped the acknowledged close sweep')


class LoadedContactAuthority(PhaseAuthority):
    """Run-owned contact provenance; no fault latch can be reopened here.

    The sequence supplies ``loaded_grasp_permission`` from this run's g event
    and ``loaded_configuration`` only after its full fresh profile check. The
    hand calls ``observe_first_close_feedback`` on complete groups after the
    first close ACK. Only status5 on an actively closing channel0..3 consumes
    that channel's single reduction; a held little/rotation status5 is not contact
    evidence. Any prior status3/4 remains a terminal fault; a later clear-looking
    group cannot reopen that capability.
    """
    def __init__(self):
        super().__init__()
        self.loaded_grasp_permission = None
        self.loaded_configuration = None
        self.first_close_feedback = None
        self.contact_freeze_values = None
        self.contact_freeze_permission = None
        self.contact_freeze_configuration = None
        self.active_targets = tuple(FIRST_CLOSE[1])
        self.frozen_channels = set()
        self.pending_freeze_channels = ()
        self.freeze_count = 0
        self.last_contact_freeze_key = None

    def observe_first_close_feedback(self, snapshot):
        if FIRST_CLOSE_KEY not in self._acknowledged_writes:
            return False
        if self._terminal_latched or self._completed:
            raise RuntimeError('Protected/completed close cannot acquire a contact freeze')
        _grasp_permission(self)
        _complete_feedback(snapshot, self.loaded_configuration, self.active_targets)
        if any(v in (3, 4) for v in snapshot['status']):
            self._terminal_latched = True
            raise RuntimeError('Protected close cannot acquire a contact freeze')
        channels = tuple(ch for ch in range(4) if snapshot['status'][ch] == 5
            and ch not in self.frozen_channels
            and snapshot['positions'][ch] < self.active_targets[ch] - 2)
        if not channels:
            return False
        # Each channel can lower its target once. Other channels retain their
        # already acknowledged target; they are never tightened by a freeze.
        self.first_close_feedback = copy.deepcopy(snapshot)
        self.pending_freeze_channels = channels
        return True


class LoadedCycleTransport(CycleTransport):
    """Close once; at most one target reduction per contacting channel0..3."""
    def __init__(self, sock, log, phase_authority):
        if not isinstance(phase_authority, LoadedContactAuthority):
            raise ValueError('Loaded cycle needs its run-owned LoadedContactAuthority')
        super().__init__(sock, log, phase_authority)

    def pre_send_quiet_seconds(self, message):
        # 023145: 7.355ms after the close ACK, the first read was rejected.
        # Revoke the unqualified zero-idle assumption. Keep the established
        # 300ms baseline; do not invent a new intermediate interval or retry
        # the failed transaction under load. Faster contact control is pending.
        return super().pre_send_quiet_seconds(message)

    def exchange(self, message):
        freezing = isinstance(message, dict) and message.get('command') == WRITE and getattr(self, '_cycle_phase', None) == FREEZE_PHASE
        result = super().exchange(message)
        if freezing:
            authority = self._phase_authority
            key = (FREEZE_PHASE, authority.freeze_count, (1135, authority.contact_freeze_values))
            if key not in authority._acknowledged_writes:
                self.latch_fault('Freeze completed without exact acknowledgement')
                raise RuntimeError('Missing contact freeze ACK')
            authority.last_contact_freeze_key = key
            authority.active_targets = authority.contact_freeze_values
            authority.frozen_channels.update(authority.pending_freeze_channels)
            authority.freeze_count += 1
            authority.pending_freeze_channels = ()
        return result

    def set_phase(self, phase, authority, approved_values=None, snapshot=None):
        if phase != FREEZE_PHASE:
            if approved_values is not None or snapshot is not None:
                self.latch_fault('Dynamic values are exclusive to contact freeze')
                raise ValueError('Unexpected dynamic cycle phase values')
            if phase == 'close':
                _grasp_permission(authority)
                if not configuration_matches(getattr(authority, 'loaded_configuration', None), EXPECTED_CONFIGURATION):
                    self.latch_fault('Full actual hand configuration required before close')
                    raise RuntimeError('Current loaded-cycle configuration required')
            if phase == 'release_to_prepare' and getattr(self, '_cycle_phase', None) == FREEZE_PHASE:
                freeze_key = authority.last_contact_freeze_key
                if self._phase_index != 1 or freeze_key not in authority._acknowledged_writes:
                    self.latch_fault('Unacknowledged contact freeze cannot advance phase')
                    raise RuntimeError('Contact freeze acknowledgement required')
            return super().set_phase(phase, authority)
        try:
            if (authority is not self._phase_authority or self.broken
                    or self.terminal_latched or self._completed
                    or getattr(self, '_cycle_phase', None) not in ('close', FREEZE_PHASE)
                    or getattr(authority, 'cycle_stage', None) != 1
                    or self._phase_index != 1
                    or FIRST_CLOSE_KEY not in authority._acknowledged_writes
                    or authority.freeze_count >= 4
                    or (getattr(self, '_cycle_phase', None) == FREEZE_PHASE
                        and authority.last_contact_freeze_key not in authority._acknowledged_writes)):
                raise RuntimeError('Contact freeze requires the live first acknowledged close')
            _grasp_permission(authority)
            _complete_feedback(snapshot, authority.loaded_configuration, authority.active_targets)
            first_group = copy.deepcopy(snapshot)
            immediate_status = first_group.pop('status_immediately_before_freeze', None)
            if authority.first_close_feedback is None or first_group != authority.first_close_feedback:
                raise RuntimeError('Only the recorded complete status5 contact feedback can reduce targets')
            status = snapshot['status']
            if 5 not in status[:4] or any(v in (3, 4) for v in status):
                raise RuntimeError('Only unprotected first-group status5 permits contact freeze')
            _u16(immediate_status, 6, 'status immediately before freeze')
            if (any(v not in range(6) for v in immediate_status)
                    or any(v in (3, 4) for v in immediate_status)):
                raise RuntimeError('Fresh pre-freeze status protection forbids contact freeze')
            if snapshot['targets'][4:] != [65535, 65222]:
                raise RuntimeError('Little/rotation targets must remain exactly 65535/65222')
            _u16(approved_values, 6, 'approved freeze target')
            channels = tuple(ch for ch in range(4) if snapshot['status'][ch] == 5
                and ch not in authority.frozen_channels
                and snapshot['positions'][ch] < authority.active_targets[ch] - 2)
            if not channels or channels != authority.pending_freeze_channels:
                raise RuntimeError('Only new contacting channels can reduce targets once')
            expected = list(authority.active_targets)
            for ch in channels:
                expected[ch] = min(snapshot['positions'][ch], expected[ch])
            if (list(approved_values) != expected
                    or any(a > b for a, b in zip(approved_values, authority.active_targets))
                    or any(value < 8000 for value in approved_values[:4])):
                raise RuntimeError('Contact freeze may only use current/old minimum targets')
            authority.contact_freeze_values = tuple(approved_values)
            authority.contact_freeze_permission = copy.deepcopy(authority.loaded_grasp_permission)
            authority.contact_freeze_configuration = copy.deepcopy(authority.loaded_configuration)
            self._cycle_phase = FREEZE_PHASE
            self._phase = WritePhase.CONTACT
            self._phase_index = 0
            self.log(dict(event='supported_contact_freeze_phase',
                phase=FREEZE_PHASE, values=approved_values,
                frozen_channels=list(channels), previous_frozen_channels=sorted(authority.frozen_channels),
                source_feedback=copy.deepcopy(snapshot),
                permission=copy.deepcopy(authority.loaded_grasp_permission),
                no_added_closure=True, no_force_or_protection_writes=True,
                physical_stop_verified=False, holding_force_verified=False))
        except BaseException as exc:
            self.latch_fault(repr(exc))
            raise

    def _validate(self, message):
        if message.get('command') != WRITE:
            return super()._validate(message)
        phase = getattr(self, '_cycle_phase', None)
        if phase == 'close':
            if self._phase_index != 0:
                raise RuntimeError('Loaded cycle forbids the additional school index close')
            _grasp_permission(self._phase_authority)
            return super()._validate(message)
        if phase != FREEZE_PHASE:
            return super()._validate(message)
        if (set(message) != {'command', 'port', 'address', 'num', 'data', 'device'}
                or type(message['port']) is not int or message['port'] != 1
                or type(message['device']) is not int or message['device'] != 2
                or type(message['address']) is not int or message['address'] != 1135
                or type(message['num']) is not int or message['num'] != 6
                or not isinstance(message['data'], list) or len(message['data']) != 12
                or any(type(v) is not int or not 0 <= v <= 255 for v in message['data'])
                or not self.motion_enabled or self.terminal_latched or self.broken
                or self._completed or self._phase_index != 0):
            raise ValueError('Invalid or unauthorized supported contact freeze')
        values = tuple(message['data'][i] * 256 + message['data'][i + 1]
                       for i in range(0, len(message['data']), 2))
        approved = self._phase_authority.contact_freeze_values
        if approved is None or values != approved:
            raise ValueError('Contact freeze differs from the approved exact target')
        _grasp_permission(self._phase_authority)
        if (self._phase_authority.loaded_grasp_permission != self._phase_authority.contact_freeze_permission
                or self._phase_authority.loaded_configuration != self._phase_authority.contact_freeze_configuration):
            raise RuntimeError('Contact freeze permission/configuration changed before write')
        signature = (1135, values)
        if (not self._phase_authority.pending_freeze_channels
                or self._phase_authority.freeze_count >= 4):
            raise RuntimeError('No fresh per-channel contact capability')
        key = (FREEZE_PHASE, self._phase_authority.freeze_count, signature)
        if key in self._attempted_writes:
            raise RuntimeError('Contact freeze already attempted; no retry')
        return key
