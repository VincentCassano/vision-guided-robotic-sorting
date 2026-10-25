"""Normal bin release only; no protected recovery and no opposition rotation."""
import copy

from school_battery_transport import WRITE
from supported_contact_transport import WritePhase
from school_smallcap_reads import KnownReadTransport, KnownReadHand
from run_school_battery_hand import TOL, TARGET_TOL
from school_smallcap_hand import _dynamic_bounds
from school_smallcap_hand import MovingLoadedHand
from school_smallcap_contact_transport import FIRST_CLOSE_KEY

DROP_TARGET = [0, 0, 0, 0, 0, 65535]
DROP_PHASE = 'normal_selected_bin_release'


class BinDropTransport(KnownReadTransport):
    """Add exactly one five-finger opening, after this run's bin endpoint."""
    def set_phase(self, phase, authority, approved_values=None, snapshot=None):
        if phase != DROP_PHASE:
            return super().set_phase(phase, authority, approved_values, snapshot)
        permission = getattr(authority, 'bin_drop_permission', None)
        if (authority is not self._phase_authority or self.broken or self.terminal_latched
                or self.motion_enabled or self._phase is not WritePhase.READONLY
                or (DROP_PHASE, 0, (1135, tuple(DROP_TARGET))) in self._attempted_writes
                or FIRST_CLOSE_KEY not in authority._acknowledged_writes or self._completed
                or authority._terminal_latched or not authority.read_pacing_qualified
                or not getattr(authority, 'selected_bin_endpoint_qualified', False)
                or not isinstance(permission, dict) or permission.get('index') != 11
                or permission.get('selected_bin_normal_drop_authorized') is not True):
            self.latch_fault('Normal bin release lacks healthy endpoint/run permission')
            raise RuntimeError('Normal bin release refused; no recovery/fault opening')
        self._cycle_phase = DROP_PHASE
        self._phase = WritePhase.RELEASE
        self._phase_index = 0
        self.log(dict(event='normal_bin_drop_phase', permission=permission,
            values=DROP_TARGET, opposition_unchanged=True, protected_recovery=False))

    def _validate(self, message):
        if message.get('command') != WRITE or getattr(self, '_cycle_phase', None) != DROP_PHASE:
            return super()._validate(message)
        exact = dict(command=WRITE, port=1, address=1135, num=6,
            data=[byte for v in DROP_TARGET for byte in (v >> 8, v & 255)], device=2)
        authority = self._phase_authority
        key = (DROP_PHASE, 0, (1135, tuple(DROP_TARGET)))
        if (message != exact or any(type(message.get(k)) is not int
                for k in ('port', 'address', 'num', 'device'))
                or any(type(v) is not int for v in message.get('data', []))
                or not self.motion_enabled or self.broken or self.terminal_latched
                or authority._terminal_latched or self._completed or self._phase_index != 0
                or not getattr(authority, 'selected_bin_endpoint_qualified', False)
                or key in self._attempted_writes):
            raise ValueError('Only one exact healthy normal bin opening is allowed')
        return key


class BinDropHand(KnownReadHand):
    def open_at_bin(self, targets, anchor):
        self._critical_configuration()
        before = self.snapshot()
        _dynamic_bounds(before, targets, anchor, reached=True)
        self.guard()
        self.io.set_phase(DROP_PHASE, self.sequence.authority)
        self.io.enable_motion()
        self.sequence.hand_motion_enabled = True
        self.report['bin_opening_before'] = copy.deepcopy(before)
        self.report['bin_opening_targets'] = DROP_TARGET[:]
        self.save()
        self.io.write_group(1135, DROP_TARGET)
        began = self.clock()
        previous = None
        minima = before['positions'][:5]
        groups = 0
        quiet_since = None
        while self.clock() - began < 30:
            row = self.snapshot()
            if any(abs(a-b) > TARGET_TOL for a,b in zip(row['targets'], DROP_TARGET)):
                raise RuntimeError('Bin opening target changed outside this run')
            if abs(row['positions'][5]-before['positions'][5]) > TOL:
                raise RuntimeError('Unwritten opposition moved during bin opening')
            if any(not 0 <= p <= initial+TOL for p,initial in
                    zip(row['positions'][:5], before['positions'][:5])):
                raise RuntimeError('Bin opening left actual-start withdrawal bounds')
            if any(p > low+TOL for p,low in zip(row['positions'][:5], minima)):
                raise RuntimeError('Finger reclosed during normal bin opening')
            minima = [min(a,b) for a,b in zip(minima, row['positions'][:5])]
            reached = all(abs(a-b) <= TOL for a,b in zip(row['positions'], DROP_TARGET))
            quiet = previous is not None and all(abs(a-b) <= TOL
                for a,b in zip(previous['positions'], row['positions']))
            if reached and quiet:
                if quiet_since is None: quiet_since = self.clock()
                groups += 1
                if groups >= 3 and self.clock()-quiet_since >= 1:
                    self._critical_configuration()
                    self.report['bin_opening_feedback'] = copy.deepcopy(row)
                    self.report['bin_opening_feedback_completed'] = True
                    self.save()
                    return row
            else:
                quiet_since = None
                groups = 0
            previous = row
        self._disable()
        raise TimeoutError('Single bin opening unconfirmed; no retry or arm return')


class EmptyBinReturnHand(MovingLoadedHand):
    """Read-only monitor with the new normal-open anchor, not a loaded anchor."""
    def preflight_empty(self, targets):
        if targets != DROP_TARGET or self.io.motion_enabled:
            raise RuntimeError('Empty return needs the exact successful bin opening')
        self._critical_configuration()
        return self.monitor_loaded(targets,self.sequence.load_anchor)

    def monitor_loaded(self, targets, anchor):
        if self.io.motion_enabled or list(targets) != DROP_TARGET:
            raise RuntimeError('Empty return monitor has no hand-write capability')
        row = self.snapshot()
        if (any(abs(a-b)>TARGET_TOL for a,b in zip(row['targets'],DROP_TARGET))
                or any(abs(a-b)>TOL for a,b in zip(row['positions'],DROP_TARGET))
                or any(abs(a-b)>TOL for a,b in zip(row['positions'],anchor['positions']))):
            raise RuntimeError('Empty bin-return hand target/orientation drifted')
        self.report['last_empty_bin_return_monitor'] = copy.deepcopy(row)
        self.save()
        return row
