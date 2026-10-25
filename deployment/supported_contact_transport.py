"""Isolated strict transport for a supported, de-energized test-piece pilot.

The default quiet interval, identity fence, CRLF framing, deadlines and raw
capture are preserved from school_battery_transport.Transport. A loaded-contact
subclass can specialize idle pacing after its acknowledged close. This class can
send the fixed two-finger 200mN trial target or empty-hand force reset. That
trial value is not a certified holding force or an instantaneous safety limit.
No arm, speed, PID, current-limit, init, node or unreviewed pose writes exist.
"""
from enum import Enum
import json
import select
import time

from school_battery_transport import Transport, TransportError, MULTI, SINGLE, WRITE
from school_battery_transport import READ_BLOCKS as BASE_READ_BLOCKS


class WritePhase(Enum):
    READONLY = 'readonly'
    PREPARE_EMPTY = 'prepare_empty'
    RESET_EMPTY = 'reset_empty'
    CONTACT = 'contact'
    RELEASE = 'release'


# Known V2 force PID groups, read-only. No writable global allowlist is edited.
READ_BLOCKS = frozenset(BASE_READ_BLOCKS | {(1225,5),(1235,5),(1245,5),(1255,5)})
PREPARE = (1135, (8000,8000,8000,8000,65535,65535))
RESET_FORCE = (1016, (1,))
CONTACT_FORCE = (1115, (200,200,0,0,0))
ZERO_FORCE = (1115, (0,0,0,0,0))
RELEASE_STEPS = (ZERO_FORCE, (1135,(0,)), (1136,(0,0,0,0)), (1140,(728,)))
PHASE_WRITES = {
    WritePhase.READONLY: (),
    WritePhase.PREPARE_EMPTY: (PREPARE,),
    WritePhase.RESET_EMPTY: (RESET_FORCE,),
    WritePhase.CONTACT: (CONTACT_FORCE,),
    WritePhase.RELEASE: RELEASE_STEPS,
}
PHASE_NEXT = {
    WritePhase.READONLY: (WritePhase.PREPARE_EMPTY, WritePhase.RESET_EMPTY,
                         WritePhase.CONTACT, WritePhase.RELEASE),
    WritePhase.PREPARE_EMPTY: (WritePhase.RESET_EMPTY, WritePhase.RELEASE),
    WritePhase.RESET_EMPTY: (WritePhase.CONTACT, WritePhase.RELEASE),
    WritePhase.CONTACT: (WritePhase.RELEASE,),
    WritePhase.RELEASE: (WritePhase.READONLY,),
}


class PhaseAuthority:
    """Entry-owned pilot permit; survives deliberate TCP close/reopen gates."""
    def __init__(self):
        self._attempted_writes = set()
        self._acknowledged_writes = set()
        self._terminal_latched = False
        self._completed = False


class SupportedContactTransport(Transport):
    """Monotonic phases and one attempt per target, with an entry-held token.

    The capability token prevents accidental phase changes by hand helpers;
    it is a Python control boundary, not protection against malicious Python.
    Application faults latch writes but permit healthy read-only diagnostics.
    Framing/communication faults additionally latch broken, as in the baseline,
    and cannot be resynchronized by retries on this socket.
    """
    def __init__(self, sock, log, phase_authority):
        if not isinstance(phase_authority, PhaseAuthority):
            raise ValueError('The main entry must hold a PhaseAuthority token')
        super().__init__(sock, log)
        self._phase_authority = phase_authority
        self._phase = WritePhase.READONLY
        self._phase_index = 0
        self._attempted_writes = phase_authority._attempted_writes

    @property
    def terminal_latched(self):
        return self._phase_authority._terminal_latched

    @property
    def _completed(self):
        return self._phase_authority._completed

    @property
    def phase(self):
        return self._phase.value

    @property
    def scope_write(self):
        return self._phase

    def latch_fault(self, reason):
        self._phase_authority._terminal_latched = True
        self.motion_enabled = False
        self.log(dict(event='supported_contact_write_latched', phase=self.phase,
                      reason=str(reason), physical_stop_verified=False,
                      torque_release_verified=False))

    def disable_motion(self):
        self.latch_fault('Application disabled further motion; no automatic restart')

    def disarm_motion(self):
        """Normal TCP close only; never clears a run-level fault or provenance."""
        self.motion_enabled = False

    def set_phase(self, phase, authority):
        try:
            if authority is not self._phase_authority:
                raise ValueError('Only the main entry may change write phase')
            desired = phase if isinstance(phase, WritePhase) else WritePhase(phase)
            if self.terminal_latched or self.broken or self._completed:
                raise TransportError('Contact write phases are terminally latched')
            if desired not in PHASE_NEXT[self._phase]:
                raise ValueError('Unapproved or repeated phase transition')
            acknowledged = self._phase_authority._acknowledged_writes
            if desired is WritePhase.RESET_EMPTY and PREPARE not in acknowledged:
                raise ValueError('Empty-hand prepare must precede force reset')
            if desired is WritePhase.CONTACT and not {PREPARE,RESET_FORCE} <= acknowledged:
                raise ValueError('Empty-hand prepare and force reset must precede contact')
            if desired in (WritePhase.RESET_EMPTY, WritePhase.CONTACT, WritePhase.READONLY) and self._phase is not WritePhase.READONLY:
                if self._phase_index != len(PHASE_WRITES[self._phase]):
                    raise ValueError('Prior phase has not completed its fixed write sequence')
            previous = self.phase
            self._phase = desired
            self._phase_index = 0
            # A deliberate clean reconnect retains run-level command provenance
            # and only advances past acknowledged prefix entries, never retries.
            for signature in PHASE_WRITES[desired]:
                if signature not in acknowledged: break
                self._phase_index += 1
            if desired is WritePhase.READONLY:
                self.motion_enabled = False
                self._phase_authority._completed = True
            self.log(dict(event='supported_contact_phase', previous=previous,
                          phase=self.phase, scope='supported_deenergized_test_piece_only'))
        except BaseException as exc:
            self.latch_fault(repr(exc))
            raise

    def enable_motion(self):
        if (self.broken or self.terminal_latched or self._completed
                or self._phase is WritePhase.READONLY):
            self.latch_fault('Attempted motion authorization outside a live write phase')
            raise TransportError('This phase cannot authorize contact writes')
        self.motion_enabled = True

    def _validate(self, message):
        if not isinstance(message, dict):
            raise ValueError('A structured hand request is required')
        command = message.get('command')
        fields = ({'command','port','address','num','data','device'} if command == WRITE
                  else {'command','port','address','num','device'} if command == MULTI
                  else {'command','port','address','device'} if command == SINGLE else set())
        if set(message) != fields:
            raise ValueError('Only the exact reviewed hand request fields are allowed')
        if (type(message.get('port')) is not int or type(message.get('device')) is not int
                or type(message.get('address')) is not int
                or message.get('port') != 1 or message.get('device') != 2):
            raise ValueError('Only hand port 1, node 2 allowed')
        if command == WRITE:
            data = message.get('data')
            if (not self.motion_enabled or self.terminal_latched or self._completed
                    or type(message.get('num')) is not int or not isinstance(data,list)
                    or len(data) != 2*message['num']):
                raise ValueError('Write not authorized in this contact phase')
            if any(type(v) is not int or not 0 <= v <= 255 for v in data):
                raise ValueError('Invalid write bytes')
            values = tuple(data[i]*256+data[i+1] for i in range(0,len(data),2))
            signature = (message['address'],values)
            allowed = PHASE_WRITES[self._phase]
            if self._phase_index >= len(allowed) or signature != allowed[self._phase_index]:
                raise ValueError('Write outside the fixed ordered phase allowlist')
            if signature in self._attempted_writes:
                raise TransportError('A target attempt cannot be repeated')
            return signature
        if command == MULTI:
            if type(message.get('num')) is not int or (message['address'],message['num']) not in READ_BLOCKS:
                raise ValueError('Unapproved read')
        elif command == SINGLE:
            if message['address'] != 1110:
                raise ValueError('Unapproved single read')
        else:
            raise ValueError('No robot/configuration command is allowed')
        return None

    def exchange(self, message):
        signature = None
        try:
            signature = self._validate(message)
            if self.broken:
                raise TransportError('Transport latched failed; no retries or writes')
            if signature is not None:
                # Mark before send, including an uncertain send or ACK; never
                # turn an exception into a retransmission of the same target.
                self._attempted_writes.add(signature)
        except BaseException as exc:
            self.latch_fault(repr(exc))
            raise
        self.sequence += 1
        start = time.monotonic()
        try:
            # Default entries retain the baseline quiet interval. The loaded
            # contact entry may omit idle pacing between acknowledged frames;
            # it still rejects buffered/unsolicited/duplicate/invalid frames.
            if self.buffer: raise TransportError('Extra buffered response')
            quiet = self.pre_send_quiet_seconds(message)
            if select.select([self.sock],[],[],quiet)[0]:
                self.receive(); raise TransportError('Unsolicited or late response before send')
            payload = (json.dumps(message,separators=(',',':'))+'\r\n').encode('ascii')
            self.log(dict(event='tx',sequence=self.sequence,monotonic=time.monotonic(),request=message,
                          pre_send_quiet_seconds=quiet,hex=payload.hex()))
            self.sock.sendall(payload)
            deadline = time.monotonic()+3
            while b'\n' not in self.buffer:
                left = deadline-time.monotonic()
                if left <= 0: raise TransportError('Frame deadline exceeded')
                self.sock.settimeout(left); self.receive()
            line,self.buffer = self.buffer.split(b'\n',1)
            response = json.loads(line.decode('utf-8'))
            if self.buffer: raise TransportError('Duplicate/extra response in same receive')
            if not isinstance(response,dict) or response.get('command') != message['command']:
                raise TransportError('Response command mismatch')
            if message['command'] == WRITE:
                if response.get('write_state') is not True: raise TransportError('Write rejected or unacknowledged')
                value = None
            else:
                if response.get('read_state') is False: raise TransportError('Read rejected')
                data = response.get('data')
                if message['command'] == SINGLE:
                    if type(data) != int or not -32768 <= data <= 65535: raise TransportError('Invalid single register')
                    value = data & 65535
                else:
                    if not isinstance(data,list) or len(data) != 2*message['num'] or any(type(x) != int or not 0 <= x <= 255 for x in data):
                        raise TransportError('Wrong response byte count/type')
                    value = [data[i]*256+data[i+1] for i in range(0,len(data),2)]
            self.log(dict(event='response',sequence=self.sequence,monotonic=time.monotonic(),
                          elapsed_s=time.monotonic()-start,response=response,value=value))
            self.previous = (message['command'],message.get('num'))
            if signature is not None:
                self._phase_index += 1
                self._phase_authority._acknowledged_writes.add(signature)
            return value
        except BaseException as exc:
            self.broken = True
            self.log(dict(event='transport_error',sequence=self.sequence,error=repr(exc),
                          pending_request=message,buffer_hex=self.buffer.hex()))
            self.latch_fault(repr(exc))
            raise

    def pre_send_quiet_seconds(self, message):
        return .3

    def read(self, address, count):
        try:
            if (type(address) is not int or type(count) is not int
                    or (address,count) not in READ_BLOCKS):
                raise ValueError('Read not in allowlist')
            # Same equal-length identity fence as the strict baseline.
            if self.previous == (MULTI,count) and (address,count) != (1000,8):
                identity = self.read(1000,8)
                if identity[0] != 512 or identity[5] != 2:
                    raise TransportError('Identity fence mismatch')
            return self.exchange(dict(command=MULTI,port=1,address=address,num=count,device=2))
        except BaseException as exc:
            self.latch_fault(repr(exc))
            raise

    def single(self, address):
        if type(address) is not int or address != 1110:
            self.latch_fault('Unapproved single read')
            raise ValueError('Single read not in allowlist')
        return self.exchange(dict(command=SINGLE,port=1,address=address,device=2))

    def write_group(self, address, values):
        try:
            if (type(address) is not int or not isinstance(values,(list,tuple))
                    or any(type(v) is not int or not 0 <= v <= 65535 for v in values)):
                raise ValueError('Invalid position/force/reset values')
            return self.exchange(dict(command=WRITE,port=1,address=address,num=len(values),
                data=[b for v in values for b in (v>>8,v&255)],device=2))
        except BaseException as exc:
            self.latch_fault(repr(exc))
            raise
