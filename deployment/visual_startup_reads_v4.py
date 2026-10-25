"""Bounded read-only startup collection; no retry capability after permission."""
import hashlib
import json
from pathlib import Path
import socket
import time

from school_battery_transport import MULTI, SINGLE, TransportError
from school_operational_readiness import OperationalReadTransport
from school_device_identity import identity_fields, same_live_identity
from supported_contact_transport import PhaseAuthority

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / 'deployment/visual-pregrasp-motion-v4-files.sha256'
BLOCKS = frozenset({(1000,8),(1020,6),(1030,6),(1040,3),
    (1045,6),(1055,6),(1065,6),(1075,6),(1095,6),
    (1185,6),(1195,6),(1205,6),(1215,6),
    (1085,6),(1125,6),(1115,5),(1135,6),(1145,6),(1105,5)})


def verify_delivery():
    result = {}
    for line in MANIFEST.read_text(encoding='ascii').splitlines():
        digest, name = line.split('  ', 1)
        path = (ROOT/name).resolve()
        if (ROOT not in path.parents or name in result or len(digest) != 64
                or hashlib.sha256(path.read_bytes()).hexdigest() != digest):
            raise ValueError('Deployment file changed: ' + name)
        result[name] = digest
    if not result:
        raise ValueError('Empty deployment inventory')
    return result


class StartupReadSession:
    """Disposable read sockets have private fault latches, never write permits.

    Only a complete, explicit read_state:false response can start another
    read-only socket. The failed socket/token remains latched. A device status,
    new diagnostic, bad frame, timeout or any application fault ends the run.
    """
    def __init__(self, folder, report, save, log, authority, guard):
        self.folder, self.report, self.save, self.log = folder, report, save, log
        self._phase_authority, self.guard = authority, guard
        self.motion_enabled = False
        self.broken = False
        self.active = True
        self.current = None
        self.socket_number = 0
        self.last_request = None
        self.reply_bytes = b''
        self.last_guard_monotonic = 0
        self.started = time.monotonic()
        self.deadline = self.started + 45
        self.proof = dict(passed=False, deadline_seconds=45, sockets=[], rejected_reads=[],
            policy='explicit_read_refusal_only_before_any_permission_or_write',
            old_socket_and_private_token_remain_latched=True,
            configuration_and_three_feedback_groups_still_required=True,
            actuator_or_settings_retries_allowed=False, stop_performance_verified=False)
        report['full_startup_read_collection'] = self.proof
        report['phase'] = 'checking_full_hand_startup_reads'
        save()

    @property
    def terminal_latched(self):
        return self._phase_authority._terminal_latched

    def _guard_scope(self):
        if (not self.active or self.broken or self.terminal_latched
                or self._phase_authority._attempted_writes
                or self.report.get('operator_permissions')
                or self.report.get('permission_index') is not None
                or any(self.report.get(key,0) for key in
                    ('arm_actuator_writes','hand_register_writes','controller_configuration_writes'))):
            raise RuntimeError('Read-only startup window closed; no reconnect or retry')
        if (self.folder/'abort.json').exists():
            raise KeyboardInterrupt('Parent cancelled startup')
        if time.monotonic() >= self.deadline:
            raise TimeoutError('Full hand startup reads exceeded45s; no motion')
        now = time.monotonic()
        if now-self.last_guard_monotonic >= .25:
            self.guard()
            self.last_guard_monotonic = time.monotonic()

    def _new_socket(self):
        self._guard_scope()
        owner = self

        class ReadOnlySocket(OperationalReadTransport):
            def _validate(self, message):
                owner._guard_scope()
                if owner.deadline-time.monotonic() < 3.4:
                    raise TimeoutError('Insufficient startup time for complete3s transaction')
                if not ((message.get('command') == MULTI and
                         (message.get('address'),message.get('num')) in BLOCKS)
                        or (message.get('command') == SINGLE and message.get('address') == 1110)):
                    raise RuntimeError('Only reviewed full-profile/feedback reads allowed')
                # Bypass only the smaller operational-only address list.
                from supported_contact_transport import SupportedContactTransport
                return SupportedContactTransport._validate(self,message)

            def exchange(self, message):
                value = super().exchange(message)
                if message.get('command') == MULTI and (message.get('address'),message.get('num')) == (1000,8):
                    identity_fields(value)
                    previous = owner._phase_authority.accepted_identity_raw
                    if not same_live_identity(value,previous):
                        owner.disable_motion()
                        raise RuntimeError('New identity/diagnostic during startup; no retry')
                    owner._phase_authority.accepted_identity_raw = value[:]
                    owner.proof['latest_identity_raw'] = value[:]
                owner._guard_scope()
                return value

        self.socket_number += 1
        number = self.socket_number
        row = dict(number=number,started_monotonic=time.monotonic(),
                   motion_capability=False,private_token=True)
        self.proof['sockets'].append(row)
        sock = socket.create_connection(('169.254.128.18',8080),timeout=min(3,self.deadline-time.monotonic()))
        self.current = ReadOnlySocket(sock,
            lambda event:self._wire_event(event,number),PhaseAuthority())
        self.current.read(1000,8)

    def _wire_event(self, event, number):
        if event.get('event') == 'tx':
            self.last_request = dict(event['request'])
            self.reply_bytes = b''
        if event.get('event') == 'rx':
            self.reply_bytes += bytes.fromhex(event['hex'])
        self.log(dict(event,startup_read_socket=number))

    def _explicit_negative_reply(self):
        def pairs(items):
            value = {}
            for key, item in items:
                if key in value:
                    raise ValueError('Duplicate reply key')
                value[key] = item
            return value
        try:
            response = json.loads(self.reply_bytes,object_pairs_hook=pairs)
            return (self.last_request is not None and self.reply_bytes.endswith(b'\r\n')
                    and response == {'command':self.last_request['command'],'read_state':False}
                    and response['read_state'] is False)
        except (ValueError,KeyError,TypeError):
            return False

    def _close_socket(self):
        if self.current is not None:
            io,self.current = self.current,None
            io.disarm_motion()
            io.sock.close()
            self.proof['sockets'][-1].update(closed=True,broken=io.broken,
                private_token_latched=io.terminal_latched,finished_monotonic=time.monotonic())

    def _read(self, address, count=None):
        while True:
            self._guard_scope()
            try:
                if self.current is None:
                    self._new_socket()
                value = (self.current.single(address) if count is None
                         else self.current.read(address,count))
                self._guard_scope()
                return value
            except TransportError as exc:
                # No recovery for timeouts, malformed data, unsolicited frames,
                # write failures, status faults or a new identity/diagnostic.
                if str(exc) != 'Read rejected' or not self._explicit_negative_reply():
                    self.disable_motion()
                    raise
                failed = dict(address=address,count=count,error=repr(exc),
                    rejected_request=dict(self.last_request),
                    socket=self.socket_number,monotonic=time.monotonic(),
                    original_wire_preserved=True,request_not_executed_as_write=True)
                self.proof['rejected_reads'].append(failed)
                self._close_socket()
                self.save()
                self._guard_scope()
                time.sleep(min(1,max(0,self.deadline-time.monotonic())))
            except BaseException:
                self.disable_motion()
                raise

    def read(self, address, count):
        if type(address) is not int or type(count) is not int or (address,count) not in BLOCKS:
            self.disable_motion()
            raise ValueError('Unreviewed startup read')
        return self._read(address,count)

    def single(self, address):
        if type(address) is not int or address != 1110:
            self.disable_motion()
            raise ValueError('Unreviewed startup single read')
        return self._read(address)

    def disable_motion(self):
        self._phase_authority._terminal_latched = True
        self.active = False
        self.broken = True

    def latch_fault(self, reason):
        self.disable_motion()
        self.log(dict(event='startup_read_session_fault',reason=str(reason),
                      no_motion_or_write_capability=True))

    def disarm_motion(self):
        self.motion_enabled = False

    def enable_motion(self):
        raise RuntimeError('Startup read collector has no motion/write capability')

    def finish(self):
        self._guard_scope()
        self.proof.update(passed=True,elapsed_seconds=time.monotonic()-self.started,
                          scope='this_run_full_configuration_and_startup_observation_only')
        self.active = False
        self.save()

    def close(self):
        self.active = False
        self._close_socket()
        self.proof['elapsed_seconds'] = time.monotonic()-self.started
        self.save()
