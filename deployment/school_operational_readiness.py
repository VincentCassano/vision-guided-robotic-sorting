"""Onsite current operational-read gate, not polling for a diagnostic to clear."""
import json
import socket
import time

from school_battery_transport import MULTI, SINGLE, TransportError
from supported_contact_transport import SupportedContactTransport, PhaseAuthority
from school_device_identity import identity_fields, same_live_identity

BLOCKS = {(1000,8), (1095,6), (1085,6), (1135,6), (1145,6),
          (1125,6), (1115,5), (1105,5)}


class OperationalReadTransport(SupportedContactTransport):
    def _validate(self, message):
        if not isinstance(message, dict):
            raise RuntimeError('Structured operational read required')
        if not ((message.get('command') == MULTI and
                (message.get('address'), message.get('num')) in BLOCKS)
                or (message.get('command') == SINGLE and message.get('address') == 1110)):
            raise RuntimeError('Operational startup gate has no write capability')
        return super()._validate(message)

    def enable_motion(self):
        raise RuntimeError('Operational startup gate cannot enable motion')

    def set_phase(self, *args, **kwargs):
        raise RuntimeError('Operational startup gate cannot authorize writes')

    def write_group(self, *args, **kwargs):
        raise RuntimeError('Operational startup gate cannot initialize/reset/write')


def wait_hand_ready(folder, report, save):
    began = time.monotonic()
    deadline = began + 45
    attempts = []
    report['phase'] = 'checking_current_hand_operational_reads'
    proof = dict(passed=False, deadline_seconds=45, attempts=attempts,
        no_actuator_writes=True, diagnostic_zero_required=False,
        no_claim_that_subexception1_is_stale=True,
        automatic_reset_or_start_init=False, normal_full_profile_still_required=True)
    report['startup_readiness'] = proof
    save()
    with (folder/'readiness_wire.jsonl').open('w', encoding='utf8') as wire:
        def log(row):
            wire.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n')
            wire.flush()

        while time.monotonic() < deadline:
            if (folder/'abort.json').exists():
                raise KeyboardInterrupt('Cancelled during read-only operational startup')
            if any(report.get(k, 0) for k in ('arm_actuator_writes','hand_register_writes',
                    'controller_configuration_writes','hand_bus_configuration_writes')):
                raise RuntimeError('Operational readiness cannot retry after any write')
            attempt = dict(number=len(attempts)+1, started_monotonic=time.monotonic())
            attempts.append(attempt)
            io = None
            try:
                io = OperationalReadTransport(socket.create_connection(('169.254.128.18',8080),timeout=3),
                    log, PhaseAuthority())
                identity = io.read(1000,8)
                identity_fields(identity)
                attempt['initial_identity_raw'] = identity
                # A successful read of the error register is not evidence of
                # current operational readiness. Read the actual work groups.
                values = {}
                for name, address, count in (
                        ('current_limit_ma',1095,6), ('speeds',1125,6),
                        ('force_targets',1115,5), ('targets',1135,6),
                        ('positions',1145,6), ('current_ma_0_to_4',1105,5)):
                    values[name] = io.read(address,count)
                    attempt['operational_reads'] = values
                values['current'] = values['current_ma_0_to_4']+[io.single(1110)]
                values['status'] = io.read(1085,6)
                if values['current_limit_ma'] != [1299]*6 or values['speeds'] != [65535]*6 or any(values['force_targets']):
                    raise RuntimeError('Current protection/speed/force mode differs; no write')
                if any(v not in range(6) for v in values['status']) or any(v in (3,4) for v in values['status']):
                    raise RuntimeError('Current protection/force/invalid status blocks startup; no retry')
                if any(c > lim for c,lim in zip(values['current'],values['current_limit_ma'])):
                    raise RuntimeError('Actual overcurrent during operational startup; no retry')
                final = io.read(1000,8)
                if not same_live_identity(final,identity):
                    raise RuntimeError('New initialization diagnostic during operational read group')
                if time.monotonic() >= deadline:
                    raise TimeoutError('Operational startup deadline elapsed; no write')
                attempt.update(final_identity_raw=final, outcome='work_registers_accepted_not_motion_certified')
                proof.update(passed=True, final_identity_raw=final,
                    initial_diagnostic=identity[6], final_diagnostic=final[6],
                    current_work_registers_readable=True, elapsed_seconds=time.monotonic()-began,
                    physical_stop_verified=False, torque_release_verified=False)
                save()
                return
            except TransportError as exc:
                attempt['transport_error'] = repr(exc)
                if str(exc) != 'Read rejected':
                    raise
                attempt['outcome'] = 'work_read_explicitly_rejected_no_write'
            finally:
                attempt['finished_monotonic'] = time.monotonic()
                save()
                if io is not None:
                    io.disarm_motion()
                    io.sock.close()
            time.sleep(min(1.0,max(0,deadline-time.monotonic())))
    proof['elapsed_seconds'] = time.monotonic()-began
    proof['failure_reason'] = 'Current work-register reads rejected through actual45s deadline; diagnostic alone was not used as readiness'
    save()
    raise RuntimeError('Actual hand work registers still unreadable after45s; no init/reset/motion sent. See exact rejected address in readiness_wire.jsonl.')
