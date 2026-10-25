"""Bounded read-only startup gate; never resets, initializes or moves hardware."""
import json
import socket
import time

from school_battery_transport import MULTI, TransportError
from supported_contact_transport import SupportedContactTransport, PhaseAuthority

IDENTITY = [512, 769, 151, 8193, 290, 2, 0, 0]


class ReadinessTransport(SupportedContactTransport):
    def _validate(self, message):
        if (not isinstance(message, dict) or message.get('command') != MULTI
                or (message.get('address'), message.get('num')) not in
                    {(1000, 8), (1095, 6), (1085, 6)}):
            raise RuntimeError('Readiness gate permits only three read-only register groups')
        return super()._validate(message)

    def enable_motion(self):
        raise RuntimeError('Readiness gate has no motion capability')

    def set_phase(self, *args, **kwargs):
        raise RuntimeError('Readiness gate cannot authorize a write phase')

    def write_group(self, *args, **kwargs):
        raise RuntimeError('Readiness gate cannot write targets or initialization registers')


def wait_hand_ready(folder, report, save):
    """Retry only startup reads, before the normal authority or any user gate.

    A rejected socket remains faulted and is closed. A new read-only object
    grants no old capability and never retries a write. The normal worker's
    full profile, hand observation and arm preflight still run afterward.
    """
    began = time.monotonic()
    deadline = began + 45
    attempts = []
    report['startup_readiness'] = dict(passed=False, deadline_seconds=45,
        attempts=attempts, no_actuator_writes=True, natural_initialization_only=True,
        automatic_reset_or_start_init=False, normal_preflight_still_required=True)
    report['phase'] = 'waiting_hand_startup_ready'
    save()
    with (folder/'readiness_wire.jsonl').open('w', encoding='utf8') as wire:
        def log(row):
            wire.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n')
            wire.flush()

        while time.monotonic() < deadline and len(attempts) < 20:
            if (folder/'abort.json').exists():
                raise KeyboardInterrupt('Cancelled during read-only startup readiness')
            if any(report.get(k, 0) for k in ('arm_actuator_writes', 'hand_register_writes',
                    'controller_configuration_writes', 'hand_bus_configuration_writes')):
                raise RuntimeError('Readiness waiting is forbidden after any actuator/configuration write')
            row = dict(attempt=len(attempts)+1, started_monotonic=time.monotonic())
            attempts.append(row)
            io = None
            try:
                io = ReadinessTransport(socket.create_connection(('169.254.128.18', 8080), timeout=3),
                    log, PhaseAuthority())
                identity = io.read(1000, 8)
                row['identity'] = identity
                if identity[:6] != IDENTITY[:6] or len(identity) != 8 or identity[7] != 0:
                    raise RuntimeError('Actual device identity/diagnostic register differs: '+str(identity))
                if identity[6] == 1:
                    row['waiting_reason'] = 'ROH_SUB_EXCEPTION=1 initialization-not-ready diagnostic; no write'
                elif identity[6] != 0:
                    raise RuntimeError('Nonzero startup sub-exception blocks operation: '+str(identity))
                else:
                    limits = io.read(1095, 6)
                    row['current_limit_ma'] = limits
                    if limits != [1299]*6:
                        raise RuntimeError('Actual current limits differ; no write')
                    status = io.read(1085, 6)
                    row['status'] = status
                    if len(status) != 6 or any(type(v) is not int or v not in range(6) for v in status):
                        raise RuntimeError('Invalid startup status; no write')
                    if any(v in (3, 4) for v in status):
                        raise RuntimeError('Actual protection/force status blocks startup; no retry')
                    final_identity = io.read(1000, 8)
                    row['final_identity'] = final_identity
                    if final_identity == IDENTITY:
                        if time.monotonic() >= deadline:
                            raise TimeoutError('Readiness deadline elapsed; no write')
                        row.update(outcome='complete_current_readonly_groups_accepted',
                            finished_monotonic=time.monotonic())
                        report['startup_readiness'].update(passed=True,
                            elapsed_seconds=time.monotonic()-began,
                            physical_stop_verified=False, torque_release_verified=False)
                        save()
                        return
                    if final_identity[:6] == IDENTITY[:6] and final_identity[6:] == [1, 0]:
                        row['waiting_reason'] = 'Initialization-not-ready diagnostic returned during startup reads'
                    else:
                        raise RuntimeError('Identity/diagnostic changed during readiness: '+str(final_identity))
            except TransportError as exc:
                row['transport_error'] = repr(exc)
                if str(exc) != 'Read rejected':
                    raise
                row['waiting_reason'] = 'Read-only startup request explicitly rejected; socket retired, no write'
            finally:
                row['finished_monotonic'] = time.monotonic()
                save()
                if io is not None:
                    io.disarm_motion()
                    io.sock.close()
            # This wait never runs during a grasp, after a permission, or
            # after a normal capability has latched a fault.
            time.sleep(min(1.0, max(0, deadline-time.monotonic())))
    report['startup_readiness']['elapsed_seconds'] = time.monotonic()-began
    save()
    raise RuntimeError('Hand startup not ready within45s; no reset/init/motion sent. Wait for self-test to finish before running again; retained diagnostic may need onsite investigation.')
