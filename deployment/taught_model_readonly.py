"""Bounded, explicitly invoked controller model reads. No SDK or actuation.

Only the caller's explicit read mode may invoke collect_model_parameters. The
normal numerical planner does not import or call it. The two commands below
are documented for Gen 3; the installed controller's generation and DH-command
support remain unknown until its responses are collected. Returned DH data is
evidence only: it is never fitted, installed, or applied to a native model.
"""
import json
import math
from pathlib import Path
import platform
import socket
import time

ENDPOINT = ('169.254.128.18', 8080)
ALLOWED_COMMANDS = frozenset(('get_arm_software_info', 'get_DH_data'))
RESPONSE_COMMANDS = {
    'get_arm_software_info': 'arm_software_info',
    'get_DH_data': 'get_DH_data',
}
IDENTITY_RESPONSE_FIELDS = frozenset(('command', 'Product_version', 'algorithm_info',
    'ctrl_info', 'dynamic_info', 'plan_info', 'communication_info', 'program_info',
    'robot_controller_version'))
COMMAND_DEADLINE_SECONDS = 3.0
MAX_RESPONSE_BYTES = 65536
OFFICIAL_REFERENCES = {
    'identity': 'https://develop.realman-robotics.com/robot/json/systemConfig/',
    'dh': 'https://develop.realman-robotics.com/robot/json/armConfig/',
}


class ModelReadError(RuntimeError):
    pass


def _reject_constant(value):
    raise ModelReadError('Nonfinite JSON constant: ' + value)


def _finite_float(value):
    number = float(value)
    if not math.isfinite(number):
        raise ModelReadError('Nonfinite JSON numeric value')
    return number


def _valid_utf8_strings(value):
    if isinstance(value, str):
        value.encode('utf-8', errors='strict')
    elif isinstance(value, dict):
        for key, item in value.items():
            _valid_utf8_strings(key)
            _valid_utf8_strings(item)
    elif isinstance(value, list):
        for item in value:
            _valid_utf8_strings(item)


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ModelReadError('Duplicate JSON field: ' + key)
        result[key] = value
    return result


def request_payload(command):
    """A closed allowlist; cannot encode setters, motion or additional fields."""
    if type(command) is not str or command not in ALLOWED_COMMANDS:
        raise ModelReadError('Forbidden controller command')
    return (json.dumps({'command': command}, separators=(',', ':')) + '\r\n').encode('ascii')


class _Recorder:
    def __init__(self, path):
        self.path = path
        self.sequence = 0

    def write(self, event, command=None, **values):
        self.sequence += 1
        row = dict(sequence=self.sequence, monotonic_seconds=time.monotonic(),
                   event=event, command=command, **values)
        with self.path.open('a', encoding='utf-8', newline='\n') as stream:
            stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n')


def _query(command, connector, recorder, transaction):
    """One request, one fresh connection, one 3-second absolute deadline."""
    payload = request_payload(command)  # Validate before any connection.
    deadline = time.monotonic() + COMMAND_DEADLINE_SECONDS
    channel = None
    raw = bytearray()
    transaction.update(command=command, endpoint=list(ENDPOINT),
                       request_hex=payload.hex(), response_hex='',
                       connection_requested=False, request_send_completed=False,
                       socket_closed=False, response=None)

    def remaining():
        value = deadline - time.monotonic()
        if value <= 0:
            raise TimeoutError('Absolute command deadline exceeded')
        return value

    try:
        transaction['connection_requested'] = True
        recorder.write('connection_requested', command, endpoint=list(ENDPOINT))
        channel = connector(ENDPOINT, timeout=remaining())
        channel.settimeout(remaining())
        recorder.write('TX', command, hex=payload.hex(), bytes=len(payload),
                       stage='send_attempt')
        channel.sendall(payload)
        transaction['request_send_completed'] = True
        recorder.write('TX_completed', command, bytes=len(payload))
        while b'\r\n' not in raw:
            channel.settimeout(remaining())
            # A one-byte sentinel distinguishes exactly bounded vs oversized.
            part = channel.recv(min(4096, MAX_RESPONSE_BYTES - len(raw) + 1))
            recorder.write('RX', command, hex=part.hex(), bytes=len(part))
            raw.extend(part)
            transaction['response_hex'] = bytes(raw).hex()
            if len(raw) > MAX_RESPONSE_BYTES:
                raise ModelReadError('Response exceeds 65536-byte bound')
            if not part:
                raise ModelReadError('Connection closed before CRLF response')
        remaining()
        frame, trailing = bytes(raw).split(b'\r\n', 1)
        if trailing:
            raise ModelReadError('Unsolicited or trailing response bytes')
        response = json.loads(frame.decode('utf-8'), parse_constant=_reject_constant,
                              parse_float=_finite_float, object_pairs_hook=_unique_object)
        _valid_utf8_strings(response)
        transaction['response'] = response  # Preserve rejected schema/identity too.
        if type(response) is not dict:
            raise ModelReadError('Response must be a JSON object')
        if response.get('command') != RESPONSE_COMMANDS[command]:
            raise ModelReadError('Unexpected response command')
        for key in ('state', 'get_state', 'success'):
            if response.get(key) is False:
                raise ModelReadError('Controller rejected read command: ' + key)
        recorder.write('response_parsed', command)
        return response
    finally:
        transaction['response_hex'] = bytes(raw).hex()
        if channel is not None:
            try:
                channel.close()
                transaction['socket_closed'] = True
                recorder.write('socket_closed', command)
            except Exception as exc:
                transaction['close_error'] = type(exc).__name__ + ': ' + str(exc)
                recorder.write('socket_close_error', command, error=transaction['close_error'])
                raise ModelReadError('Socket close failed; no next read: ' + transaction['close_error'])


def validate_dh(response):
    """Accept only the documented six-joint integer schema; retain units."""
    expected = {'command'} | {'joint_' + str(index) for index in range(1, 7)}
    if type(response) is not dict or set(response) != expected:
        raise ModelReadError('Unknown or incomplete six-joint DH schema')
    if response['command'] != 'get_DH_data':
        raise ModelReadError('Unexpected DH response command')
    converted = []
    for index in range(1, 7):
        row = response['joint_' + str(index)]
        if type(row) is not list or len(row) != 4 or any(type(item) is not int for item in row):
            raise ModelReadError('DH row must contain exactly four integers: joint_' + str(index))
        converted.append(dict(joint=index, raw=list(row), alpha_degrees=row[0] / 1000.0,
                              a_meters=row[1] / 1000000.0, d_meters=row[2] / 1000000.0,
                              offset_degrees=row[3] / 1000.0))
    return converted


def collect_model_parameters(folder, connector=socket.create_connection):
    """Return and persist success or unknown evidence; never apply the data.

    The host check is repeated here so direct invocation cannot bypass the
    entry point. Tests pass an explicit fake connector and patch host identity.
    Identity failure prevents the second command. There is no fallback/retry.
    """
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    recorder = _Recorder(folder / 'model_reads.jsonl')
    report = dict(schema_version=1, endpoint=list(ENDPOINT),
                  network_connection_requested=False, hardware_writes=0,
                  controller_configuration_writes=0, actuator_writes=0,
                  execution_ready=False, model_data_applied=False,
                  fitted_correction_applied=False, dh_valid=False,
                  identity_valid=False, installed_controller_generation='unknown',
                  dh_command_generation_support='documented_for_Gen3_actual_support_unknown',
                  parameters_origin='unknown_until_controller_response',
                  official_references=dict(OFFICIAL_REFERENCES),
                  units=dict(alpha='1000 raw units per degree',
                             a='1000000 raw units per meter',
                             d='1000000 raw units per meter',
                             offset='1000 raw units per degree'),
                  transactions=[], errors=[])
    started = time.monotonic()
    command = None
    try:
        report['host'] = dict(system=platform.system(), machine=platform.machine())
        if report['host'] != {'system': 'Linux', 'machine': 'aarch64'}:
            raise ModelReadError('Read mode only allowed on onsite Linux aarch64 host')
        command = 'get_arm_software_info'
        report['transactions'].append({})
        identity = _query(command, connector, recorder, report['transactions'][-1])
        report['identity_response'] = identity
        if not set(identity).issubset(IDENTITY_RESPONSE_FIELDS):
            raise ModelReadError('Unknown software identity response schema; no DH read')
        if identity.get('Product_version') != 'RM65-BI':
            raise ModelReadError('Identity must be exact Product_version RM65-BI; no DH read')
        report['identity_valid'] = True
        command = 'get_DH_data'
        report['transactions'].append({})
        response = _query(command, connector, recorder, report['transactions'][-1])
        report['dh_response'] = response
        report['dh_parameters'] = validate_dh(response)
        report['dh_valid'] = True
        report['dh_command_generation_support'] = 'this_controller_returned_documented_schema'
        report['parameters_origin'] = 'controller_readback_not_manufacturer_default'
    except (Exception, KeyboardInterrupt) as exc:
        error = dict(command=command, type=type(exc).__name__, message=str(exc))
        report['errors'].append(error)
        recorder.write('read_error', command, error=error)
    finally:
        report['network_connection_requested'] = any(
            row.get('connection_requested', False) for row in report['transactions'])
        report['elapsed_seconds'] = time.monotonic() - started
        report['outcome'] = ('readback_recorded_not_applied' if report['dh_valid']
                             else 'model_parameters_unknown_no_fallback')
        recorder.write('finished', dh_valid=report['dh_valid'],
                       hardware_writes=0, execution_ready=False)
        (folder / 'model_parameters.json').write_text(
            json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + '\n',
            encoding='utf-8')
    return report
