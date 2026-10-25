"""Fresh CRLF JSON transactions for four reviewed arm protection commands.

API4.3.1's self-collision getter can reuse an unconsumed cached reply. This
channel never calls that getter, edits native memory, or retries a setting.
Each transaction has its own socket. Motion commands are not permitted.
"""
import json
import select
import socket
import time

ENDPOINT = ('169.254.128.18', 8080)
READS = {'get_collision_stage', 'get_self_collision_enable'}
SETTINGS = {
    'set_collision_stage': {'command': 'set_collision_stage', 'collision_stage': 8},
    'set_self_collision_enable': {'command': 'set_self_collision_enable', 'set_enable': True},
}
METHODS = {'Get_Collision_Stage', 'Get_Self_Collision_Enable',
           'Set_Collision_Stage', 'Set_Self_Collision_Enable'}


class ProtectionError(RuntimeError):
    pass


def decode_reply(request, response):
    """Require the documented command/state discriminator and exact value types."""
    command = request['command']
    if not isinstance(response, dict):
        raise ProtectionError('Protection response is not an object')
    # The documented collision GET uses state; setters and self GET use command.
    field = 'state' if command == 'get_collision_stage' else 'command'
    if response.get(field) != command:
        raise ProtectionError('Protection response identifier mismatch: ' + repr(response))
    other = 'command' if field == 'state' else 'state'
    if other in response and response[other] != command:
        raise ProtectionError('Conflicting protection response identifiers')
    if command == 'get_collision_stage':
        value = response.get('collision_stage')
        if type(value) is not int or not 0 <= value <= 8:
            raise ProtectionError('Invalid collision stage: ' + repr(response))
        return [0, value]
    if command == 'get_self_collision_enable':
        value = response.get('enable_state')
        if type(value) is not bool:
            raise ProtectionError('Invalid self-collision enable state: ' + repr(response))
        return [0, value]
    key = 'collision_state' if command == 'set_collision_stage' else 'set_state'
    if response.get(key) is not True:
        raise ProtectionError('Controller rejected/uncertain protection setting: ' + repr(response))
    return 0


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ProtectionError('Duplicate JSON field: ' + key)
        result[key] = value
    return result


class ProtectionClient:
    def __init__(self, log, socket_factory=None, select_fn=None, clock=None):
        self.log = log
        self.socket_factory = socket_factory or socket.create_connection
        self.select = select_fn or select.select
        self.clock = clock or time.monotonic
        self.sequence = 0
        self.broken = False
        self.settings_enabled = False
        self.attempted_settings = set()

    def authorize_settings(self):
        if self.broken:
            raise ProtectionError('Failed protection channel cannot authorize settings')
        self.settings_enabled = True

    def exchange(self, request):
        command = request.get('command')
        if command in READS:
            if request != {'command': command}:
                raise ValueError('Unexpected protection GET fields')
        elif command in SETTINGS:
            expected = SETTINGS[command]
            if (not self.settings_enabled or request != expected
                    or any(type(request[k]) is not type(v) for k, v in expected.items())):
                raise ValueError('Only explicitly authorized stage8/selfTrue settings allowed')
            if command in self.attempted_settings:
                raise ProtectionError('Protection setting already attempted; no retry')
            self.attempted_settings.add(command)
        else:
            raise ValueError('No motion, disable, reset, mode, frame or payload commands allowed')
        if self.broken:
            raise ProtectionError('Protection channel latched failed; no retry')
        self.sequence += 1
        sock = None
        buffer = b''
        begin = self.clock()
        self.log(dict(event='transaction_begin', sequence=self.sequence,
                      monotonic=begin, request=request, fresh_socket=True))
        try:
            sock = self.socket_factory(ENDPOINT, timeout=3)
            self.log(dict(event='connected', sequence=self.sequence, endpoint=list(ENDPOINT),
                          monotonic=self.clock()))
            # Exclusive control plus a quiet fresh connection is required;
            # this protocol has no transaction ID. Never swallow broadcasts.
            if self.select([sock], [], [], .3)[0]:
                sock.settimeout(3)
                buffer = sock.recv(4096)
                self.log(dict(event='rx_before_tx', sequence=self.sequence,
                              monotonic=self.clock(), hex=buffer.hex()))
                raise ProtectionError('Unsolicited response on fresh protection connection')
            payload = (json.dumps(request, separators=(',', ':')) + '\r\n').encode('ascii')
            self.log(dict(event='tx', sequence=self.sequence, monotonic=self.clock(),
                          request=request, hex=payload.hex()))
            sock.sendall(payload)
            deadline = self.clock() + 3
            while b'\n' not in buffer:
                left = deadline - self.clock()
                if left <= 0:
                    raise ProtectionError('Protection response deadline exceeded')
                sock.settimeout(left)
                chunk = sock.recv(4096)
                self.log(dict(event='rx', sequence=self.sequence,
                              monotonic=self.clock(), hex=chunk.hex()))
                if not chunk:
                    raise ProtectionError('Protection connection closed before reply')
                buffer += chunk
                if len(buffer) > 65536:
                    raise ProtectionError('Oversized protection response')
            line, extra = buffer.split(b'\n', 1)
            if extra:
                raise ProtectionError('Duplicate/extra protection response')
            response = json.loads(line.decode('utf-8'), object_pairs_hook=unique_object,
                                  parse_constant=lambda v: (_ for _ in ()).throw(
                                      ProtectionError('Invalid JSON constant: ' + v)))
            value = decode_reply(request, response)
            if self.select([sock], [], [], .05)[0]:
                extra = sock.recv(4096)
                self.log(dict(event='rx_after_reply', sequence=self.sequence,
                              monotonic=self.clock(), hex=extra.hex()))
                if extra:
                    raise ProtectionError('Additional protection response; exclusive control uncertain')
            self.log(dict(event='response', sequence=self.sequence, monotonic=self.clock(),
                          elapsed_s=self.clock()-begin, response=response, value=value))
            return value
        except BaseException as exc:
            self.broken = True
            self.log(dict(event='transport_error', sequence=self.sequence,
                          monotonic=self.clock(), error=repr(exc), pending_request=request,
                          buffer_hex=buffer.hex(), retry=False))
            raise
        finally:
            if sock is not None:
                try:
                    sock.close()
                    self.log(dict(event='closed', sequence=self.sequence, monotonic=self.clock()))
                except BaseException as exc:
                    self.broken = True
                    self.log(dict(event='close_error', sequence=self.sequence, error=repr(exc)))
                    raise

    def Get_Collision_Stage(self):
        return self.exchange({'command': 'get_collision_stage'})

    def Get_Self_Collision_Enable(self):
        return self.exchange({'command': 'get_self_collision_enable'})

    def Set_Collision_Stage(self, stage, block=True):
        if type(stage) is not int or stage != 8 or block is not True:
            raise ValueError('Only acknowledged collision stage8 allowed')
        return self.exchange(dict(SETTINGS['set_collision_stage']))

    def Set_Self_Collision_Enable(self, enable):
        if enable is not True:
            raise ValueError('Self-collision disable is forbidden')
        return self.exchange(dict(SETTINGS['set_self_collision_enable']))
