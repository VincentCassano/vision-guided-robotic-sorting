"""One exact controller-side terminal RTU setup; no hand/power/arm commands."""
import json
import select
import socket
import time
from arm_protection_transport import unique_object

REQUEST={'command':'set_modbus_mode','port':1,'baudrate':115200,'timeout':1}
ENDPOINT=('169.254.128.18',8080)


class HandBusStartup:
    def __init__(self,log):
        self.log=log;self.attempted=False

    def configure_once(self):
        if self.attempted:raise RuntimeError('Hand bus setup already attempted; no retry')
        self.attempted=True;sock=None;buffer=b'';began=time.monotonic()
        self.log(dict(event='transaction_begin',request=REQUEST,monotonic=began,
            controller_communication_only=True,hand_register_write=False,power_write=False))
        try:
            sock=socket.create_connection(ENDPOINT,timeout=3)
            if select.select([sock],[],[],.3)[0]:
                sock.settimeout(3);buffer=sock.recv(4096)
                self.log(dict(event='rx_before_tx',hex=buffer.hex()))
                raise RuntimeError('Unsolicited hand-bus response; no setup sent')
            payload=(json.dumps(REQUEST,separators=(',',':'))+'\r\n').encode('ascii')
            self.log(dict(event='tx',request=REQUEST,hex=payload.hex(),monotonic=time.monotonic()))
            sock.sendall(payload);deadline=time.monotonic()+3
            while b'\n' not in buffer:
                left=deadline-time.monotonic()
                if left<=0:raise TimeoutError('Hand bus setup response deadline; no retry')
                sock.settimeout(left);chunk=sock.recv(4096)
                self.log(dict(event='rx',hex=chunk.hex(),monotonic=time.monotonic()))
                if not chunk:raise OSError('Hand bus EOF before ACK')
                buffer+=chunk
                if len(buffer)>65536:raise ValueError('Oversized bus response')
            line,extra=buffer.split(b'\n',1)
            if extra:raise ValueError('Duplicate/extra bus response')
            response=json.loads(line.decode('utf8'),object_pairs_hook=unique_object,
                parse_constant=lambda v:(_ for _ in ()).throw(ValueError('Nonfinite bus JSON')))
            if (not isinstance(response,dict) or response.get('command')!='set_modbus_mode'
                    or response.get('set_state') is not True):
                raise RuntimeError('Hand bus setup rejected or uncertain: '+repr(response))
            if select.select([sock],[],[],.05)[0]:
                extra=sock.recv(4096);self.log(dict(event='rx_after_reply',hex=extra.hex()))
                if extra:raise ValueError('Additional bus response; exclusive control uncertain')
            self.log(dict(event='response',response=response,elapsed_s=time.monotonic()-began,
                acknowledged=True,identity_verified=False,physical_stop_verified=False))
            return response
        except BaseException as exc:
            self.log(dict(event='transport_error',error=repr(exc),pending_request=REQUEST,
                buffer_hex=buffer.hex(),retry=False))
            raise
        finally:
            if sock is not None:sock.close();self.log(dict(event='closed'))
