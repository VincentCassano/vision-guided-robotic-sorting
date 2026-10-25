"""Four exact fresh controller GETs; tip units from vendor JSON, no setters."""
import json
import socket
import time
from arm_protection_transport import unique_object

FIELDS = {'get_arm_max_line_speed': ('arm_max_line_speed','arm_line_speed','line_speed_m_s'),
          'get_arm_max_line_acc': ('arm_max_line_acc','arm_line_acc','line_acc_m_s2'),
          'get_arm_max_angular_speed': ('arm_max_angular_speed','arm_angular_speed','angular_speed_rad_s'),
          'get_arm_max_angular_acc': ('arm_max_angular_acc','arm_angular_acc','angular_acc_rad_s2')}


def decode(command, response):
    if command not in FIELDS:
        raise ValueError('Only four tip parameter GETs permitted')
    state,key,name=FIELDS[command]
    if (not isinstance(response,dict) or response.get('state')!=state
            or set(response)!={'state',key} or type(response[key]) is not int or response[key]<=0):
        raise ValueError('Unsupported tip response/unit/schema: '+repr(response))
    return name,response[key]/1000.


def collect(log, connector=socket.create_connection, clock=time.monotonic):
    values={}
    for command in FIELDS:
        sock=None;buffer=b'';deadline=clock()+3
        request={'command':command}
        def remaining():
            left=deadline-clock()
            if left<=0:raise TimeoutError('Tip GET absolute deadline')
            return left
        try:
            sock=connector(('169.254.128.18',8080),timeout=remaining())
            payload=(json.dumps(request,separators=(',',':'))+'\r\n').encode('ascii')
            log(dict(event='tx',request=request,hex=payload.hex()))
            sock.settimeout(remaining());sock.sendall(payload)
            while b'\r\n' not in buffer:
                sock.settimeout(remaining());part=sock.recv(4096)
                log(dict(event='rx',command=command,hex=part.hex()))
                if not part:raise OSError('Tip GET EOF')
                buffer+=part
                if len(buffer)>65536:raise ValueError('Tip reply oversized')
            remaining()
            line,extra=buffer.split(b'\r\n',1)
            if extra:raise ValueError('Trailing/duplicate tip reply')
            response=json.loads(line.decode('utf8'),object_pairs_hook=unique_object,
                parse_constant=lambda v:(_ for _ in ()).throw(ValueError('Nonfinite JSON')))
            name,value=decode(command,response)
            values[name]=value
            log(dict(event='response',command=command,response=response,name=name,SI_value=value))
        except BaseException as exc:
            log(dict(event='error',command=command,buffer_hex=buffer.hex(),error=repr(exc),retry=False))
            raise
        finally:
            if sock is not None:sock.close()
    return values
