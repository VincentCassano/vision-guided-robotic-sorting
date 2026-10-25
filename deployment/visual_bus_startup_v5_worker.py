"""One explicit, historically used terminal RTU setup; isolated from motion."""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import socket
import time
import traceback

ROOT=Path(__file__).resolve().parents[1]
MANIFEST=ROOT/'deployment/visual-pregrasp-motion-v5-files.sha256'


def verify_delivery():
    inventory={}
    for line in MANIFEST.read_text(encoding='ascii').splitlines():
        digest,name=line.split('  ',1);path=(ROOT/name).resolve()
        if (ROOT not in path.parents or name in inventory or len(digest)!=64
                or hashlib.sha256(path.read_bytes()).hexdigest()!=digest):
            raise ValueError('Delivery changed: '+name)
        inventory[name]=digest
    if not inventory:raise ValueError('Empty delivery manifest')
    return inventory


def probe_tool_mode(wire):
    # Optional getter is generation-specific. Never assume support or invent it.
    request={'command':'get_tool_RS485_mode'}
    row=dict(role='optional_tool_RS485_mode',request=request,recognized=False,
             endpoint=['169.254.128.18',8080],chunks_hex=[],socket_closed=False)
    sock=None;raw=b'';deadline=time.monotonic()+3
    def left():
        value=deadline-time.monotonic()
        if value<=0:raise TimeoutError('Optional tool mode GET3s deadline')
        return value
    try:
        from arm_protection_transport import unique_object
        sock=socket.create_connection(('169.254.128.18',8080),timeout=left())
        payload=(json.dumps(request,separators=(',',':'))+'\r\n').encode('ascii')
        row['request_hex']=payload.hex();sock.settimeout(left());sock.sendall(payload)
        while b'\n' not in raw:
            sock.settimeout(left());part=sock.recv(4096);row['chunks_hex'].append(part.hex())
            if not part:raise OSError('Optional getter EOF')
            raw+=part
            if len(raw)>65536:raise ValueError('Optional getter oversized reply')
        line,extra=raw.split(b'\n',1)
        if extra or not line.endswith(b'\r'):raise ValueError('Extra/non-CRLF optional reply')
        reply=json.loads(line.decode('utf8'),object_pairs_hook=unique_object)
        row['response']=reply
        row['recognized']=(isinstance(reply,dict) and reply.get('command')==request['command']
            and type(reply.get('tool_RS485_mode')) is int and reply['tool_RS485_mode'] in (0,1,2)
            and type(reply.get('baudrate')) is int)
    except Exception as exc:row['error']=repr(exc)
    finally:
        row['raw_received_hex']=raw.hex()
        if sock is not None:
            try:sock.close();row['socket_closed']=True
            except BaseException as exc:row['close_error']=repr(exc)
        wire.write(json.dumps(row,ensure_ascii=False)+'\n');wire.flush()
    if row.get('close_error'):raise RuntimeError('Optional getter socket cleanup failed')
    return row


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--folder',type=Path,required=True)
    parser.add_argument('--initialize-hand-modbus',action='store_true',required=True)
    parser.add_argument('--manifest-sha256',required=True)
    args=parser.parse_args()
    if platform.system()!='Linux' or platform.machine()!='aarch64':parser.error('Actual robot Linux aarch64 only')
    from known_cylinder_records import dump
    folder=args.folder
    report=dict(scope='one_explicit_controller_terminal_RTU_configuration_before_separate_motion_run',
        SDK_loaded=False,arm_actuator_writes=0,hand_register_writes=0,
        controller_configuration_writes=0,physical_bus_cause_confirmed=False,
        configuration_count_kind='raw_TX_attempts_not_ACK_or_confirmed_device_state',
        execution_ready=False,exceptions=[],setup_acknowledged=False)
    def save():dump(folder/'bus_worker_result.json',report)
    save()
    try:
        if hashlib.sha256(MANIFEST.read_bytes()).hexdigest()!=args.manifest_sha256:raise ValueError('Manifest changed')
        verify_delivery()
        from read_visual_collision_snapshot import query,QUERIES
        from hand_bus_startup import HandBusStartup,REQUEST
        import fcntl
        with (ROOT/'captures/hand_stage.lock').open('a') as lock, \
             (folder/'bus_startup_wire.jsonl').open('w',encoding='utf8') as wire:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            def get(role):
                row=query(next(item for item in QUERIES if item[0]==role),wire)
                if row.get('error') or row.get('close_error') or row.get('socket_closed') is not True:
                    raise RuntimeError('Fresh pre-setup GET failed: '+role)
                return row['response']
            identity=get('identity');arm=get('arm_before');collision=get('collision_level');self_collision=get('self_collision')
            report['before']=dict(identity=identity,arm=arm,collision=collision,self_collision=self_collision);save()
            arm_errors=[arm.get('arm_state',{}).get(key) for key in ('arm_err','sys_err')]
            if (identity.get('Product_version')!='RM65-BI'
                    or identity.get('ctrl_info',{}).get('version')!='V1.6.5'
                    or any(type(value) is not int or value!=0 for value in arm_errors)
                    or type(collision.get('collision_stage')) is not int
                    or collision['collision_stage']!=8 or self_collision.get('enable_state') is not True):
                raise RuntimeError('Expected healthy actual robot and protections8/True required before bus setup')
            report['mode_before']=probe_tool_mode(wire);report['setup_request']=dict(REQUEST);save()
            def log(row):
                wire.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n');wire.flush()
                if row.get('event')=='tx':
                    if row.get('request')!=REQUEST or report['controller_configuration_writes']!=0:
                        raise RuntimeError('Only one exact known bus setup allowed')
                    report['controller_configuration_writes']+=1;save()
            report['setup_response']=HandBusStartup(log).configure_once()
            report['setup_acknowledged']=True;save()
            report['mode_after']=probe_tool_mode(wire)
            after=report['mode_after']
            if after['recognized']:
                mode=after['response']
                if (mode['tool_RS485_mode']!=1 or mode['baudrate']!=115200
                        or (mode.get('modbus_timeout') is not None and mode['modbus_timeout']!=1)):
                    raise RuntimeError('Recognized tool mode readback differs after setup; no movement')
            report['after_protection']=dict(collision=get('collision_level'),self_collision=get('self_collision'))
            if (report['after_protection']['collision'].get('collision_stage')!=8
                    or report['after_protection']['self_collision'].get('enable_state') is not True):
                raise RuntimeError('Protection differs after bus setup; no motion')
            report['outcome']='communication_setup_ACK_only_actual_hand_configuration_and_motion_still_required'
    except BaseException as exc:
        report['outcome']='bus_setup_aborted_no_retry_or_motion'
        report['exceptions'].append(dict(error=repr(exc),traceback=traceback.format_exc()))
    finally:save()
    return 0 if report.get('setup_acknowledged') and not report['exceptions'] else 1


if __name__=='__main__':raise SystemExit(main())
