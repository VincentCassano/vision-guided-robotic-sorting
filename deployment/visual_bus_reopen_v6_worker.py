"""One explicit controller tool RTU close/open; no SDK or actuator capability."""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import select
import socket
import time
import traceback

from visual_bus_startup_v5_worker import probe_tool_mode

ROOT=Path(__file__).resolve().parents[1]
MANIFEST=ROOT/'deployment/visual-pregrasp-motion-v6-files.sha256'
CLOSE={'command':'close_modbus_mode','port':1}
OPEN={'command':'set_modbus_mode','port':1,'baudrate':115200,'timeout':1}


def verify_delivery():
    inventory={}
    for line in MANIFEST.read_text(encoding='ascii').splitlines():
        digest,name=line.split('  ',1);path=(ROOT/name).resolve()
        if (ROOT not in path.parents or name in inventory or len(digest)!=64
                or hashlib.sha256(path.read_bytes()).hexdigest()!=digest):raise ValueError('Delivery changed: '+name)
        inventory[name]=digest
    if not inventory:raise ValueError('Empty delivery')
    return inventory


def close_once(log):
    from arm_protection_transport import unique_object
    sock=None;raw=b'';deadline=time.monotonic()+3
    def left():
        seconds=deadline-time.monotonic()
        if seconds<=0:raise TimeoutError('RTU close3s deadline; no retry')
        return seconds
    try:
        sock=socket.create_connection(('169.254.128.18',8080),timeout=left())
        if select.select([sock],[],[],min(.3,left()))[0]:
            sock.settimeout(left());raw=sock.recv(4096)
            log(dict(event='rx_before_tx',role='close',hex=raw.hex()))
            raise RuntimeError('Unsolicited data before close; no close TX')
        payload=(json.dumps(CLOSE,separators=(',',':'))+'\r\n').encode('ascii')
        log(dict(event='tx',request=CLOSE,hex=payload.hex(),monotonic=time.monotonic()))
        sock.settimeout(left());sock.sendall(payload)
        while b'\n' not in raw:
            sock.settimeout(left());part=sock.recv(4096)
            log(dict(event='rx',role='close',hex=part.hex(),monotonic=time.monotonic()))
            if not part:raise OSError('RTU close EOF')
            raw+=part
            if len(raw)>65536:raise ValueError('Oversized RTU close reply')
        line,extra=raw.split(b'\n',1)
        if extra or not line.endswith(b'\r'):raise ValueError('Extra/non-CRLF close reply')
        answer=json.loads(line.decode('utf8'),object_pairs_hook=unique_object)
        if not isinstance(answer,dict) or answer.get('command')!='close_modbus_mode' or answer.get('set_state') is not True:
            raise RuntimeError('RTU close rejected/uncertain: '+str(answer))
        if select.select([sock],[],[],min(.05,left()))[0]:
            more=sock.recv(4096);log(dict(event='rx_after_reply',role='close',hex=more.hex()))
            if more:raise RuntimeError('Extra RTU close response')
        log(dict(event='response',role='close',response=answer,monotonic=time.monotonic()))
        return answer
    except BaseException as exc:
        log(dict(event='transport_error',role='close',error=repr(exc),raw_received_hex=raw.hex(),retry=False));raise
    finally:
        if sock is not None:sock.close();log(dict(event='closed',role='close'))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--folder',type=Path,required=True)
    parser.add_argument('--restart-tool-rs485',action='store_true',required=True)
    parser.add_argument('--manifest-sha256',required=True)
    args=parser.parse_args()
    if platform.system()!='Linux' or platform.machine()!='aarch64':parser.error('Actual Linux aarch64 robot only')
    from known_cylinder_records import dump
    from read_visual_collision_snapshot import query,QUERIES
    from hand_bus_startup import HandBusStartup
    import fcntl
    folder=args.folder
    report=dict(scope='one_fixed_tool_RTU_close_open_before_separate_visual_motion_gate',SDK_loaded=False,
        arm_actuator_writes=0,hand_register_writes=0,controller_configuration_writes=0,
        configuration_count_kind='raw_TX_attempts_not_ACK_or_confirmed_device_state',exceptions=[],
        setup_acknowledged=False,close_TX_attempted=False,open_TX_attempted=False,
        original_mode_restored_verified=False,close_reopen_verified=False,execution_ready=False,
        communication_fault_repaired=False)
    def save():dump(folder/'bus_worker_result.json',report)
    save()
    try:
        if hashlib.sha256(MANIFEST.read_bytes()).hexdigest()!=args.manifest_sha256:raise ValueError('Manifest changed')
        verify_delivery()
        with (ROOT/'captures/hand_stage.lock').open('a') as lock, (folder/'bus_startup_wire.jsonl').open('w',encoding='utf8') as wire:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            def log(row):
                if row.get('event')=='tx':
                    request=row.get('request')
                    if request==CLOSE and not report['close_TX_attempted'] and not report['open_TX_attempted']:
                        report['close_TX_attempted']=True
                    elif request==OPEN and report['close_TX_attempted'] and not report['open_TX_attempted']:
                        report['open_TX_attempted']=True
                    else:raise RuntimeError('Only one close and one original-mode open TX allowed')
                    report['controller_configuration_writes']+=1;save()
                wire.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n');wire.flush()
            def get(role):
                row=query(next(item for item in QUERIES if item[0]==role),wire)
                if row.get('error') or row.get('close_error') or row.get('socket_closed') is not True:
                    raise RuntimeError('Fresh GET failed: '+role)
                return row['response']
            def health():
                values=dict(identity=get('identity'),arm=get('arm_before'),collision=get('collision_level'),self_collision=get('self_collision'))
                state=values['arm'].get('arm_state',{})
                if (values['identity'].get('Product_version')!='RM65-BI'
                        or values['identity'].get('ctrl_info',{}).get('version')!='V1.6.5'
                        or any(type(state.get(k)) is not int or state[k]!=0 for k in ('arm_err','sys_err'))
                        or values['collision'].get('collision_stage')!=8 or values['self_collision'].get('enable_state') is not True):
                    raise RuntimeError('Actual identity/error/protection differs; no motion')
                return values
            def original_mode():
                row=probe_tool_mode(wire)
                if not row['recognized'] or row['response']!=dict(command='get_tool_RS485_mode',
                        tool_RS485_mode=1,baudrate=115200,modbus_timeout=1):
                    raise RuntimeError('Original tool RTU1/115200/timeout1 not verified')
                return row
            open_called=False
            def open_original_once():
                nonlocal open_called
                if open_called:raise RuntimeError('Original-mode reopen not retried')
                open_called=True
                report['setup_response']=HandBusStartup(log).configure_once();report['setup_acknowledged']=True;save()
                report['mode_after']=original_mode();report['original_mode_restored_verified']=True;save()
            report['before']=health();report['mode_before']=original_mode();save()
            try:
                report['close_response']=close_once(log);save()
                report['mode_closed']=probe_tool_mode(wire);save()
                if not report['mode_closed']['recognized'] or report['mode_closed']['response'].get('tool_RS485_mode')!=0:
                    raise RuntimeError('Closed serial mode0 not read back; no motion')
                open_original_once()
                report['after']=health();report['close_reopen_verified']=True;save()
            finally:
                # Even an uncertain close is followed only by the one explicitly
                # allowed original-mode restoration. Never retry its failure.
                if report['close_TX_attempted'] and not open_called:
                    try:open_original_once()
                    except BaseException as exc:
                        report['exceptions'].append(dict(stage='original_mode_restoration',error=repr(exc),traceback=traceback.format_exc()))
            report['outcome']='interface_close_open_readback_only_full_hand_and_motion_gates_still_required'
    except BaseException as exc:
        report['outcome']='interface_reopen_aborted_no_motion_or_retry'
        report['exceptions'].append(dict(error=repr(exc),traceback=traceback.format_exc()))
    finally:save()
    return 0 if report['close_reopen_verified'] and report['original_mode_restored_verified'] and not report['exceptions'] else 1


if __name__=='__main__':raise SystemExit(main())
