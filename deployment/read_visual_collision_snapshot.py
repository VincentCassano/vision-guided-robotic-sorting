#!/usr/bin/env python3
"""One raw GET-only collision snapshot; no SDK, hand bus or recovery writes."""
from datetime import datetime,timezone
import fcntl
import hashlib
import json
from pathlib import Path
import platform
import select
import socket
import time
import traceback
import zipfile

ROOT=Path(__file__).resolve().parents[1]
ENDPOINT=('169.254.128.18',8080)
QUERIES=(('identity','get_arm_software_info','arm_software_info'),
    ('arm_before','get_current_arm_state','current_arm_state'),
    ('joint_errors_and_brakes','get_joint_err_flag','joint_err_flag'),
    ('all_joint_states','get_arm_all_state','arm_all_state'),
    ('active_tool_payload_raw','get_current_tool_frame','current_tool_frame'),
    ('active_work_frame_raw','get_current_work_frame','current_work_frame'),
    ('installation','get_install_pose','install_pose'),
    ('collision_level','get_collision_stage','get_collision_stage'),
    ('self_collision','get_self_collision_enable','get_self_collision_enable'),
    ('joint_enable','get_joint_en_state','joint_en_state'),
    ('arm_after','get_current_arm_state','current_arm_state'))


def dump(path,value):
    raw=(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode('utf8')
    pending=path.with_name(path.name+'.pending');pending.write_bytes(raw);pending.replace(path)


def pairs(values):
    result={}
    for key,value in values:
        if key in result:raise ValueError('Duplicate response key: '+key)
        result[key]=value
    return result


def query(spec,wire):
    if spec not in QUERIES:raise ValueError('Only fixed documented GETs allowed')
    role,command,expected=spec
    event=dict(role=role,request={'command':command},endpoint=list(ENDPOINT),
        started_monotonic=time.monotonic(),timestamp_utc=datetime.now(timezone.utc).isoformat(),
        chunks_hex=[],socket_closed=False)
    sock=None;buffer=b'';deadline=time.monotonic()+3
    def remaining():
        left=deadline-time.monotonic()
        if left<=0:raise TimeoutError('Absolute3s GET deadline; no retry')
        return left
    try:
        sock=socket.create_connection(ENDPOINT,timeout=remaining())
        payload=(json.dumps(event['request'],separators=(',',':'))+'\r\n').encode('ascii')
        event['request_hex']=payload.hex();sock.settimeout(remaining());sock.sendall(payload)
        while b'\n' not in buffer:
            sock.settimeout(remaining());part=sock.recv(4096);event['chunks_hex'].append(part.hex())
            if not part:raise OSError('GET reply EOF')
            buffer+=part
            if len(buffer)>65536:raise ValueError('Reply oversized')
        line,extra=buffer.split(b'\n',1)
        if not line.endswith(b'\r') or extra:raise ValueError('Invalid CRLF or extra reply')
        response=json.loads(line.decode('utf8'),object_pairs_hook=pairs,
            parse_constant=lambda v:(_ for _ in ()).throw(ValueError('Nonfinite response')))
        event['response']=response
        if (not isinstance(response,dict) or expected not in (response.get('state'),response.get('command'))
                or any(response.get(key) is False for key in ('read_state','get_state'))):
            raise ValueError('Unrecognized/rejected GET response; raw retained, no assumed value')
        if select.select([sock],[],[],min(.05,remaining()))[0]:
            more=sock.recv(4096)
            event['chunks_hex'].append(more.hex())
            if more:raise ValueError('Late/extra GET response')
        event['response_schema_recognized']=True
    except Exception as exc:
        event['error']=repr(exc)
    finally:
        event['raw_received_hex']=buffer.hex()
        if sock is not None:
            try:sock.close();event['socket_closed']=True
            except BaseException as exc:event['close_error']=repr(exc)
        event['finished_monotonic']=time.monotonic()
        wire.write(json.dumps(event,ensure_ascii=False,allow_nan=False)+'\n');wire.flush()
    return event


def main():
    if platform.system()!='Linux' or platform.machine()!='aarch64':
        raise SystemExit('Actual Linux aarch64 host only; no connection')
    folder=ROOT/'captures'/datetime.now().strftime('visual_collision_snapshot_%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True)
    report=dict(scope='raw_current_collision_joint_and_payload_GETs_only',read_only=True,
        SDK_loaded=False,arm_motion_writes=0,hand_connections=0,hand_writes=0,
        configuration_writes=0,clear_error_or_reset_writes=0,execution_ready=False,
        physical_stop_verified=False,root_cause_confirmed=False,queries=[],exceptions=[])
    print('只读当前错误、关节/抱闸及工具负载原始报文；不摆位、不清错、不准备手型、不移动。',flush=True)
    print('先保留故障现场。若已经清错/移臂/重启，不恢复旧姿态；本次只记录现在。',flush=True)
    try:
        with (ROOT/'captures/hand_stage.lock').open('a') as lock,(folder/'raw_wire.jsonl').open('w',encoding='utf8') as wire:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            for spec in QUERIES:
                row=query(spec,wire);report['queries'].append(row);dump(folder/'result.json',report)
                print(spec[0],json.dumps(row.get('response',{}),ensure_ascii=False),row.get('error',''),flush=True)
                if spec[0]=='identity' and (row.get('error') or row.get('response',{}).get('Product_version')!='RM65-BI'):
                    raise RuntimeError('Expected actual RM65-BI identity not confirmed; no further GETs')
            report['outcome']='readonly_raw_snapshot_saved_not_motion_authorization'
    except BaseException as exc:
        report['outcome']='readonly_snapshot_incomplete'
        report['exceptions'].append(dict(error=repr(exc),traceback=traceback.format_exc()))
        print('ERROR:',repr(exc),flush=True)
    finally:
        report['query_error_roles']=[r['role'] for r in report['queries'] if r.get('error') or r.get('close_error')]
        dump(folder/'result.json',report)
        inventory={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in folder.iterdir() if p.is_file()}
        dump(folder/'source_hashes.json',inventory)
        archive=folder.with_suffix('.zip')
        with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
            for p in sorted(folder.iterdir()):
                if p.is_file():z.write(p,folder.name+'/'+p.name)
        print('SAVED:',archive,'EXECUTION_READY: False',flush=True)
    return 1 if report['exceptions'] or report['query_error_roles'] else 0


if __name__=='__main__':raise SystemExit(main())
