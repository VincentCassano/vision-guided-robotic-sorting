"""One editable hand-shape capture stage; no arm SDK or arm command.

An explicit x records a fresh complete snapshot and returns to the parent.
Targets are operator-selected counts, not angles, force or proof of holding.
"""
import argparse
import copy
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import re
import sys

ROOT=Path(__file__).resolve().parents[1]
KEYS={'a':0,'b':1,'c':2,'d':3,'e':4,'o':5}


def valid_snapshot(snapshot):
    if not isinstance(snapshot,dict):return False
    for key in ('targets','positions','current','current_limit_ma','status'):
        values=snapshot.get(key)
        if (not isinstance(values,(list,tuple)) or len(values)!=6
                or any(type(v) is not int or not 0<=v<=65535 for v in values)):
            return False
    return all(v in range(6) for v in snapshot['status'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--role',choices=('grip','prepare'),required=True)
    parser.add_argument('--manifest',type=Path,required=True)
    args=parser.parse_args()
    output=args.output.resolve();manifest=args.manifest.resolve()
    session=datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    prefix=output.stem+'_'+session
    wire_path=output.parent/(prefix+'_wire.jsonl')
    events_path=output.parent/(prefix+'_events.jsonl')
    pending=output.with_name(prefix+'.pending')
    report=dict(scope='operator_selected_hand_shape_capture_only',version=1,role=args.role,
        recorded=False,outcome='editing',SDK_loaded=False,arm_pose_queried=False,
        arm_motion_writes=0,arm_stop_writes=0,controller_configuration_writes=0,
        target_attempts=0,target_acknowledgements=0,actual_finger_arrival_required=False,
        ordinary_status5_is_failure=False,holding_verified=False,physical_stop_verified=False,
        automatic_motion_qualified=False,wire_file=wire_path.name,events_file=events_path.name,
        manifest_file=str(manifest),started_timestamp_utc=datetime.now(timezone.utc).isoformat(),
        notices=[],cleanup_errors=[],snapshot=None,snapshot_stale=True)
    bus=None;latest=None;selected='a';notice='';disk_error=None

    def save():
        nonlocal disk_error,notice
        try:
            output.parent.mkdir(parents=True,exist_ok=True)
            pending.write_bytes((json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode('utf8'))
            pending.replace(output)
            disk_error=None
            return True
        except Exception as exc:
            disk_error=repr(exc);notice='记录未能写入：'+str(exc)+'；r重读或q取消。'
            print(notice,flush=True)
            return False

    def append(path,row):
        output.parent.mkdir(parents=True,exist_ok=True)
        with path.open('a',encoding='utf8') as stream:
            stream.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n');stream.flush()

    def event(kind,**fields):
        append(events_path,dict(event=kind,timestamp_utc=datetime.now(timezone.utc).isoformat(),**fields))

    def wire(row):
        append(wire_path,row)

    def verify_manifest():
        checked={}
        for line in manifest.read_text(encoding='ascii').splitlines():
            digest,name=line.split('  ',1);path=(ROOT/name).resolve()
            if (not name or name in checked or ROOT not in path.parents
                    or len(digest)!=64 or any(v not in '0123456789abcdef' for v in digest)
                    or hashlib.sha256(path.read_bytes()).hexdigest()!=digest):
                raise ValueError('交付文件不符：'+name)
            checked[name]=digest
        if not checked:raise ValueError('交付清单为空。')
        report['manifest_sha256']=hashlib.sha256(manifest.read_bytes()).hexdigest()
        report['checked_files']=len(checked)
        return len(checked)

    def accept(result):
        nonlocal latest,notice
        if not isinstance(result,dict):raise ValueError('手部未返回有效记录；r重连。')
        report['latest_bus_result']=copy.deepcopy(result)
        if result.get('snapshot') is not None:
            latest=copy.deepcopy(result['snapshot'])
            report['snapshot']=copy.deepcopy(latest)
        report['snapshot_stale']=result.get('snapshot_stale') is not False
        report['configuration_matched']=result.get('configuration_matched',False)
        report['blockers']=copy.deepcopy(result.get('blockers',[]))
        if result.get('attempted'):report['target_attempts']+=1
        if result.get('acknowledged'):report['target_acknowledgements']+=1
        notice=str(result.get('notice','已读取。'))
        event('bus_result',result=result)
        save()
        return result

    def connect():
        nonlocal bus,notice
        if bus is not None:bus.close();bus=None
        if not save():return
        verify_manifest()
        from manual_hand_bus_v2 import ManualHandBus
        bus=ManualHandBus(wire)
        print('读取当前手型，不发送初始手势。',flush=True)
        accept(bus.connect_read())

    def refresh():
        nonlocal notice
        if bus is None:
            notice='尚未连接；r重读。';return None
        return accept(bus.refresh())

    def record_shape():
        nonlocal notice
        row=refresh()
        if (not isinstance(row,dict) or row.get('snapshot_stale') is not False
                or row.get('needs_reconnect') or not valid_snapshot(row.get('snapshot'))):
            notice='未取得新鲜完整手型；r重连后再x，界面保留。';return False
        # This captures targets, including a protected/stopped-short channel.
        # It does not convert those targets into automatic-motion readiness.
        hand=copy.deepcopy(row['snapshot'])
        report.update(recorded=True,outcome='shape_recorded',snapshot=hand,snapshot_stale=False,
            targets=hand['targets'][:],positions=hand['positions'][:],current=hand['current'][:],
            status=hand['status'][:],protected_channels=copy.deepcopy(hand.get('protected_channels',[])),
            capture_bus_ok=row.get('ok') is True,
            recorded_timestamp_utc=datetime.now(timezone.utc).isoformat(),
            snapshot_is_capture_not_live_after_exit=True,automatic_motion_qualified=False)
        event('shape_recorded',role=args.role,targets=hand['targets'],actual_arrival_required=False)
        if not save():
            report['recorded']=False;report['outcome']='editing';return False
        print(('抓取' if args.role=='grip' else '预抓取')+'手势已记录，返回主流程。',flush=True)
        return True

    def screen():
        title='抓取手势' if args.role=='grip' else '预抓取手势'
        print('\n'+title+' | a拇 b食 c中 d无名 e小 o对掌 | 全部0..65535',flush=True)
        if latest is not None:
            print('目标：'+str(latest.get('targets',[])),flush=True)
            print('实际：%s 状态：%s'%(latest.get('positions',[]),latest.get('status',[])),flush=True)
        print('如 a 40000 / d +500；x记录并返回，r重连，q取消本阶段。',flush=True)
        if notice:print(notice,flush=True)

    try:
        try:connect()
        except KeyboardInterrupt:notice='读取已中断；r重读，q取消。'
        except Exception as exc:
            notice='未连接：'+str(exc)+'；r重读或q取消。'
            report['notices'].append(dict(error=repr(exc),phase='initial_read'));save()
        while True:
            try:
                screen();raw=input(selected+'> ').strip().lower()
                event('operator_input',value=raw)
                if raw=='q':
                    report.update(recorded=False,outcome='operator_cancelled_stage');break
                if raw=='x':
                    if record_shape():break
                    continue
                if raw=='r':connect();continue
                if not raw:refresh();continue
                if raw in KEYS:selected=raw;notice='已选'+selected+'；输入整数目标。';continue
                match=re.fullmatch(r'(?:(?P<key>[abcdeo])\s*)?(?P<absolute>=)?\s*(?P<number>[+-]?\d+)',raw)
                if match is None:
                    notice='输入如 a 40000、d +500、d -500 或 x。';continue
                if match.group('key'):selected=match.group('key')
                if bus is None:notice='尚未连接；先r重读。';continue
                if disk_error is not None:notice='记录尚未可写；r重读，未发送新目标。';continue
                token=match.group('number');value=int(token)
                if match.group('absolute') or not token.startswith(('+','-')):
                    accept(bus.set_target(KEYS[selected],absolute=value))
                else:accept(bus.set_target(KEYS[selected],delta=value))
            except KeyboardInterrupt:notice='本条已中断，界面保留；不重发。'
            except EOFError:
                report.update(recorded=False,outcome='input_closed_stage_cancelled');break
            except Exception as exc:
                notice='本条未确认：'+str(exc)+'；界面保留，可r重读或q取消。'
                report['notices'].append(dict(error=repr(exc),phase='editing'));save()
    finally:
        if bus is not None:
            try:report['close_result']=bus.close()
            except Exception as exc:report['cleanup_errors'].append(dict(method='bus.close',error=repr(exc)))
        report['closed_timestamp_utc']=datetime.now(timezone.utc).isoformat()
        report['live_connection_retained']=False
        save()
    return 0 if report.get('recorded') is True else 2


if __name__=='__main__':raise SystemExit(main())
