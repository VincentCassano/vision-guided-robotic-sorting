"""Persistent manual thumb-opposition/finger console; no arm SDK or pose control."""
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import re
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
MANIFEST=ROOT/'deployment/new-grasp-reference-v1-files.sha256'
KEYS={'a':0,'b':1,'c':2,'d':3,'e':4,'o':5}
NAMES=['拇指弯曲','食指','中指','无名指','小拇指','拇指对掌']
OPPOSITION_CANDIDATE=65535


def main():
    execute='--execute' in sys.argv[1:]
    folder=ROOT/'captures'/datetime.now().strftime('hand_thumb_tuner_v2_%Y%m%d_%H%M%S_%f')
    selected='o';bus=None;latest=None;latest_stale=True;notice='';observed=None;saves=0;events=[]
    report=dict(scope='manual_thumb_and_five_digit_targets_only',execute_requested=execute,
        outcome='manual_console_running',automatic_arm_motion=False,SDK_loaded=False,
        arm_pose_queried=False,hand_target_attempts=0,hand_target_acknowledgements=0,
        physical_stop_verified=False,holding_verified=False,automatic_reference_qualified=False,
        offline_project_tests_executed=False)

    def write_json(path,value):
        try:
            path.parent.mkdir(parents=True,exist_ok=True)
            pending=path.with_name(path.name+'.pending')
            pending.write_bytes((json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode('utf8'))
            pending.replace(path)
            return True
        except Exception as exc:
            print('记录未写入：'+str(exc),flush=True)
            return False

    def event(kind,**values):
        entry=dict(event=kind,timestamp_utc=datetime.now(timezone.utc).isoformat(),**values)
        events.append(entry)
        try:
            folder.mkdir(parents=True,exist_ok=True)
            with (folder/'events.jsonl').open('a',encoding='utf8') as out:
                out.write(json.dumps(entry,ensure_ascii=False,allow_nan=False)+'\n')
        except Exception:
            report['disk_logging_incomplete']=True

    def verify_delivery():
        errors=[];checked=0
        try:
            for line in MANIFEST.read_text(encoding='ascii').splitlines():
                digest,name=line.split('  ',1);path=(ROOT/name).resolve()
                if ROOT not in path.parents or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
                    errors.append(name)
                checked+=1
            if not checked:errors.append('empty manifest')
        except Exception as exc:errors.append(str(exc))
        report['delivery_check']=dict(checked=checked,differences=errors)
        return errors

    def accept(result):
        nonlocal latest,latest_stale,notice,observed
        if not isinstance(result,dict):
            notice='未取得有效读数；r重新读取，界面保留。'
            return
        if result.get('snapshot') is not None:
            latest=result['snapshot']
            if not result.get('snapshot_stale'):observed=time.monotonic()
        latest_stale=bool(result.get('snapshot_stale',True))
        notice=str(result.get('notice','已读取'))
        if result.get('attempted'):report['hand_target_attempts']+=1
        if result.get('acknowledged'):report['hand_target_acknowledgements']+=1
        report['last_bus_result']=result
        event('manual_bus_result',result=result)
        write_json(folder/'result.json',report)

    def connect():
        nonlocal bus,notice
        prior,bus=bus,None
        if prior is not None:prior.close()
        differences=verify_delivery()
        if differences:
            notice='文件核对未通过，未连接设备；修正文件后r重读，q退出。'
            return
        try:
            from manual_hand_bus_v2 import ManualHandBus
            bus=ManualHandBus(lambda row:event('wire',row=row))
            accept(bus.connect_read())
        except Exception as exc:
            notice='连接未完成：'+str(exc)+'；r重读，界面保留。'
            event('connection_notice',error=repr(exc))

    def refresh():
        nonlocal notice
        if bus is None:
            notice='尚未连接；r读取当前手部。'
            return
        accept(bus.refresh())

    def target(channel,absolute=None,delta=None):
        nonlocal notice
        if not execute:
            notice='当前只读；使用 --execute 才发送手指目标。'
            return
        if report.get('disk_logging_incomplete'):
            notice='原始记录未能写入，保留界面，未发送新目标。'
            return
        if bus is None:
            notice='尚未连接；r读取后再输入目标。'
            return
        accept(bus.set_target(channel,absolute=absolute,delta=delta))

    def preset(which):
        nonlocal notice
        recipe=json.loads((ROOT/'deployment/black_shell_new_grasp_recipe_20261023.json').read_bytes())
        pre=recipe['pregrasp_targets'];grip=recipe['grasp_targets']
        if pre!=[35000,7500,9480,30000,65535,65535] or grip!=[40000,12500,14480,30000,65535,65535]:
            notice='配方文件不符，保留界面，未发指令。'
            return
        steps=([(5,pre[5]),(4,pre[4]),(3,pre[3]),(0,pre[0]),(1,pre[1]),(2,pre[2])] if which=='pre'
            else [(4,pre[4]),(3,pre[3])] if which=='hold'
            else [(0,grip[0]),(1,grip[1]),(2,grip[2])])
        if which=='grip':
            refresh()
            if latest is None or latest_stale or any(abs(latest['targets'][ch]-grip[ch])>2 for ch in (3,4,5)):
                notice='无名指/小拇指/对掌目标尚非30000/65535/65535；先明确输入pre或hold及p。'
                return
        for channel,value in steps:
            prior=report['hand_target_attempts']
            target(channel,absolute=value)
            result=report.get('last_bus_result',{})
            if not execute or bus is None or report.get('disk_logging_incomplete') or not result.get('ok') or result.get('needs_reconnect'):
                notice+='；配方暂停，仍可手动调指，不自动重试。'
                return
            event('preset_step',preset=which,channel=channel,target=value,
                sent=report['hand_target_attempts']>prior,actual_arrival_required=False)
        notice='配方目标已写入/回读；堵转或未到位只显示，未发自动提起。'

    def save_shape():
        nonlocal saves,notice
        refresh();saves+=1
        result=dict(scope='manual_hand_shape_snapshot_not_qualified_grasp',
            timestamp_utc=datetime.now(timezone.utc).isoformat(),snapshot=latest,
            snapshot_stale=latest_stale,
            latest_bus_result=report.get('last_bus_result'),arm_pose=None,
            holding_verified=False,automatic_reference_qualified=False,physical_stop_verified=False)
        path=folder/('hand_shape_%03d.json'%saves)
        if write_json(path,result):
            if latest is not None and not latest_stale:
                write_json(ROOT/'calibration/hand_thumb_tuning_latest.json',result)
                notice='已保存手型：'+str(path)
            else:notice='已保存旧读数/故障记录，未更新当前手型：'+str(path)
        else:notice='手型未能写入文件，界面保留。'

    def screen():
        if sys.stdout.isatty():print('\033[2J\033[H',end='')
        print('手部手动调试 | 六路下限0 / 上限65535 | 已选 '+selected+' '+NAMES[KEYS[selected]],flush=True)
        if latest:
            for key,ch in KEYS.items():
                def item(name):
                    values=latest.get(name,[])
                    return values[ch] if isinstance(values,(list,tuple)) and len(values)>ch else '?'
                print('%s %-5s 范围0..65535 实际 %s / 目标 %s  状态 %s  电流 %s mA'%(
                    key,NAMES[ch],item('positions'),item('targets'),item('status'),item('current')),flush=True)
        else:print('暂未取得手部读数；可r重新读取。',flush=True)
        print('p对掌65535 | pre新预抓 | hold小/无名指 | grip拇→食→中 | a-e/o选指 | 任意数值0..65535',flush=True)
        print('x保存继续 | r新读 | 回车刷新 | q退出；仅手部，不自动提起。',flush=True)
        if latest_stale:print('当前显示为旧读数；r重连，未更新当前手型。',flush=True)
        elif observed is not None:print('最近读数 %.1f 秒前；写入前会重新读取。'%(time.monotonic()-observed),flush=True)
        if notice:print(notice,flush=True)

    print('仅手部调试：关闭其他手部控制，手内无物、六路活动范围清空；先p对掌，再调a-e。',flush=True)
    try:
        try:connect()
        except KeyboardInterrupt:notice='读取已中断，界面保留；r新读，不自动重发。'
        while True:
            try:
                screen()
                raw=input(selected+'> ').strip().lower()
                event('operator_input',value=raw)
                if raw=='q':
                    report['outcome']='operator_exit_preserving_targets'
                    break
                if not raw:refresh();continue
                if raw in KEYS:selected=raw;notice='';continue
                if raw in ('r','s'):connect();continue
                if raw in ('x','save'):save_shape();continue
                if raw=='p':selected='o';target(5,absolute=OPPOSITION_CANDIDATE);continue
                if raw in ('pre','hold','grip'):preset(raw);continue
                if raw in ('w','?','help'):
                    notice=('本工具不控制或检查臂位，可用示教拖臂。p对掌；o可自调对掌；'
                        'd 18000设值，d +500/-500按最新实际加减，数字任意，0..65535限幅。')
                    continue
                match=re.fullmatch(r'(?:(?P<key>[abcdeo])\s*)?(?P<absolute>=)?\s*(?P<number>[+-]?\d+)',raw)
                if match is None:
                    notice='输入如 p、d 18000、d +500、o -500；输入错误不会退出。'
                    continue
                if match.group('key'):selected=match.group('key')
                token=match.group('number');value=int(token)
                if match.group('absolute') or not token.startswith(('+','-')):
                    target(KEYS[selected],absolute=value)
                else:target(KEYS[selected],delta=value)
            except KeyboardInterrupt:
                notice='已回到调试界面；q才退出，不自动重发上一条。'
                event('keyboard_interrupt_no_retry')
            except EOFError:
                report['outcome']='operator_closed_terminal_input'
                break
            except Exception as exc:
                notice='本条未确认完成：'+str(exc)+'；界面保留，可r新读。'
                event('command_notice',error=repr(exc),automatic_retry=False)
    finally:
        if bus is not None:
            try:bus.close()
            except Exception as exc:event('close_notice',error=repr(exc))
        report.update(last_snapshot=latest,saved_shapes=saves,physical_stop_verified=False,
            closed_timestamp_utc=datetime.now(timezone.utc).isoformat())
        write_json(folder/'result.json',report)
        write_json(folder/'events_memory_copy.json',events)
        print('记录留在 '+str(folder)+'；关闭连接不等于电机停止。',flush=True)


if __name__=='__main__':main()
