"""Manual wrist/finger/lift teaching loop. SDK is isolated; arm motion is manual."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import platform
import select
import signal
import subprocess
import sys
import time
import zipfile

ROOT=Path(__file__).resolve().parents[1]
MANIFEST=ROOT/'deployment/grasp-teach-loop-v2-files.sha256'


def verify_delivery():
    checked={}
    for line in MANIFEST.read_text(encoding='ascii').splitlines():
        h,n=line.split('  ',1);path=(ROOT/n).resolve()
        if ROOT not in path.parents or n in checked or len(h)!=64 or hashlib.sha256(path.read_bytes()).hexdigest()!=h:
            raise ValueError('Teaching deployment differs: '+n)
        checked[n]=h
    if not checked:raise ValueError('Empty manifest')
    return checked


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('execute','confirm-inert-shell-exclusive','confirm-onsite-stop'):
        p.add_argument('--'+name,action='store_true',required=True)
    p.add_argument('--sdk-dir',type=Path,default=Path('/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package'))
    a=p.parse_args()
    if platform.system()!='Linux' or platform.machine()!='aarch64':p.error('Actual Linux aarch64 only')
    from known_cylinder_records import dump
    f=ROOT/'captures'/datetime.now().strftime('grasp_teach_loop_v2_%Y%m%d_%H%M%S_%f');f.mkdir(parents=True)
    report=dict(scope='manual_wrist_and_finger_teaching_manual_lift_confirmation',SDK_in_parent=False,
        automatic_arm_motion=False,automatic_lift=False,automatic_sort=False,offline_tests_executed=False,
        outcome='started',rounds=[],exceptions=[],physical_grasp_success_observed=False,execution_ready=False)
    process=None;child=None;preset_path=None;preset_hash=None
    def save():dump(f/'result.json',report)
    def event(kind,**values):
        with (f/'operator_events.jsonl').open('a',encoding='utf8') as out:
            out.write(json.dumps(dict(event=kind,timestamp_utc=datetime.now(timezone.utc).isoformat(),**values),ensure_ascii=False)+'\n')
    def ask(prompt,values):
        while True:
            answer=input(prompt).strip().lower();event('answer',value=answer,prompt=prompt)
            if answer in values:return answer
            print('请输入：'+'/'.join(values),flush=True)
    def worker(mode,folder):
        nonlocal process,child
        child=folder;child.mkdir(parents=True,exist_ok=True);(child/'permissions').mkdir(exist_ok=True)
        command=[sys.executable,str(ROOT/'deployment/grasp_teach_loop_v2_worker.py'),'--mode',mode,
            '--folder',str(child),'--sdk-dir',str(a.sdk_dir.resolve()),
            '--manifest-sha256',hashlib.sha256(MANIFEST.read_bytes()).hexdigest()]
        if mode in ('grip','restore'):
            command+=['--pre-shape-record',str(preset_path),'--pre-shape-sha256',preset_hash]
        state_path=child/'worker_result.json'
        with (child/'worker_stdout.txt').open('wb') as out:
            process=subprocess.Popen(command,cwd=str(ROOT),stdin=subprocess.DEVNULL,stdout=out,stderr=subprocess.STDOUT)
            handled=set();last=None;since=time.monotonic()
            try:
                while process.poll() is None:
                    if state_path.exists():
                        state=json.loads(state_path.read_bytes());phase=state['phase']
                        if phase!=last:last=phase;since=time.monotonic();print('PHASE:',phase,flush=True)
                        if phase=='await_hand_command' and state['permission_index'] not in handled:
                            index=state['permission_index']
                            print('目标:',state['feedback']['targets'],'实际:',state['feedback']['positions'],
                                '电流mA:',state['feedback']['current'],'状态:',state['feedback']['status'],flush=True)
                            if state.get('last_command_result'):
                                last_result={k:v for k,v in state['last_command_result'].items() if k!='groups'}
                                print('上一指令:',last_result,flush=True)
                            print('a=拇指 b=食指 c=中指 d=无名指 e=小拇指；单字母=加弯250；如 d - =回退250。',flush=True)
                            print('可输入 d +500 / a -100；单次1..1000计数。闭合候选上限30000/31500/29500/20250/65535。',flush=True)
                            print('数值不是角度/夹持力；没有整组夹持快捷键，不调整对掌。先弯无名指就输入 d。',flush=True)
                            print('本阶段不要动臂或物体。x=保存本阶段并关闭SDK；q=保持退出。',flush=True)
                            print('手指指令 > ',end='',flush=True)
                            answer=None;began=time.monotonic()
                            while process.poll() is None and time.monotonic()-began<570:
                                now=json.loads(state_path.read_bytes())
                                if now.get('phase')!='await_hand_command' or now.get('permission_index')!=index:
                                    raise RuntimeError('Child changed while awaiting a finger command')
                                if select.select([sys.stdin],[],[],.2)[0]:
                                    line=sys.stdin.readline()
                                    if line=='':raise EOFError('Input closed')
                                    answer=line.strip().lower();break
                            if answer is None:raise RuntimeError('No current child/command; no stale input accepted')
                            event('hand_command',index=index,command=answer)
                            dump(child/'permissions'/('%d.json'%index),dict(index=index,command=answer,
                                only_intended_shell_contact_and_digit_sweep_clear=True))
                            handled.add(index);since=time.monotonic()
                        if time.monotonic()-since>(600 if phase=='await_hand_command' else 150):
                            raise TimeoutError('SDK child phase deadline: '+phase)
                    elif time.monotonic()-since>90:raise TimeoutError('SDK child did not produce state')
                    time.sleep(.2)
            except BaseException:
                dump(child/'abort.json',dict(abort=True))
                try:process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    process.send_signal(signal.SIGINT)
                    try:process.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        report['SDK_child_unresponsive']=True
                        print('子进程尚未确认退出；禁止手动示教或继续本程序，意外运动现场硬件停止。',flush=True)
                raise
        result=json.loads(state_path.read_bytes());result['process_returncode']=process.returncode
        process=None
        for error in result.get('exceptions',[])+result.get('cleanup_errors',[]):print('STOP:',error,flush=True)
        return result
    def good(result):
        return (result.get('process_returncode')==0 and not result.get('exceptions')
            and not result.get('cleanup_errors') and result.get('SDK_cleanup_finished') is True)
    try:
        report['delivery_sha256']=verify_delivery();save()
        print('本流程不自动移动臂、不自动提起或分拣。只在调指期间连接SDK；手动示教前必须等SDK子进程退出。',flush=True)
        print('只用同一断电空壳，退出其他控制，现场守急停。数字是原始位置计数，不是弯曲角度或持力。',flush=True)
        print('第一阶段先调预抓手势：保留当前手型，不自动发送PREPARE或旧夹持配方。',flush=True)
        print('SDK尚未启动。空手放在手势扫掠清空处，退出示教、停稳、撤手。',flush=True)
        if ask('s=确认空手与五指扫掠清空，开始a–e预抓调指；q=保持退出：',('s','q'))=='q':
            report['outcome']='operator_exit_before_pre_shape'
        else:
            pre=worker('pre_shape',f/'pre_shape');report['pre_shape']=pre;save()
            if not good(pre) or pre.get('outcome')!='pre_shape_recorded':
                raise RuntimeError('Pre-shape not recorded; no wrist or grip continuation')
            preset_path=f/'pre_shape/worker_result.json'
            preset_hash=hashlib.sha256(preset_path.read_bytes()).hexdigest()
            report['pre_shape_record_sha256']=preset_hash;save()
            for number in range(1,51):
                item=dict(round=number,success=False);report['rounds'].append(item);save()
                base=f/('round_%02d'%number);base.mkdir()
                print('SDK已退出。现在用示教器调整腕姿，保留刚保存的预抓手型。空壳直立落底。',flush=True)
                print('手与空壳/板/外物仍留间隙，不要求手到电池14mm；不要在SDK调指时拖臂。',flush=True)
                if ask('s=腕姿摆好，退出示教并停稳撤手，开始a–e夹持调指；q=保持退出：',('s','q'))=='q':break
                item['tune']=worker('grip',base/'grip');save()
                if not good(item['tune']) or item['tune'].get('outcome')!='ready_for_manual_teach_lift':
                    raise RuntimeError('Grip tuning incomplete; no automatic lift/open/retry; raw retained')
                print('SDK调指已结束并退出。现在用示教器低速World-Z+抬起查看是否夹紧，不横移/投放。',flush=True)
                answer=ask('y=空壳实际离槽且无人工托举、夹稳，成功停止记录；n=未夹紧，重调腕姿；q=保持退出：',('y','n','q'))
                item['manual_lift_operator_verdict']=answer;save()
                if answer=='y':
                    report['physical_grasp_success_observed']=True;item['success']=True
                    report['outcome']='manual_grasp_success_observed_recording';save()
                    print('退出示教并实际停稳、撤手后，s只读保存提起末态；q保留现场成功但不补读：',flush=True)
                    if ask('> ',('s','q'))=='s':
                        item['after_manual_lift']=worker('capture',base/'after_manual_lift');save()
                    else:item['after_manual_lift']={'capture_skipped_by_operator':True}
                    before=item['tune'];after=item['after_manual_lift']
                    qualified=(good(after) and all(abs(x-y)<=2 for x,y in zip(before['tuned_targets'],after.get('final_hand',{}).get('targets',[])))
                        and len(after.get('final_hand',{}).get('targets',[]))==6
                        and before['observed_run_configuration']['raw']==after.get('observed_run_configuration',{}).get('raw'))
                    success=dict(scope='operator_confirmed_manual_grasp_not_automatic_visual_acceptance',
                        physical_lift_confirmed_by_operator=True,full_success_record_qualified=qualified,
                        pre_shape_targets=pre['tuned_targets'],pre_shape_raw=pre,
                        pregrasp_arm=before['pregrasp_arm'],initial_hand=before['pregrasp_hand'],
                        commanded_grip_targets=before['tuned_targets'],pre_manual_lift_hand=before['final_hand'],
                        after_manual_lift_raw=after,configuration=before['observed_run_configuration'],
                        old_left_offsets_reused=False,automatic_motion_or_sort_accepted=False,execution_ready=False)
                    dump(f/'success.json',success);report['success']=success
                    report['outcome']='manual_grasp_success_recorded' if qualified else 'physical_success_reported_current_capture_not_qualified'
                    save();print('已停止调试并记录现场成功，没有自动张开/回臂/投放。',flush=True)
                    if not qualified:print('完整读数未合格，现场成功单独保存，不作自动执行参考。',flush=True)
                    break
                if answer=='q':break
                print('SDK已退出。先用现场操作让空壳由板/完整槽承托全部重量；五指张开及恢复预抓的扫掠须清空，撤手。',flush=True)
                answer=ask('b=确认承托/空手恢复范围清空，张开五指并恢复本次预抓手势，然后重调腕姿；q=保持退出：',('b','q'))
                if answer=='q':break
                item['open']=worker('open',base/'open');save()
                if not good(item['open']):raise RuntimeError('Opening not confirmed; no pre-shape restore or manual handoff')
                item['restore']=worker('restore',base/'restore');save()
                if not good(item['restore']):raise RuntimeError('Pre-shape restore not confirmed; no manual wrist continuation')
                print('SDK已退出，已恢复本次预抓手势。下一轮直接调整腕姿，不重建旧手型。',flush=True)
        if report['outcome']=='started':report['outcome']='stopped_without_grasp_success'
    except BaseException as exc:
        report['exceptions'].append(dict(error=repr(exc)));report['outcome']='aborted_without_automatic_recovery'
        print('STOP:',repr(exc),flush=True)
    finally:
        report['execution_ready']=False;save()
        payload={p.relative_to(f).as_posix():p.read_bytes() for p in f.rglob('*') if p.is_file() and p.name!='source_hashes.json'}
        dump(f/'source_hashes.json',{n:hashlib.sha256(b).hexdigest() for n,b in payload.items()})
        payload['source_hashes.json']=(f/'source_hashes.json').read_bytes();archive=f.with_suffix('.zip')
        with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
            for n,b in sorted(payload.items()):z.writestr(f.name+'/'+n,b)
        print('SAVED:',archive,'OUTCOME:',report['outcome'],flush=True)
        if report.get('success'):
            accepted=report['success']['full_success_record_qualified'] and not report['exceptions']
            target=ROOT/'calibration'/('grasp_teach_success.json' if accepted else 'grasp_teach_reported_success.json')
            target.parent.mkdir(parents=True,exist_ok=True)
            dump(target,dict(archive=str(archive.resolve()),archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
                **report['success']))
            print('成功记录：',target,flush=True)
        print('结果自动留本机；反馈文字即可，不以回传ZIP为继续条件。',flush=True)
    return 0 if report['outcome']=='manual_grasp_success_recorded' and not report['exceptions'] else 1


if __name__=='__main__':raise SystemExit(main())
