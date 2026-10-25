"""Manual wrist/finger/lift teaching loop. SDK is isolated; arm motion is manual."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import platform
import queue
import re
import threading
import signal
import subprocess
import sys
import time
import zipfile

ROOT=Path(__file__).resolve().parents[1]
MANIFEST=ROOT/'deployment/grasp-teach-loop-v6-files.sha256'


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
    f=ROOT/'captures'/datetime.now().strftime('grasp_teach_loop_v6_%Y%m%d_%H%M%S_%f');f.mkdir(parents=True)
    report=dict(scope='manual_wrist_and_finger_teaching_manual_lift_confirmation',SDK_in_parent=False,
        automatic_arm_motion=False,automatic_lift=False,automatic_sort=False,offline_tests_executed=False,
        outcome='started',rounds=[],exceptions=[],physical_grasp_success_observed=False,execution_ready=False)
    process=None;child=None;preset_path=None;preset_hash=None
    selected='d'
    names={'a':'拇指','b':'食指','c':'中指','d':'无名指','e':'小拇指'}
    caps=[65535]*5
    try:
        import readline  # Terminal editing/history, including arrow keys. No device import.
    except ImportError:pass
    def save():dump(f/'result.json',report)
    def event(kind,**values):
        with (f/'operator_events.jsonl').open('a',encoding='utf8') as out:
            out.write(json.dumps(dict(event=kind,timestamp_utc=datetime.now(timezone.utc).isoformat(),**values),ensure_ascii=False)+'\n')
    def ask(prompt,values):
        while True:
            answer=input(prompt).strip().lower();event('answer',value=answer,prompt=prompt)
            if answer in values:return answer
            print('请输入：'+'/'.join(values),flush=True)
    def worker(mode,folder,resume_source=None):
        nonlocal process,child,selected
        child=folder;child.mkdir(parents=True,exist_ok=True);(child/'permissions').mkdir(exist_ok=True)
        command=[sys.executable,str(ROOT/'deployment/grasp_teach_loop_v6_worker.py'),'--mode',mode,
            '--folder',str(child),'--sdk-dir',str(a.sdk_dir.resolve()),
            '--manifest-sha256',hashlib.sha256(MANIFEST.read_bytes()).hexdigest()]
        if mode in ('grip','restore'):
            command+=['--pre-shape-record',str(preset_path),'--pre-shape-sha256',preset_hash]
        if resume_source is not None:
            command+=['--resume-record',str(resume_source),'--resume-sha256',hashlib.sha256(resume_source.read_bytes()).hexdigest()]
        state_path=child/'worker_result.json'
        with (child/'worker_stdout.txt').open('wb') as out:
            process=subprocess.Popen(command,cwd=str(ROOT),stdin=subprocess.DEVNULL,stdout=out,stderr=subprocess.STDOUT)
            handled=set();last=None;since=time.monotonic()
            try:
                while process.poll() is None:
                    if state_path.exists():
                        state=json.loads(state_path.read_bytes());phase=state['phase']
                        if phase!=last:
                            last=phase;since=time.monotonic()
                            if phase=='hand_target_in_progress':print('执行中…',flush=True)
                        if phase=='await_hand_command' and state['permission_index'] not in handled:
                            index=state['permission_index']
                            feedback=state['feedback'];notice=''
                            previous=state.get('last_command_result')
                            if previous:
                                if previous.get('rejected_before_write'):notice=previous.get('reason','未执行')
                                else:
                                    resolution=previous.get('target_resolution') or {}
                                    notice=('已按上限/下限设为 '+str(resolution['applied']) if resolution.get('clamped')
                                        else '已处理' if previous.get('writes') else '目标未变')
                            answer=None;began=time.monotonic()
                            def screen():
                                if sys.stdout.isatty():print('\033[2J\033[H',end='')
                                title='预抓手势' if mode=='pre_shape' else '夹持手势'
                                print(title+' | 已选 '+selected+' '+names[selected],flush=True)
                                for ch,key in enumerate(names):
                                    print(('%s %s  实际 %s / 目标 %s  上限 %s')%(key,names[key],feedback['positions'][ch],feedback['targets'][ch],caps[ch]),flush=True)
                                print('字母选指；数字设目标；也可 d 18000。w调腕 x完成 q退出 ?帮助',flush=True)
                                if notice:print(notice,flush=True)
                            screen()
                            while process.poll() is None and time.monotonic()-began<570:
                                inbox=queue.Queue()
                                def read_command():
                                    try:inbox.put((True,input(selected+'> ')))
                                    except BaseException as exc:inbox.put((False,exc))
                                thread=threading.Thread(target=read_command,daemon=True);thread.start()
                                got=None;idle_handoff=False
                                while time.monotonic()-began<570:
                                    now=json.loads(state_path.read_bytes())
                                    if (now.get('phase')=='completed' and now.get('outcome')=='paused_for_manual_wrist'):
                                        idle_handoff=True
                                        print('\n检测到拖臂，正在退出连接；按回车进入示教续接。',flush=True)
                                        process.wait(timeout=30)
                                        while time.monotonic()-began<570:
                                            try:got=inbox.get(timeout=.2);break
                                            except queue.Empty:continue
                                        break
                                    if now.get('phase')!='await_hand_command' or now.get('permission_index')!=index:
                                        raise RuntimeError('Child changed while awaiting a hand target')
                                    if process.poll() is not None:raise RuntimeError('Child exited before accepting input')
                                    try:got=inbox.get(timeout=.2);break
                                    except queue.Empty:continue
                                if idle_handoff:
                                    if got is None:raise RuntimeError('Input owner not released; no resume permitted')
                                    event('discarded_input_during_manual_handoff',raw=got[1] if got[0] else repr(got[1]))
                                    answer='__idle_handoff__';break
                                if got is None:raise RuntimeError('No current child/input; no stale command sent')
                                if not got[0]:raise got[1]
                                raw=got[1].strip().lower();event('terminal_input',value=raw)
                                if not raw:continue
                                if raw in names:selected=raw;notice='';screen();continue
                                if raw=='?':
                                    notice='目标是绝对计数，不是角度。任意整数自动限制在0到该指上限；输入d 18000只调无名指。'
                                    screen();continue
                                if raw in ('w','x','q'):answer=raw;break
                                match=re.fullmatch(r'([abcde])\s*([+-]?\d+)',raw)
                                if match:selected=match.group(1);token=match.group(2)
                                elif re.fullmatch(r'[+-]?\d+',raw):token=raw
                                else:notice='请输入字母和整数，如 d 18000';screen();continue
                                answer='set '+selected+' '+str(int(token));break
                            if answer is None:raise RuntimeError('No current child/target; no stale input accepted')
                            if answer=='__idle_handoff__':break
                            current=json.loads(state_path.read_bytes())
                            if current.get('outcome')=='paused_for_manual_wrist':
                                process.wait(timeout=30);break
                            if current.get('phase')!='await_hand_command' or current.get('permission_index')!=index:
                                raise RuntimeError('Hand target became stale before permission; not sent')
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
        result['source_result_path']=str(state_path.resolve())
        process=None
        if result.get('exceptions') or result.get('cleanup_errors'):
            print('本次未完成，原始错误已保存在 '+str(child/'worker_result.json'),flush=True)
        return result
    def good(result):
        return (result.get('process_returncode')==0 and not result.get('exceptions')
            and not result.get('cleanup_errors') and result.get('SDK_cleanup_finished') is True)
    def tuning(mode,folder):
        resume_source=None;segments=[]
        for attempt in range(1,101):
            result=worker(mode,folder/('segment_%03d'%attempt),resume_source)
            segments.append(dict(path=result['source_result_path'],outcome=result.get('outcome')))
            report.setdefault('teaching_segments',[]).append(dict(mode=mode,**segments[-1]));save()
            if not good(result) or result.get('outcome')!='paused_for_manual_wrist':
                result['teaching_segments']=segments
                return result
            print('SDK已退出，当前手型保留。现在可拖臂调腕。',flush=True)
            if ask('摆好并退出示教、停稳撤手：s继续调指 / q退出：',('s','q'))=='q':return result
            resume_source=Path(result['source_result_path'])
        raise RuntimeError('Manual teaching segment count exhausted')
    try:
        report['delivery_sha256']=verify_delivery();save()
        print('空手、活动范围清空、退出示教后开始。输入w调腕，等SDK退出后拖臂；若调指空闲时直接拖臂，会暂停保留手型，按回车进入续接。',flush=True)
        if ask('s开始预抓调指 / q退出：',('s','q'))=='q':
            report['outcome']='operator_exit_before_pre_shape'
        else:
            pre=tuning('pre_shape',f/'pre_shape');report['pre_shape']=pre;save()
            if not good(pre) or pre.get('outcome')!='pre_shape_recorded':
                raise RuntimeError('Pre-shape not recorded; no wrist or grip continuation')
            preset_path=Path(pre['source_result_path'])
            preset_hash=hashlib.sha256(preset_path.read_bytes()).hexdigest()
            report['pre_shape_record_sha256']=preset_hash;save()
            for number in range(1,51):
                item=dict(round=number,success=False);report['rounds'].append(item);save()
                base=f/('round_%02d'%number);base.mkdir()
                print('SDK已退出：调整腕姿，空壳落底，预抓保持间隙。',flush=True)
                if ask('退出示教、停稳撤手后 s调夹持 / q退出：',('s','q'))=='q':break
                item['tune']=tuning('grip',base/'grip');save()
                if not good(item['tune']) or item['tune'].get('outcome')!='ready_for_manual_teach_lift':
                    raise RuntimeError('Grip tuning incomplete; no automatic lift/open/retry; raw retained')
                print('SDK已退出：用示教器低速向上试提。',flush=True)
                answer=ask('y已夹稳并离槽 / n没夹紧 / q退出：',('y','n','q'))
                item['manual_lift_operator_verdict']=answer;save()
                if answer=='y':
                    report['physical_grasp_success_observed']=True;item['success']=True
                    report['outcome']='manual_grasp_success_observed_recording';save()
                    print('退出示教、停稳撤手：s保存末态 / q跳过：',flush=True)
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
                        noncontact_pregrasp_reference_qualified=before.get('noncontact_pregrasp_reference_qualified',True),
                        commanded_grip_targets=before['tuned_targets'],pre_manual_lift_hand=before['final_hand'],
                        after_manual_lift_raw=after,configuration=before['observed_run_configuration'],
                        old_left_offsets_reused=False,automatic_motion_or_sort_accepted=False,execution_ready=False)
                    dump(f/'success.json',success);report['success']=success
                    report['outcome']='manual_grasp_success_recorded' if qualified else 'physical_success_reported_current_capture_not_qualified'
                    save();print('已停止调试并记录现场成功，没有自动张开/回臂/投放。',flush=True)
                    if not qualified:print('完整读数未合格，现场成功单独保存，不作自动执行参考。',flush=True)
                    break
                if answer=='q':break
                print('先让空壳由板完全承托，清空五指活动范围并撤手。',flush=True)
                answer=ask('b张开并恢复预抓，再调腕 / q退出：',('b','q'))
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
