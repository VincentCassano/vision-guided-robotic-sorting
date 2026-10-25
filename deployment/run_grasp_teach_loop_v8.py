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

class OperatorExit(Exception):
    pass


ROOT=Path(__file__).resolve().parents[1]
MANIFEST=ROOT/'deployment/grasp-teach-loop-v8-files.sha256'


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
    f=ROOT/'captures'/datetime.now().strftime('grasp_teach_loop_v8_%Y%m%d_%H%M%S_%f');f.mkdir(parents=True)
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
    def worker(mode,folder,resume_source=None,fresh_entry=False,prepare_first=False):
        nonlocal process,child,selected
        child=folder;child.mkdir(parents=True,exist_ok=True);(child/'permissions').mkdir(exist_ok=True)
        command=[sys.executable,str(ROOT/'deployment/grasp_teach_loop_v8_worker.py'),'--mode',mode,
            '--folder',str(child),'--sdk-dir',str(a.sdk_dir.resolve()),
            '--manifest-sha256',hashlib.sha256(MANIFEST.read_bytes()).hexdigest()]
        if mode in ('grip','restore'):
            command+=['--pre-shape-record',str(preset_path),'--pre-shape-sha256',preset_hash]
        if fresh_entry:command.append('--fresh-manual-entry')
        if prepare_first:command.append('--prepare-first')
        if resume_source is not None:
            command+=['--resume-record',str(resume_source),'--resume-sha256',hashlib.sha256(resume_source.read_bytes()).hexdigest()]
        state_path=child/'worker_result.json'
        supervisor_error=None;input_owner=None;exit_during_fault=False
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
                                    if previous.get('actual_target_arrival_qualified') is False:
                                        notice+='；未到位，可继续调'
                                    if previous.get('observation_timed_out') or previous.get('feedback_stable') is False:
                                        notice+='；尚未确认静稳，由你决定下一步'
                            answer=None;began=time.monotonic()
                            def screen():
                                if sys.stdout.isatty():print('\033[2J\033[H',end='')
                                title='预抓手势' if mode=='pre_shape' else '夹持手势'
                                print(title+' | 已选 '+selected+' '+names[selected],flush=True)
                                for ch,key in enumerate(names):
                                    print(('%s %s  实际 %s / 目标 %s  上限 %s')%(key,names[key],feedback['positions'][ch],feedback['targets'][ch],caps[ch]),flush=True)
                                print('字母选指；数字设目标；d 18000设值，d +500加弯、d -500退。w调腕 x保存 q退出',flush=True)
                                if notice:print(notice,flush=True)
                            screen()
                            while process.poll() is None:
                                inbox=queue.Queue()
                                def read_command():
                                    try:inbox.put((True,input(selected+'> ')))
                                    except BaseException as exc:inbox.put((False,exc))
                                thread=threading.Thread(target=read_command,daemon=True);thread.start();input_owner=thread
                                got=None;idle_handoff=False
                                while True:
                                    now=json.loads(state_path.read_bytes())
                                    if (now.get('phase')=='completed' and now.get('outcome')=='paused_for_manual_wrist'):
                                        idle_handoff=True
                                        print('\n检测到拖臂，正在退出连接；按回车进入示教续接。',flush=True)
                                        process.wait(timeout=30)
                                        while True:
                                            try:got=inbox.get(timeout=.2);break
                                            except queue.Empty:continue
                                        break
                                    if now.get('phase')=='fault' or (now.get('phase')=='completed' and now.get('outcome')!='paused_for_manual_wrist'):
                                        print('\n当前连接已暂停，按回车查看状态。',flush=True)
                                        while True:
                                            try:got=inbox.get(timeout=.2);break
                                            except queue.Empty:continue
                                        process.wait(timeout=30)
                                        exit_during_fault=bool(got and got[0] and str(got[1]).strip().lower()=='q')
                                        answer='__child_fault__';break
                                    if now.get('phase')!='await_hand_command' or now.get('permission_index')!=index:
                                        raise RuntimeError('Child changed while awaiting a hand target')
                                    if process.poll() is not None:
                                        print('\n连接已结束，按回车查看当前状态。',flush=True)
                                        while True:
                                            try:got=inbox.get(timeout=.2);break
                                            except queue.Empty:continue
                                        exit_during_fault=bool(got and got[0] and str(got[1]).strip().lower()=='q')
                                        answer='__child_fault__';break
                                    try:got=inbox.get(timeout=.2);break
                                    except queue.Empty:continue
                                if answer=='__child_fault__':break
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
                                    notice='数字/d 18000设绝对值；d +500或d -500从当前实际加减，步幅自定，0到65535限幅。'
                                    screen();continue
                                if raw in ('w','x','q'):answer=raw;break
                                match=re.fullmatch(r'([abcde])\s*([+-]?\d+)',raw)
                                if match:selected=match.group(1);token=match.group(2)
                                elif re.fullmatch(r'[+-]?\d+',raw):token=raw
                                else:notice='请输入字母和整数，如 d 18000';screen();continue
                                answer=('step ' if match and token.startswith(('+','-')) else 'set ')+selected+' '+str(int(token));break
                            if answer is None:raise RuntimeError('No current child/target; no stale input accepted')
                            if answer in ('__idle_handoff__','__child_fault__'):break
                            current=json.loads(state_path.read_bytes())
                            if current.get('outcome')=='paused_for_manual_wrist':
                                process.wait(timeout=30);break
                            if (current.get('phase')!='await_hand_command' or current.get('permission_index')!=index
                                    or process.poll() is not None):
                                event('discarded_input_after_child_changed',raw=answer,current_phase=current.get('phase'))
                                raise RuntimeError('Child state changed before permission; old input discarded, no write sent')
                            event('hand_command',index=index,command=answer)
                            dump(child/'permissions'/('%d.json'%index),dict(index=index,command=answer,
                                only_intended_shell_contact_and_digit_sweep_clear=True))
                            handled.add(index);since=time.monotonic()
                        if phase!='await_hand_command' and time.monotonic()-since>150:
                            raise TimeoutError('SDK child phase deadline: '+phase)
                    elif time.monotonic()-since>90:raise TimeoutError('SDK child did not produce state')
                    time.sleep(.2)
            except BaseException as exc:
                supervisor_error=repr(exc)
                dump(child/'abort.json',dict(abort=True,reason=supervisor_error))
                try:process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    process.send_signal(signal.SIGINT)
                    try:process.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        report['SDK_child_unresponsive']=True
                        print('子进程尚未确认退出；禁止手动示教或继续本程序，意外运动现场硬件停止。',flush=True)
                if input_owner is not None and input_owner.is_alive():
                    print('按回车结束本次输入，保留当前手指目标。',flush=True)
                    input_owner.join()
        try:result=json.loads(state_path.read_bytes())
        except (OSError,ValueError) as exc:
            result=dict(phase='fault',outcome='child_result_unavailable',SDK_cleanup_finished=False,
                exceptions=[dict(error=repr(exc))],cleanup_errors=[])
        result['process_returncode']=process.poll()
        result['source_result_path']=str(state_path.resolve())
        result['operator_exit_during_fault']=exit_during_fault
        if supervisor_error:
            result.setdefault('exceptions',[]).append(dict(error=supervisor_error,source='parent_supervisor'))
            dump(child/'parent_supervision.json',dict(error=supervisor_error,
                child_returncode=process.poll(),SDK_cleanup_finished=result.get('SDK_cleanup_finished')))
        if process.poll() is None:result['SDK_cleanup_finished']=False
        else:process=None
        if result.get('exceptions') or result.get('cleanup_errors'):
            print('本次未完成，原始错误已保存在 '+str(child/'worker_result.json'),flush=True)
        return result
    def good(result):
        return (result.get('process_returncode')==0 and not result.get('exceptions')
            and not result.get('cleanup_errors') and result.get('SDK_cleanup_finished') is True)
    def reconnect_choice(result):
        nonlocal process
        errors=result.get('exceptions') or []
        if errors:print('本段已暂停：'+str(errors[-1].get('error',''))[:220],flush=True)
        while result.get('SDK_cleanup_finished') is not True:
            if result.get('process_returncode') is not None:
                print('子进程已退出，SDK清理记录不完整；r仅重开只读核对，不代表电机已停止。',flush=True)
                break
            choice=ask('连接清理未确认。w查看退出状态 / q保存退出：',('w','q'))
            if choice=='q':raise OperatorExit()
            if process is not None and process.poll() is not None:
                result['process_returncode']=process.poll();process=None
            try:
                latest=json.loads(Path(result['source_result_path']).read_bytes())
                if process is None and latest.get('SDK_cleanup_finished') is True:
                    result['SDK_cleanup_finished']=True;break
            except (OSError,ValueError):pass
            print('仍未确认连接清理，不开启第二个控制连接。',flush=True)
        return ask('r重新读取当前状态进入手动调指（不重发旧指令） / q保存退出：',('r','q'))
    def tuning(mode,folder,fresh_entry=False,prepare_first=False):
        resume_source=None;segments=[];attempt=0
        while True:
            attempt+=1
            result=worker(mode,folder/('segment_%03d'%attempt),resume_source,fresh_entry,prepare_first)
            prepare_first=False
            segments.append(dict(path=result['source_result_path'],outcome=result.get('outcome')))
            report.setdefault('teaching_segments',[]).append(dict(mode=mode,**segments[-1]));save()
            if not good(result):
                if result.get('operator_exit_during_fault'):raise OperatorExit()
                commands=result.get('operator_commands') or []
                if commands and commands[-1].get('command')=='q':raise OperatorExit()
                if reconnect_choice(result)=='q':raise OperatorExit()
                resume_source=None;fresh_entry=True
                continue
            if result.get('outcome')!='paused_for_manual_wrist':
                result['teaching_segments']=segments
                return result
            print('SDK已退出，当前目标保留；退出连接不停止电机，确认现场停稳后调腕。',flush=True)
            if ask('摆好并退出示教、停稳撤手：s继续调指 / q退出：',('s','q'))=='q':raise OperatorExit()
            resume_source=Path(result['source_result_path']);fresh_entry=False
    try:
        report['delivery_sha256']=verify_delivery();save()
        print('先恢复五指预抓，再现场逐指调弯；无需提前填写最终目标。对掌保持当前值，机械臂不自动移动。',flush=True)
        if ask('确认手空、五指范围清空，退出示教并撤手：s恢复预抓并调指 / q退出：',('s','q'))=='q':
            report['outcome']='operator_exit_before_pre_shape'
        else:
            pre=tuning('pre_shape',f/'pre_shape',prepare_first=True);report['pre_shape']=pre;save()
            if not good(pre) or pre.get('outcome')!='pre_shape_recorded':
                raise RuntimeError('Pre-shape not recorded; no wrist or grip continuation')
            preset_path=Path(pre['source_result_path'])
            preset_hash=hashlib.sha256(preset_path.read_bytes()).hexdigest()
            report['pre_shape_record_sha256']=preset_hash;save()
            number=0
            while True:
                number+=1
                item=dict(round=number,success=False);report['rounds'].append(item);save()
                base=f/('round_%02d'%number);base.mkdir()
                print('SDK已退出：需调腕时先确认手指停稳、空壳可靠承托，保持预抓间隙。',flush=True)
                if ask('退出示教、停稳撤手后 s调夹持 / q退出：',('s','q'))=='q':break
                item['tune']=tuning('grip',base/'grip',fresh_entry=True);save()
                if not good(item['tune']) or item['tune'].get('outcome')!='ready_for_manual_teach_lift':
                    raise RuntimeError('Grip tuning incomplete; no automatic lift/open/retry; raw retained')
                print('SDK已退出；先现场确认手指停稳，再用示教器低速试提。',flush=True)
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
                opened=item['open'].get('last_command_result',{})
                if (not good(item['open']) or opened.get('actual_target_arrival_qualified') is not True
                        or opened.get('feedback_stable') is not True):
                    print('未确认张开，不自动接恢复；r仅重新读取后手动调整。',flush=True)
                    if reconnect_choice(item['open'])=='q':raise OperatorExit()
                    item['manual_after_incomplete_open']=tuning('grip',base/'open_manual',fresh_entry=True);save()
                    continue
                item['restore']=worker('restore',base/'restore');save()
                restored_again=item['restore'].get('last_command_result',{})
                if (not good(item['restore']) or restored_again.get('actual_target_arrival_qualified') is not True
                        or restored_again.get('feedback_stable') is not True):
                    print('恢复未确认到位；r仅重新读取后手动调整。',flush=True)
                    if reconnect_choice(item['restore'])=='q':raise OperatorExit()
                    item['manual_after_incomplete_restore']=tuning('grip',base/'restore_manual',fresh_entry=True);save()
                    continue
                print('SDK已退出，已恢复本次预抓手势。下一轮直接调整腕姿，不重建旧手型。',flush=True)
        if report['outcome']=='started':report['outcome']='stopped_without_grasp_success'
    except OperatorExit:
        report['outcome']='operator_exit_preserving_current_targets';save()
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
