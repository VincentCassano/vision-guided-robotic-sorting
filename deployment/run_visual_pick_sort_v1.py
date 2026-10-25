"""Stdlib parent: accepted visual pregrasp, supervised grip/lift/bin/empty return."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import platform
import signal
import subprocess
import sys
import time
import traceback
import zipfile

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-pregrasp',type=Path,required=True)
    parser.add_argument('--capture-current-target',action='store_true')
    parser.add_argument('--color',choices=('black',),required=True)
    parser.add_argument('--execute',action='store_true',required=True)
    for name in ('empty-inert-shell-exclusive','onsite-stop','fixed-setup'):
        parser.add_argument('--confirm-'+name,action='store_true',required=True)
    parser.add_argument('--sdk-dir',type=Path,default=Path('/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package'))
    args=parser.parse_args()
    if platform.system()!='Linux' or platform.machine()!='aarch64':parser.error('Actual Linux aarch64 robot only')
    from known_cylinder_records import dump
    from visual_pick_sort_source import verify_delivery,accepted_pregrasp,MANIFEST,RECIPE
    from run_school_bin_cycle_v6 import _live_input,permission,operator_prompt,write_permission,SUCCESS
    folder=ROOT/'captures'/datetime.now().strftime('visual_pick_sort_v1_%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True);(folder/'permissions').mkdir();state_path=folder/'worker_result.json'
    report=dict(scope='single_measured_black_shell_visual_pregrasp_to_school_bin_cycle',
        outcome='preflight',execution_ready=False,exceptions=[],SDK_in_parent=False,
        offline_tests_executed=False,source_mode='new_capture' if args.capture_current_target else 'accepted_pregrasp_continuation',
        selected_color=args.color,new_cycle_real_device_accepted=False)
    process=None
    def save():dump(folder/'result.json',report)
    def log(row):
        with (folder/'operator_events.jsonl').open('a',encoding='utf8') as stream:
            stream.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
    try:
        report['delivery_sha256']=verify_delivery();save()
        if args.capture_current_target:
            print('先运行现有v8实时视觉对准：S/Q保存目标，m/d执行，终点必须p才进入夹持。',flush=True)
            before=set((ROOT/'captures').glob('visual_pregrasp_stage_v8_*.zip'))
            command=[sys.executable,str(ROOT/'deployment/run_visual_pregrasp_stage_v8.py'),
                '--capture-current-target','--completed-run',str(args.source_pregrasp.resolve()),
                '--execute','--confirm-empty-clear-exclusive','--confirm-onsite-stop','--confirm-inert-shell',
                '--confirm-fixed-head-board-mount','--sdk-dir',str(args.sdk_dir.resolve())]
            report['visual_pregrasp_command']=command;save()
            captured=subprocess.run(command,cwd=str(ROOT),check=False)
            created=set((ROOT/'captures').glob('visual_pregrasp_stage_v8_*.zip'))-before
            if len(created)==1:
                candidate=created.pop();(folder/'input_pregrasp.zip').write_bytes(candidate.read_bytes())
                args.source_pregrasp=candidate
            if captured.returncode or len(created)>0 or not (folder/'input_pregrasp.zip').exists():
                raise RuntimeError('Fresh visual pregrasp not accepted; no grip process')
        else:(folder/'input_pregrasp.zip').write_bytes(args.source_pregrasp.read_bytes())
        source=accepted_pregrasp(folder/'input_pregrasp.zip');report['accepted_pregrasp']=source;save()
        print('视觉预抓已p确认；当前臂手/空壳/板不能改变。先新鲜核对，不重复下降或张开。',flush=True)
        print('使用已有成功配方：',RECIPE,'；合法status5只允许该指降低一次目标，不加紧。',flush=True)
        print('输入顺序：g夹持；确认夹稳后l提140mm；实测带物净空≥20mm才授权投黑框；u空返/e结束。',flush=True)
        print('无自动故障松手/回臂/重试；状态3/4、超流、拒读阻断。现场全程守急停。',flush=True)
        command=[sys.executable,str(ROOT/'deployment/visual_pick_sort_worker.py'),
            '--folder',str(folder),'--source-pregrasp',str((folder/'input_pregrasp.zip').resolve()),
            '--source-sha256',source['archive_sha256'],'--manifest-sha256',hashlib.sha256(MANIFEST.read_bytes()).hexdigest(),
            '--color',args.color,'--sdk-dir',str(args.sdk_dir.resolve())]
        report['worker_command']=command;save()
        with (folder/'worker_stdout.txt').open('wb') as output:
            process=subprocess.Popen(command,cwd=str(ROOT),stdin=subprocess.DEVNULL,stdout=output,stderr=subprocess.STDOUT)
            handled=set();last=None;began=time.monotonic()
            phases={5:'await_grasp',8:'await_bin_lift',11:'await_bin_transfer',12:'await_empty_bin_return'}
            while process.poll() is None:
                if state_path.exists():
                    state=json.loads(state_path.read_text(encoding='utf8'));phase=state.get('phase')
                    if phase!=last:last=phase;began=time.monotonic();print('PHASE:',phase,flush=True)
                    if str(phase).startswith('await_'):
                        index=state['permission_index']
                        if (index,phase) not in handled:
                            if phases.get(index)!=phase:raise RuntimeError('Unknown permission phase')
                            prompt=operator_prompt(index,state)
                            if index==5:
                                prompt=('确认仍是本次p确认的非接触位，同一空壳全部重量由完整槽承托，'
                                    '手指只接触该空壳，整手闭合范围清空，双手撤开。'
                                    'g=已有成功配方夹持，合法接触status5仅降低该指目标一次；q=退出：')
                            while True:
                                answer=_live_input(prompt,process,state_path,phase,index,log)
                                try:event=permission(index,phase,answer);break
                                except ValueError as exc:print(str(exc),flush=True)
                            current=json.loads(state_path.read_text(encoding='utf8'))
                            if process.poll() is not None or current.get('phase')!=phase or current.get('permission_index')!=index:
                                raise RuntimeError('Stale permission refused')
                            write_permission(folder,event);handled.add((index,phase));began=time.monotonic()
                    if time.monotonic()-began>(600 if str(phase).startswith('await_') else 180):
                        raise TimeoutError('Worker phase deadline: '+str(phase))
                elif time.monotonic()-began>180:raise TimeoutError('Worker result missing')
                time.sleep(.2)
        worker=json.loads(state_path.read_bytes());report['worker_returncode']=process.returncode
        report['outcome']=worker.get('outcome','missing_worker_outcome')
        report['new_cycle_real_device_accepted']=(report['outcome']==SUCCESS and process.returncode==0
            and not worker.get('exceptions') and not worker.get('cleanup_errors'))
        report['current_invocation_write_counts']={key:worker.get(name,0)+(
            source['pregrasp_write_counts'][key] if args.capture_current_target else 0) for key,name in
            (('arm','arm_actuator_writes'),('hand','hand_register_writes'),('configuration','controller_configuration_writes'))}
        report['previous_pregrasp_replayed']=args.capture_current_target
        for error in worker.get('exceptions',[]):print('STOP:',error['error'],flush=True)
    except BaseException as exc:
        report['exceptions'].append(dict(error=repr(exc),traceback=traceback.format_exc()))
        report['outcome']='parent_aborted_no_automatic_retry'
        dump(folder/'abort.json',dict(reason=repr(exc)));print('STOP:',repr(exc),flush=True)
        if process is not None and process.poll() is None:
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                process.send_signal(signal.SIGINT)
                try:process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    report['worker_cleanup_timeout']=True
                    print('SDK子进程尚未退出；不自动杀进程或松手，现场硬件停止处理继续运动。',flush=True)
    finally:
        if state_path.exists():report['worker']=json.loads(state_path.read_bytes())
        if process is not None:report['worker_returncode']=process.poll()
        if report.get('worker_cleanup_timeout'):report['outcome']='worker_unresponsive_hardware_stop_required'
        report['execution_ready']=False;save()
        payload={p.relative_to(folder).as_posix():p.read_bytes()
            for p in folder.rglob('*') if p.is_file() and p!=folder/'source_hashes.json'}
        inventory={name:hashlib.sha256(raw).hexdigest() for name,raw in payload.items()}
        dump(folder/'source_hashes.json',inventory)
        payload['source_hashes.json']=(folder/'source_hashes.json').read_bytes()
        archive=folder.with_suffix('.zip')
        with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
            for name,raw in sorted(payload.items()):z.writestr(folder.name+'/'+name,raw)
        print('SAVED:',archive,'OUTCOME:',report['outcome'],flush=True)
    return 0 if (report['outcome']==SUCCESS and report.get('worker_returncode')==0
        and not report['exceptions'] and not report.get('worker',{}).get('cleanup_errors')) else 1


if __name__=='__main__':raise SystemExit(main())
