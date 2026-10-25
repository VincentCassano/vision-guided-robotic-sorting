"""Stdlib parent: continue a freshly matched successful tuned grip into one bin."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import platform
import shlex
import signal
import subprocess
import sys
import time
import zipfile
from known_cylinder_records import dump
from run_school_bin_cycle_v6 import (_live_input,permission,operator_prompt,
    write_permission,_phase_deadline,SUCCESS)

ROOT=Path(__file__).resolve().parents[1]
PHASES={8:'await_bin_lift',11:'await_bin_transfer',12:'await_empty_bin_return'}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--color',required=True,choices=('black','red','white'))
    p.add_argument('--confirm-held-inert-shell-exclusive',action='store_true')
    p.add_argument('--confirm-onsite-stop',action='store_true')
    p.add_argument('--sdk-dir',type=Path,default=Path('/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package'))
    a=p.parse_args()
    if not all((a.execute,a.confirm_held_inert_shell_exclusive,a.confirm_onsite_stop)):
        p.error('Current matched held inert shell/exclusive control/onsite stop required; no connection')
    if platform.system()!='Linux' or platform.machine()!='aarch64':p.error('Onsite Linux aarch64 only')
    hashes={}
    for line in (ROOT/'deployment/tuned-grip-bin-files.sha256').read_text(encoding='ascii').splitlines():
        digest,name=line.split('  ',1);path=(ROOT/name).resolve()
        if (not name or name in hashes or ROOT not in path.parents or len(digest)!=64
                or any(c not in '0123456789abcdef' for c in digest)
                or hashlib.sha256(path.read_bytes()).hexdigest()!=digest):p.error('Installed file differs: '+name)
        hashes[name]=digest
    if not hashes:p.error('Empty payload manifest')
    folder=ROOT/'captures'/('tuned_grip_bin_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True);(folder/'permissions').mkdir()
    report=dict(scope='fresh_matched_tuned_grip_to_school_bin',installed_file_sha256=hashes,
        sdk_in_parent=False,exceptions=[],operator_inputs=[],selected_color=a.color,
        no_prepare_descent_or_reclose=True,execution_ready=False)
    def save():dump(folder/'parent_result.json',report)
    def log(row):
        with (folder/'operator_input_events.jsonl').open('a',encoding='utf8') as stream:
            stream.write(json.dumps(row,ensure_ascii=False)+'\n')
    process=None;seen=set();last=None;began=time.monotonic();save()
    try:
        print('本轮保留已调手型，从当前夹持继续；不张开重抓、不重复下降、不使用视觉坐标。',flush=True)
        print('只适用于081023保存后未移臂/松手/重启的同一空壳；变化会拒绝，不自动对齐旧点。',flush=True)
        print('l=竖直提起140mm；实际离槽且未滑落后量净空，输入≥20mm才授权学校过渡位→所选盒→正常张开。',flush=True)
        print('所选颜色:',a.color,'；现场守急停。非预期碰撞/滑落/非受控运动/明显升温立即硬件停止。',flush=True)
        print('故障不自动松指/回臂/重试；状态3/4与实际超流阻断。q保留现场姿态退出。',flush=True)
        (ROOT/'docs').mkdir(exist_ok=True)
        with (ROOT/'docs/执行命令日志.txt').open('a',encoding='utf8') as stream:
            stream.write(shlex.join([sys.executable,*sys.argv])+'\n')
        command=[sys.executable,str(ROOT/'deployment/tuned_grip_bin_worker.py'),
            '--folder',str(folder),'--sdk-dir',str(a.sdk_dir),'--color',a.color]
        report['worker_process']=dict(command=command,stdin='DEVNULL');save()
        with (folder/'process_stdout.log').open('wb') as out,(folder/'process_stderr.log').open('wb') as err:
            process=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=out,stderr=err,close_fds=True)
            while process.poll() is None:
                path=folder/'worker_result.json'
                if not path.exists():
                    if time.monotonic()-began>30:raise TimeoutError('Startup result missing')
                    time.sleep(.2);continue
                state=json.loads(path.read_text(encoding='utf8'));phase=state.get('phase');index=state.get('permission_index')
                if phase!=last:
                    last=phase;began=time.monotonic()
                    if not str(phase).startswith('await_'):print('PHASE:',phase,flush=True)
                if time.monotonic()-began>_phase_deadline(phase,state):raise TimeoutError('Worker phase expired: '+str(phase))
                if str(phase).startswith('await_') and (index,phase) not in seen:
                    if PHASES.get(index)!=phase:raise RuntimeError('Unexpected phase; no prepare/teach/close permission')
                    seen.add((index,phase));prompt=operator_prompt(index,state)
                    while True:
                        answer=_live_input(prompt,process,path,phase,index,log)
                        try:row=permission(index,phase,answer);break
                        except ValueError as exc:print(exc,flush=True)
                    current=json.loads(path.read_text(encoding='utf8'))
                    if process.poll() is not None or current.get('phase')!=phase or current.get('permission_index')!=index:
                        raise RuntimeError('Stale operator permission refused')
                    report['operator_inputs'].append(row);save();write_permission(folder,row)
                time.sleep(.2)
        report['worker_return_code']=process.returncode
    except (Exception,KeyboardInterrupt) as exc:
        report['exceptions'].append(dict(error=repr(exc)));dump(folder/'abort.json',dict(abort=True))
        print('已中止；无自动松手/回臂。现场急停处理接触、滑落或继续运动。',flush=True)
        if process is not None and process.poll() is None:
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.send_signal(signal.SIGINT)
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:report['worker_cleanup_timeout']=True
    finally:
        if process is not None:report['worker_return_code']=process.poll()
        path=folder/'worker_result.json'
        if path.exists():report['worker_result']=json.loads(path.read_text(encoding='utf8'))
        worker=report.get('worker_result',{});report['outcome']=worker.get('outcome','worker_result_missing')
        if report.get('worker_cleanup_timeout'):report['outcome']='SDK_worker_unresponsive_hardware_stop_required'
        save()
        files=sorted(p for p in folder.rglob('*') if p.is_file())
        dump(folder/'source_hashes.json',{p.relative_to(folder).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files})
        bundle=folder.with_suffix('.zip')
        with zipfile.ZipFile(bundle,'x',zipfile.ZIP_DEFLATED) as archive:
            for path in sorted(p for p in folder.rglob('*') if p.is_file()):
                archive.write(path,folder.name+'/'+path.relative_to(folder).as_posix())
        print('SAVED:',bundle,'OUTCOME:',report['outcome'],flush=True)
        if worker.get('exceptions'):print('ERROR:',worker['exceptions'][-1]['error'],flush=True)
    return 0 if report.get('outcome')==SUCCESS and report.get('worker_return_code')==0 and not report['exceptions'] and not report.get('worker_result',{}).get('cleanup_errors') else 1


if __name__=='__main__':raise SystemExit(main())
