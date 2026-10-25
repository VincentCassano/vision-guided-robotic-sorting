"""Current held-shell transfer and bin drop, stdlib parent; no new lift."""
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
import zipfile
from known_cylinder_records import dump
from run_school_bin_cycle_v6 import _live_input,permission,operator_prompt,write_permission,_phase_deadline

ROOT=Path(__file__).resolve().parents[1]


def archive_folder(folder):
    files=sorted(p for p in folder.rglob('*') if p.is_file())
    dump(folder/'source_hashes.json',{p.relative_to(folder).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files})
    archive=folder.with_suffix('.zip')
    with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
        for path in sorted(p for p in folder.rglob('*') if p.is_file()):
            z.write(path,folder.name+'/'+path.relative_to(folder).as_posix())
    return archive


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--source-report',type=Path,default=ROOT/'deployment/held_transfer_source_123051.json')
    p.add_argument('--confirm-held-shell-exclusive',action='store_true')
    p.add_argument('--confirm-onsite-stop',action='store_true')
    p.add_argument('--sdk-dir',type=Path,default=Path('/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package'))
    a=p.parse_args()
    if not all((a.execute,a.confirm_held_shell_exclusive,a.confirm_onsite_stop)):
        p.error('Already held inert shell, exclusive control and onsite stop confirmations required')
    if platform.system()!='Linux' or platform.machine()!='aarch64':p.error('Onsite Linux aarch64 only')
    lines=(ROOT/'deployment/held-bin-transfer-v1-files.sha256').read_text(encoding='ascii').splitlines()
    if not lines:p.error('Empty payload manifest; no device connection')
    hashes={}
    for line in lines:
        digest,name=line.split('  ',1);target=(ROOT/name).resolve()
        if ROOT not in target.parents or name in hashes or len(digest)!=64 or hashlib.sha256(target.read_bytes()).hexdigest()!=digest:
            p.error('Installed payload differs: '+name+'; no device connection')
        hashes[name]=digest
    folder=ROOT/'captures'/('held_bin_transfer_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True);(folder/'permissions').mkdir()
    report=dict(scope='new_current_held_start_to_bin_no_regrip_or_vertical_command',sdk_in_parent=False,
        source_report=str(a.source_report),installed_file_sha256=hashes,exceptions=[],operator_inputs=[],arm_actuator_writes=0,execution_ready=False)
    process=None;last=None
    def log(row):
        with (folder/'operator_events.jsonl').open('a',encoding='utf8') as stream:
            stream.write(json.dumps(row,ensure_ascii=False)+'\n')
    try:
        command=[sys.executable,str(ROOT/'deployment/held_bin_transfer_worker.py'),
            '--folder',str(folder),'--source-report',str(a.source_report.resolve()),'--sdk-dir',str(a.sdk_dir)]
        print('本次从当前已夹起的空壳位置接投放，不再下降/闭指/再升140mm；先读当前健康手型与静止臂，按提示输入真实持物净空。',flush=True)
        print('其他控制须退出，人员双手撤离夹持区并守急停；非预期接触、明显升温或非受控运动立即现场硬件停止。',flush=True)
        with (folder/'process_stdout.log').open('wb') as out,(folder/'process_stderr.log').open('wb') as err:
            process=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=out,stderr=err,close_fds=True)
            began=time.monotonic()
            while process.poll() is None:
                path=folder/'worker_result.json'
                if not path.exists():
                    if time.monotonic()-began>30:raise TimeoutError('Worker startup record missing')
                    time.sleep(.2);continue
                state=json.loads(path.read_text(encoding='utf8'))
                key=(state.get('phase'),state.get('permission_index'))
                if key!=last:
                    last=key;began=time.monotonic()
                    if key[0] in ('await_bin_transfer','await_empty_bin_return'):
                        if key[1] not in (11,12):raise RuntimeError('Unexpected delivery permission index')
                        prompt=operator_prompt(key[1],state)
                        while True:
                            answer=_live_input(prompt,process,path,key[0],key[1],log)
                            try:row=permission(key[1],key[0],answer);break
                            except ValueError as exc:print(exc,flush=True)
                        report['operator_inputs'].append(row);dump(folder/'parent_result.json',report)
                        write_permission(folder,row)
                    else:print('PHASE:',key[0],flush=True)
                if time.monotonic()-began>_phase_deadline(state.get('phase'),state):raise TimeoutError('Worker phase timeout')
                time.sleep(.2)
        report['worker_return_code']=process.returncode
    except (Exception,KeyboardInterrupt) as exc:
        report['exceptions'].append(repr(exc));dump(folder/'abort.json',dict(abort=True))
        if process is not None and process.poll() is None:
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.send_signal(signal.SIGINT)
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    report['worker_cleanup_timeout']=True
                    print('SDKworker未确认退出；立即现场硬件停止，进程状态不证明停机。',flush=True)
        print('中止后无自动张开或回臂；接触/继续运动/升温使用现场硬件停止。',flush=True)
    finally:
        path=folder/'worker_result.json'
        if path.exists():report['worker_result']=json.loads(path.read_text(encoding='utf8'))
        worker=report.get('worker_result',{})
        for key in ('outcome','arm_actuator_writes','hand_register_writes','configuration_writes',
                'loaded_lift_performed','load_lift_or_transfer_performed','loaded_transfer_performed',
                'bin_release_feedback_completed','shell_in_selected_bin_observed_by_operator',
                'empty_return_completed','active_targets','execution_ready'):
            if key in worker:report[key]=worker[key]
        dump(folder/'parent_result.json',report)
        print('SAVED:',archive_folder(folder),flush=True)
        if report.get('worker_result',{}).get('error'):print('ERROR:',report['worker_result']['error'],flush=True)
    return 0 if not report['exceptions'] and not report.get('worker_cleanup_timeout') and report.get('worker_return_code')==0 else 1


if __name__=='__main__':raise SystemExit(main())
