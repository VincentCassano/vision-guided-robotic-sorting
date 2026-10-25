"""Parent with terminal input only: supported thumb/ring adjustment, no arm move."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
import time
import zipfile
from known_cylinder_records import dump
from run_school_bin_cycle_v6 import _live_input

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
    p.add_argument('--source-report',type=Path,default=ROOT/'deployment/grip_tune_source_075643.json')
    p.add_argument('--recover-ring-contact',action='store_true')
    p.add_argument('--confirm-shell-supported-exclusive',action='store_true')
    p.add_argument('--confirm-onsite-stop',action='store_true')
    p.add_argument('--sdk-dir',type=Path,default=Path('/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package'))
    a=p.parse_args()
    if not all((a.execute,a.recover_ring_contact,a.confirm_shell_supported_exclusive,a.confirm_onsite_stop)):
        p.error('Supported inert shell, exclusive control and onsite stop confirmations required')
    if platform.system()!='Linux' or platform.machine()!='aarch64':p.error('Onsite Linux aarch64 only')
    lines=(ROOT/'deployment/supported-grip-tune-v2-files.sha256').read_text(encoding='ascii').splitlines()
    if not lines:p.error('Empty payload manifest; no device connection')
    hashes={}
    for line in lines:
        digest,name=line.split('  ',1);target=(ROOT/name).resolve()
        if (ROOT not in target.parents or name in hashes or len(digest)!=64
                or any(c not in '0123456789abcdef' for c in digest)
                or hashlib.sha256(target.read_bytes()).hexdigest()!=digest):
            p.error('Installed payload differs: '+name+'; no device connection')
        hashes[name]=digest
    folder=ROOT/'captures'/('supported_grip_tune_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True);(folder/'permissions').mkdir()
    report=dict(scope='supported_manual_thumb_ring_tune_no_arm_motion',sdk_in_parent=False,
        source_report=str(a.source_report),installed_file_sha256=hashes,
        exceptions=[],operator_inputs=[],arm_actuator_writes=0,execution_ready=False)
    process=None;last=None
    def log(row):
        with (folder/'operator_events.jsonl').open('a',encoding='utf8') as stream:
            stream.write(json.dumps(row,ensure_ascii=False)+'\n')
    try:
        command=[sys.executable,str(ROOT/'deployment/supported_grip_tune_worker.py'),
            '--folder',str(folder),'--source-report',str(a.source_report.resolve()),'--sdk-dir',str(a.sdk_dir),
            '--recover-ring-contact']
        print('本轮不移臂、不提起、不投放；只调整仍由原完整槽承托的空壳握持。',flush=True)
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
                    if key[0] in ('await_grip_tune','await_ring_withdrawal'):
                        print('当前目标:',state['active_targets'],'；禁止再加弯的通道:',state.get('blocked_closure_channels',[]),flush=True)
                        prompt=('t=拇指+1000（上限30000）；f=无名指+250（上限22000）；'
                            's=确认贴合后记录手型并结束（不证明持力）；o=确认仍完整承托且张开范围无遮挡，退回PREPARE并结束；q=保留姿态退出：')
                        choices=('t','f','s','o','q')
                        if key[0]=='await_ring_withdrawal':
                            prompt=('确认空壳仍完整落底承托、退让范围无遮挡、双手退出并守急停。'
                                'r=只将无名指22500退至21250，拇指17500及其他通道保持；q=保留姿态退出：')
                            choices=('r','q')
                        while True:
                            answer=_live_input(prompt,process,path,key[0],key[1],log).strip().lower()
                            if answer in choices:break
                            print('请输入'+ '/'.join(choices)+'。',flush=True)
                        row=dict(index=key[1],phase=key[0],value=answer,
                            shell_fully_supported=True,hand_swept_volume_clear=True,onsite_stop_ready=True)
                        report['operator_inputs'].append(row);dump(folder/'parent_result.json',report)
                        dump(folder/'permissions'/('%d.json'%key[1]),row)
                    else:print('PHASE:',key[0],flush=True)
                if time.monotonic()-began>150:raise TimeoutError('Worker phase timeout')
                time.sleep(.2)
        report['worker_return_code']=process.returncode
    except (Exception,KeyboardInterrupt) as exc:
        report['exceptions'].append(repr(exc));dump(folder/'abort.json',dict(abort=True))
        if process is not None and process.poll() is None:
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.terminate();report['worker_termination_does_not_prove_hand_stop']=True
        print('中止后无自动张开或回臂；接触/继续运动/升温使用现场硬件停止。',flush=True)
    finally:
        path=folder/'worker_result.json'
        if path.exists():report['worker_result']=json.loads(path.read_text(encoding='utf8'))
        dump(folder/'parent_result.json',report)
        print('SAVED:',archive_folder(folder),flush=True)
        if report.get('worker_result',{}).get('error'):print('ERROR:',report['worker_result']['error'],flush=True)
    return 0 if not report['exceptions'] and report.get('worker_return_code')==0 else 1


if __name__=='__main__':raise SystemExit(main())
