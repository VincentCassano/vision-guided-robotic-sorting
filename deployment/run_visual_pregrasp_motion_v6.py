"""Explicit terminal RTU close/reopen then unchanged supervised v4 visual alignment."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import platform
import signal
import subprocess
import sys
import traceback
import zipfile

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vision-archive',type=Path,required=True)
    parser.add_argument('--restart-tool-rs485',action='store_true',required=True)
    parser.add_argument('--execute',action='store_true',required=True)
    for name in ('empty-clear-exclusive','onsite-stop','inert-shell','same-shell-scene'):
        parser.add_argument('--confirm-'+name,action='store_true',required=True)
    args=parser.parse_args()
    if platform.system()!='Linux' or platform.machine()!='aarch64':parser.error('Actual robot Linux aarch64 only')
    from known_cylinder_records import dump
    from visual_bus_reopen_v6_worker import verify_delivery,MANIFEST
    folder=ROOT/'captures'/datetime.now().strftime('visual_pregrasp_alignment_v6_%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True)
    report=dict(scope='one_explicit_terminal_RTU_close_reopen_plus_unchanged_v4_supervised_visual_alignment',
        outcome='preflight',execution_ready=False,exceptions=[],interrupted=False,
        arm_actuator_writes=0,hand_register_writes=0,controller_configuration_writes=0,
        configuration_counts_include_bus_worker_and_motion_worker=True,
        no_automatic_motion_retry=True,offline_tests_executed=False)
    def save():dump(folder/'result.json',report)
    # Keep this wrapper alive to collect the children's normal abort/cleanup.
    # Foreground children receive Ctrl-C themselves; never kill an active SDK
    # worker merely to finish packaging evidence.
    original_handler=signal.getsignal(signal.SIGINT)
    def interrupt(signum,frame):
        report['interrupted']=True;save()
        print('已中断：等待子进程停止/清理；意外继续运动使用现场硬件急停。',flush=True)
    signal.signal(signal.SIGINT,interrupt)
    try:
        report['delivery_sha256']=verify_delivery()
        # Validate the actual target before any communication setting.
        from visual_pregrasp_source import load_target
        receipt=load_target(args.vision_archive)
        report['vision_archive_sha256']=receipt['archive_sha256'];save()
        print('先关闭再开启末端RS485一次，回读0→1；保持115200/100ms。通过后进入现有视觉预抓，不是手复位。',flush=True)
        bus=subprocess.run([sys.executable,str(ROOT/'deployment/visual_bus_reopen_v6_worker.py'),
            '--folder',str(folder),'--restart-tool-rs485','--manifest-sha256',
            hashlib.sha256(MANIFEST.read_bytes()).hexdigest()],cwd=str(ROOT),stdin=subprocess.DEVNULL,check=False)
        bus_report=json.loads((folder/'bus_worker_result.json').read_bytes());report['bus_worker']=bus_report
        report['controller_configuration_writes']=bus_report['controller_configuration_writes'];save()
        print('RS485 CLOSE/OPEN VERIFIED:',bus_report.get('close_reopen_verified'),
            'ORIGINAL MODE RESTORED:',bus_report.get('original_mode_restored_verified'),flush=True)
        for error in bus_report['exceptions']:
            print('BUS ABORT:',error['error'],flush=True)
        if (bus.returncode!=0 or bus_report['exceptions'] or not bus_report.get('close_reopen_verified')
                or report['interrupted']):
            raise RuntimeError('Communication setup aborted; no motion subprocess started')
        before=set((ROOT/'captures').glob('visual_pregrasp_alignment_v4_*.zip'))
        command=[sys.executable,str(ROOT/'deployment/run_visual_pregrasp_motion_v4.py'),
            '--vision-archive',str(args.vision_archive.resolve()),'--execute',
            '--confirm-empty-clear-exclusive','--confirm-onsite-stop','--confirm-inert-shell','--confirm-same-shell-scene']
        report['motion_command']=command;save()
        motion=subprocess.run(command,cwd=str(ROOT),check=False)
        report['motion_parent_returncode']=motion.returncode
        created=set((ROOT/'captures').glob('visual_pregrasp_alignment_v4_*.zip'))-before
        if len(created)!=1:raise RuntimeError('Expected unique new v4 motion result; never replay automatically')
        source=created.pop();raw=source.read_bytes();(folder/'motion_result_v4.zip').write_bytes(raw)
        report['motion_archive_sha256']=hashlib.sha256(raw).hexdigest()
        with zipfile.ZipFile(source) as archive:
            prefix=archive.namelist()[0].split('/')[0]+'/'
            parent=json.loads(archive.read(prefix+'result.json'))
        report['motion_outcome']=parent['outcome']
        report['new_target_physical_alignment_accepted']=parent.get('new_target_physical_alignment_accepted',False)
        worker=parent.get('worker',{})
        for key in ('arm_actuator_writes','hand_register_writes','controller_configuration_writes'):
            report[key]+=worker.get(key,0)
        report['motion_worker_exceptions']=worker.get('exceptions',[])
        report['outcome']='visual_pregrasp_confirmed' if motion.returncode==0 and report['new_target_physical_alignment_accepted'] else 'motion_aborted_or_alignment_not_accepted_no_retry'
    except BaseException as exc:
        report['outcome']='aborted_no_automatic_retry'
        report['exceptions'].append(dict(error=repr(exc),traceback=traceback.format_exc()))
        print('ABORT:',repr(exc),flush=True)
    finally:
        signal.signal(signal.SIGINT,original_handler)
        report['execution_ready']=False;save()
        hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in folder.iterdir() if p.is_file()}
        dump(folder/'source_hashes.json',hashes)
        archive=folder.with_suffix('.zip')
        with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as output:
            for p in sorted(folder.iterdir()):
                if p.is_file():output.write(p,folder.name+'/'+p.name)
        print('SAVED:',archive,'OUTCOME:',report['outcome'],flush=True)
    return 0 if report['outcome']=='visual_pregrasp_confirmed' else 1


if __name__=='__main__':raise SystemExit(main())
