"""Empty vertical retreat, fresh visual target, image-left2.8mm pregrasp."""
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
    parser.add_argument('--completed-run',type=Path,required=True)
    parser.add_argument('--previous-vision-archive',type=Path,required=True)
    parser.add_argument('--restart-tool-rs485',action='store_true',required=True)
    parser.add_argument('--execute',action='store_true',required=True)
    for name in ('empty-clear-exclusive','onsite-stop','inert-shell','fixed-head-board-mount','shell-removed-for-retreat'):
        parser.add_argument('--confirm-'+name,action='store_true',required=True)
    args=parser.parse_args()
    if platform.system()!='Linux' or platform.machine()!='aarch64':parser.error('Actual Linux aarch64 robot only')
    from known_cylinder_records import dump
    from run_visual_pregrasp_stage_v7 import verify_delivery
    from visual_image_left_correction import completed_source
    from visual_pregrasp_source import load_target
    from visual_bus_reopen_v6_worker import MANIFEST as BUS_MANIFEST
    folder=ROOT/'captures'/datetime.now().strftime('visual_pregrasp_alignment_v7_%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True)
    report=dict(scope='completed_empty_endpoint_vertical_retreat_fresh_vision_image_left2p8mm_alignment',
        outcome='preflight',execution_ready=False,exceptions=[],interrupted=False,
        arm_actuator_writes=0,hand_register_writes=0,controller_configuration_writes=0,
        grasp_performed=False,no_automatic_motion_retry=True,offline_tests_executed=False,
        operator_confirmations=dict(empty_hand=True,exclusive_control=True,onsite_stop=True,
            inert_shell=True,fixed_head_board_mount=True,shell_removed_before_retreat=True))
    def save():dump(folder/'result.json',report)
    def interrupt(signum,frame):
        report['interrupted']=True;save()
        print('已中断，等待子进程清理；意外继续运动现场硬件停止。',flush=True)
    original_handler=signal.signal(signal.SIGINT,interrupt)
    try:
        report['delivery_sha256']=verify_delivery()
        report['completed_source']=completed_source(args.completed_run)
        report['previous_vision_sha256']=load_target(args.previous_vision_archive)['archive_sha256'];save()
        print('空壳已移走：先核对并竖直退高，再重新采图；不在原低位加横向偏移。',flush=True)
        bus=subprocess.run([sys.executable,str(ROOT/'deployment/visual_bus_reopen_v6_worker.py'),
            '--folder',str(folder),'--restart-tool-rs485','--manifest-sha256',
            hashlib.sha256(BUS_MANIFEST.read_bytes()).hexdigest()],cwd=str(ROOT),stdin=subprocess.DEVNULL,check=False)
        bus_report=json.loads((folder/'bus_worker_result.json').read_bytes());report['bus_worker']=bus_report
        report['controller_configuration_writes']=bus_report['controller_configuration_writes'];save()
        for error in bus_report['exceptions']:print('BUS STOP:',error['error'],flush=True)
        if bus.returncode or bus_report['exceptions'] or not bus_report['close_reopen_verified'] or report['interrupted']:
            raise RuntimeError('Bus reopen failed/interrupted; no actuator subprocess')
        common=['--execute','--completed-run',str(args.completed_run.resolve()),
            '--confirm-empty-clear-exclusive','--confirm-onsite-stop','--confirm-inert-shell','--confirm-fixed-head-board-mount']
        def stage(label,options):
            before=set((ROOT/'captures').glob('visual_pregrasp_stage_v7_*.zip'))
            command=[sys.executable,str(ROOT/'deployment/run_visual_pregrasp_stage_v7.py')]+common+options
            report[label+'_command']=command;save()
            process=subprocess.run(command,cwd=str(ROOT),check=False)
            created=set((ROOT/'captures').glob('visual_pregrasp_stage_v7_*.zip'))-before
            if len(created)!=1:raise RuntimeError('Expected unique '+label+' result; no automatic replay')
            source=created.pop();raw=source.read_bytes();(folder/(label+'_result.zip')).write_bytes(raw)
            with zipfile.ZipFile(source) as z:
                prefix=z.namelist()[0].split('/')[0]+'/'
                result=json.loads(z.read(prefix+'result.json'))
            report[label+'_archive_sha256']=hashlib.sha256(raw).hexdigest()
            report[label+'_outcome']=result['outcome']
            worker=result.get('worker',{})
            for key in ('arm_actuator_writes','hand_register_writes','controller_configuration_writes'):
                report[key]+=worker.get(key,0)
            report[label+'_worker_errors']=worker.get('exceptions',[])+worker.get('cleanup_errors',[]);save()
            if process.returncode or report['interrupted']:raise RuntimeError(label+' stopped; no automatic retry')
            return result
        retreat=stage('retreat',['--retract-only','--vision-archive',str(args.previous_vision_archive.resolve())])
        if retreat.get('clearance_retract_completed') is not True:raise RuntimeError('Vertical retreat not completed')
        print('退高完成。现在把同一空壳放到本次想抓的同高完整槽，落底、撤手。不要动头/板/底座或手安装。',flush=True)
        answer=input('准备好输入v打开实时视觉；随后点空壳，S采图，看到PLANE TARGET SAVED后Q：').strip().lower()
        if answer!='v' or report['interrupted']:raise RuntimeError('Fresh target placement not confirmed; no alignment')
        aligned=stage('fresh_alignment',['--capture-current-target'])
        report['new_target_physical_alignment_accepted']=aligned.get('new_target_physical_alignment_accepted',False)
        report['grasp_offset']=aligned.get('grasp_offset')
        report['outcome']='corrected_fresh_visual_pregrasp_confirmed_no_grasp'
    except BaseException as exc:
        report['outcome']='stopped_no_automatic_retry'
        report['exceptions'].append(dict(error=repr(exc),traceback=traceback.format_exc()))
        print('STOP:',repr(exc),flush=True)
    finally:
        signal.signal(signal.SIGINT,original_handler);report['execution_ready']=False;save()
        inventory={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in folder.iterdir() if p.is_file()}
        dump(folder/'source_hashes.json',inventory)
        archive=folder.with_suffix('.zip')
        with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
            for p in sorted(folder.iterdir()):
                if p.is_file():z.write(p,folder.name+'/'+p.name)
        print('SAVED:',archive,'OUTCOME:',report['outcome'],flush=True)
    return 0 if report['outcome']=='corrected_fresh_visual_pregrasp_confirmed_no_grasp' else 1


if __name__=='__main__':raise SystemExit(main())
