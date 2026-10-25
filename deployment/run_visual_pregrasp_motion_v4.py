#!/usr/bin/env python3
"""Fresh supervised empty-hand alignment to a saved live visual target."""
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
import traceback
import zipfile

ROOT=Path(__file__).resolve().parents[1]
MANIFEST=ROOT/'deployment/visual-pregrasp-motion-v4-files.sha256'


def verify_delivery():
    checked={}
    for line in MANIFEST.read_text(encoding='ascii').splitlines():
        expected,name=line.split('  ',1)
        path=(ROOT/name).resolve()
        if (ROOT not in path.parents or name in checked or len(expected)!=64
                or hashlib.sha256(path.read_bytes()).hexdigest()!=expected):
            raise ValueError('Delivery file missing/changed: '+name)
        checked[name]=expected
    if not checked:raise ValueError('Empty delivery inventory')
    return checked


def live_input(prompt,process,path,phase,index,log):
    print(prompt,end='',flush=True);began=time.monotonic()
    while time.monotonic()-began<560:
        if process.poll() is not None:raise RuntimeError('Worker exited; no stale permission')
        state=json.loads(path.read_text(encoding='utf8'))
        if state.get('phase')!=phase or state.get('permission_index')!=index:
            raise RuntimeError('Worker changed/aborted while awaiting input')
        if not select.select([sys.stdin],[],[],.2)[0]:continue
        answer=sys.stdin.readline()
        if answer=='':raise EOFError('Input closed; no permission')
        answer=answer.strip().lower()
        log(dict(event='operator_answer',index=index,phase=phase,value=answer,
            timestamp_utc=datetime.now(timezone.utc).isoformat()))
        return answer
    raise TimeoutError('Onsite permission expired')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    source=parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--vision-archive',type=Path)
    source.add_argument('--capture-current-target',action='store_true')
    parser.add_argument('--position-session',type=Path,default=ROOT/'calibration/position_rgbd_20261020_001259_216986')
    parser.add_argument('--inference-python',default='/usr/bin/python3')
    parser.add_argument('--device',default='auto')
    parser.add_argument('--execute',action='store_true',required=True)
    for name in ('empty-clear-exclusive','onsite-stop','inert-shell','same-shell-scene'):
        parser.add_argument('--confirm-'+name,action='store_true',required=True)
    parser.add_argument('--sdk-dir',type=Path,
        default=Path('/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package'))
    args=parser.parse_args()
    if platform.system()!='Linux' or platform.machine()!='aarch64' or sys.version_info<(3,8):
        parser.error('Actual Linux aarch64 Python3.8+ only; no device connection')
    from known_cylinder_records import dump
    from visual_pregrasp_source import load_target
    folder=ROOT/'captures'/datetime.now().strftime('visual_pregrasp_alignment_v4_%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True);(folder/'permissions').mkdir()
    state_path=folder/'worker_result.json'
    report=dict(scope='one_saved_live_visual_target_actual_empty_hand_pregrasp_alignment',
        outcome='preflight',execution_ready=False,new_target_physical_alignment_accepted=False,
        automatic_grasp=False,automatic_return=False,device_worker_started=False,exceptions=[],offline_tests_executed=False,
        confirmations={k:True for k in ('empty_hand_clear_exclusive','onsite_hardware_stop',
            'inert_shell','same_shell_scene_since_vision_capture')})
    process=None
    def log(row):
        with (folder/'operator_events.jsonl').open('a',encoding='utf8') as stream:
            stream.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
    try:
        report['deployed_source_sha256']=verify_delivery()
        report['configuration_policy']='observe_and_freeze_current_settings_no_payload_setters'
        report['payload_physical_parameters_verified']=False
        if args.capture_current_target:
            print('先取当前视觉目标：点空壳，S采集；显示PLANE TARGET SAVED后按Q，随后进入实际动作预检。',flush=True)
            before=set((ROOT/'captures').glob('visual_task_binding_*.zip'))
            command=[sys.executable,str(ROOT/'deployment/run_visual_task_binding_v4.py'),
                '--position-session',str(args.position_session.resolve()),
                '--confirm-fixed-camera','--confirm-measured-shell-only','--confirm-unchanged-setup',
                '--confirm-upright-equal-height-recesses',
                '--anchor',str(ROOT/'deployment/visual_reference_201634/task_anchor.json'),
                '--inference-python',args.inference_python,'--device',args.device]
            report['live_vision_command']=command;dump(folder/'result.json',report)
            completed=subprocess.run(command,cwd=str(ROOT),check=False)
            report['live_vision_returncode']=completed.returncode
            created=set((ROOT/'captures').glob('visual_task_binding_*.zip'))-before
            if len(created)==1:
                original=next(iter(created)).read_bytes()
                (folder/'input_visual_target.zip').write_bytes(original)
                report['input_visual_target_sha256']=hashlib.sha256(original).hexdigest()
            if completed.returncode!=0 or len(created)!=1:
                raise RuntimeError('本次视觉没有唯一合格目标包；不连接臂手，不使用历史目标补位。')
            args.vision_archive=created.pop()
        if not (folder/'input_visual_target.zip').exists():
            original=args.vision_archive.read_bytes()
            (folder/'input_visual_target.zip').write_bytes(original)
            report['input_visual_target_sha256']=hashlib.sha256(original).hexdigest()
        receipt=load_target(args.vision_archive)
        report['vision_archive_sha256']=receipt['archive_sha256']
        report['target_sha256']=receipt['target_sha256']
        print('使用本次选定的视觉目标，不回旧槽或旧臂姿；动作开始后目标及场景保持不变。',flush=True)
        print('非接触预抓TCP(mm)：',receipt['target']['pregrasp_tcp_candidate_mm'],flush=True)
        print('先只读当前臂手与保护；当前手须空、悬空。采图后空壳/板/头/安装不能改变。',flush=True)
        print('m：准备手型→必要时原位竖直升高→视觉高位对准→140mm悬停；d：非接触下降。负载只读锁定，不辨识或写入。',flush=True)
        print('软件Stop/退出不证明物理停止。意外接触或运动立即现场硬件停止。',flush=True)
        dump(folder/'result.json',report)
        with (folder/'worker_stdout.txt').open('wb') as output:
            process=subprocess.Popen([sys.executable,str(ROOT/'deployment/visual_pregrasp_motion_worker_v4.py'),
                '--folder',str(folder),'--vision-archive',str(args.vision_archive.resolve()),
                '--sdk-dir',str(args.sdk_dir.resolve()),
                '--delivery-manifest-sha256',hashlib.sha256(MANIFEST.read_bytes()).hexdigest(),
                '--execute'],cwd=str(ROOT),stdin=subprocess.DEVNULL,
                stdout=output,stderr=subprocess.STDOUT,close_fds=True)
            report['device_worker_started']=True
            handled=set();last_phase=None;phase_began=time.monotonic()
            while process.poll() is None:
                if state_path.exists():
                    state=json.loads(state_path.read_text(encoding='utf8'))
                    phase=state.get('phase')
                    if phase!=last_phase:
                        last_phase=phase;phase_began=time.monotonic()
                        print('PHASE:',phase,flush=True)
                    if str(phase).startswith('await_'):
                        index=state['permission_index']
                        if index not in handled:
                            if index==1 and phase=='await_visual_high_alignment':
                                print('当前TCP(mm)：',[v*1000 for v in state['current_arm']['pose'][:3]],flush=True)
                                print('原位升高目标(mm)：',[v*1000 for v in state['startup_raise_pose'][:3]],flush=True)
                                print('高位目标(mm)：',[v*1000 for v in state['high_alignment_pose'][:3]],flush=True)
                                print('现有工具配置（只读，不代表物理参数已验证）：',state['observed_run_configuration']['raw']['active_tool_payload_raw'],flush=True)
                                print('悬停目标(mm)：',[v*1000 for v in state['hover_pose'][:3]],flush=True)
                                prompt=('确认空手、整手准备动作范围和机械臂/相机/线缆全段无遮挡；'
                                    '先按当前腕姿原位升高（如需要），再高位MoveJ-P调整位置/腕姿；全部路径须现场可见，'
                                    '现场看不清全段就q。保持独占、退出示教、撤手并守急停。'
                                    'm=准备一次、必要时升高并对准、再降至140mm悬停；q=退出：')
                                expected='m'
                            elif index==2 and phase=='await_visual_final_descent':
                                prompt=('已到新视觉目标上方140mm。确认实际目标在夹持空隙正下方、'
                                    '当前固定腕姿下整段下降至终点均不接触电池/板/邻物；'
                                    '此处不要求手到电池14mm，不能把140mm当已测净空。'
                                    '若偏位、不确定或已接触就q；不要手动拖臂。'
                                    'd=仅沿World-Z下降到非接触预抓位；q=退出：')
                                expected='d'
                            else:raise RuntimeError('Unknown permission phase')
                            answer=live_input(prompt,process,state_path,phase,index,log)
                            if answer not in (expected,'q'):raise ValueError('Unknown input; no motion permission')
                            dump(folder/'permissions'/('%d.json'%index),dict(index=index,phase=phase,value=answer,
                                scene_unchanged_and_path_clear=answer==expected,
                                timestamp_utc=datetime.now(timezone.utc).isoformat()))
                            handled.add(index);phase_began=time.monotonic()
                    limit=600 if str(phase).startswith('await_') else 180
                    if time.monotonic()-phase_began>limit:raise TimeoutError('Worker phase deadline exceeded')
                elif time.monotonic()-phase_began>180:raise TimeoutError('Worker startup deadline exceeded')
                time.sleep(.2)
        worker=json.loads(state_path.read_text(encoding='utf8'))
        report['worker_returncode']=process.returncode;report['worker']=worker
        segment=worker.get('current_segment',{})
        if segment:
            print('LAST SEGMENT:',segment.get('notification',{}),flush=True)
        if worker.get('first_rejected_arm_query_raw') is not None:
            print('FIRST REJECTED ARM RAW:',worker['first_rejected_arm_query_raw'],flush=True)
        if worker.get('latest_arm_query_raw') is not None:
            print('LAST ARM RAW:',worker['latest_arm_query_raw'],flush=True)
        if (process.returncode==0 and worker.get('pregrasp_endpoint_feedback_completed') is True
                and not worker.get('exceptions') and not worker.get('cleanup_errors')
                and worker.get('vision_source',{}).get('archive_sha256')==receipt['archive_sha256']):
            print('SDK已关闭，臂保持终点，手保持PREPARE；不自动夹持。',flush=True)
            answer=input('现场结果：p=电池在拇指与四指夹持空隙内、各处无接触；'
                'n=未对齐/已接触；u=未确认（只记录，无动作）：').strip().lower()
            if answer not in ('p','n','u'):answer='u'
            report['operator_endpoint_verdict']=answer
            report['new_target_physical_alignment_accepted']=answer=='p'
            report['outcome']=('visual_pregrasp_alignment_confirmed_by_operator' if answer=='p'
                else 'endpoint_feedback_completed_physical_alignment_not_accepted')
            log(dict(event='operator_endpoint_verdict',value=answer,no_device_commands=True))
        else:
            report['outcome']='worker_aborted_no_automatic_retry'
            for row in worker.get('exceptions',[])+worker.get('cleanup_errors',[]):
                print('ABORT:',row.get('error',row),'；原始数据已保留，不自动重试。',flush=True)
    except BaseException as exc:
        report['exceptions'].append(dict(error=repr(exc),traceback=traceback.format_exc()))
        report['outcome']='parent_aborted_no_automatic_retry'
        print('ERROR:',repr(exc),flush=True)
        if process is not None and process.poll() is None:
            dump(folder/'abort.json',dict(reason=repr(exc)))
            print('ABORT：现场硬件停止用于接触或继续运动；无自动张开或回臂。',flush=True)
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                process.send_signal(signal.SIGINT)
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    try:process.wait(timeout=3)
                    except subprocess.TimeoutExpired:process.kill();process.wait(timeout=3)
        if state_path.exists():report['worker']=json.loads(state_path.read_text(encoding='utf8'))
    finally:
        report['execution_ready']=False
        if process is not None:report['worker_returncode']=process.poll()
        # Include nested raw model/readiness files and permission events.
        inventory={p.relative_to(folder).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
            for p in folder.rglob('*') if p.is_file() and p not in (folder/'result.json',folder/'source_hashes.json')}
        report['result_file_sha256']=inventory;dump(folder/'result.json',report)
        dump(folder/'source_hashes.json',dict(inventory,**{'result.json':hashlib.sha256((folder/'result.json').read_bytes()).hexdigest()}))
        archive=folder.with_suffix('.zip')
        with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
            for p in sorted(folder.rglob('*')):
                if p.is_file():z.write(p,folder.name+'/'+p.relative_to(folder).as_posix())
        print('SAVED:',archive,'OUTCOME:',report['outcome'],'EXECUTION_READY: False',flush=True)
        print('SHA256:',hashlib.sha256(archive.read_bytes()).hexdigest(),flush=True)
    return 0 if report['new_target_physical_alignment_accepted'] else 1


if __name__=='__main__':raise SystemExit(main())
