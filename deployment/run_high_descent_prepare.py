#!/usr/bin/env python3
"""Actual supervised high50mm descent then empty PREPARE, one result ZIP."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import platform
import signal
import subprocess
import sys
import time
import zipfile

ROOT=Path(__file__).resolve().parents[1]


def installed():
    manifest=ROOT/'deployment/high-descent-prepare-v3-files.sha256'
    hashes={}
    for line in manifest.read_text(encoding='ascii').splitlines():
        digest,name=line.split('  ',1);path=(ROOT/name).resolve()
        if ROOT.resolve() not in path.parents or name in hashes or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
            raise ValueError('Installed trial payload changed: '+name+'; no connection')
        hashes[name]=digest
    return hashes


def permission(index,phase,answer):
    answer=answer.strip().lower()
    expected='m' if index==1 else 'p'
    if answer not in (expected,'q'):raise ValueError('Enter '+expected+' orq')
    row=dict(index=index,phase=phase,value=answer,timestamp_utc=datetime.now(timezone.utc).isoformat())
    if index==1:row['qualified_onsite_full_trial_clear']=answer=='m'
    else:row['actual_arm_stop_and_empty_prepare_sweep_confirmed']=answer=='p'
    return row


def write_permission(folder,row):
    target=folder/'permissions'/('%d.json'%row['index'])
    if target.exists():raise ValueError('A trial permission cannot be repeated')
    temp=target.with_suffix('.tmp')
    temp.write_text(json.dumps(row,ensure_ascii=False)+'\n',encoding='utf8')
    temp.replace(target)


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--confirm-empty-clear-exclusive',action='store_true')
    p.add_argument('--confirm-onsite-stop',action='store_true')
    p.add_argument('--confirm-fixed-scene',action='store_true')
    p.add_argument('--enable-arm-protection',action='store_true',
        help='After onsite m only: enable missing collision8/selfTrue once; keep protection enabled')
    p.add_argument('--sdk-dir',type=Path,default=Path('/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package'))
    p.add_argument('--output-parent',type=Path,default=ROOT/'captures')
    args=p.parse_args(argv)
    if not all((args.execute,args.confirm_empty_clear_exclusive,args.confirm_onsite_stop,args.confirm_fixed_scene)):
        p.error('Requires explicit50mm trial, empty/exclusive control, onsite stop, unchanged scene; no connection')
    if platform.system()!='Linux' or platform.machine()!='aarch64':
        p.error('Execution only on reviewed onsite Linux aarch64; no connection')
    try:hashes=installed()
    except (OSError,ValueError) as exc:p.error(str(exc))
    folder=args.output_parent/('high_descent_prepare_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True,exist_ok=False);(folder/'permissions').mkdir()
    report=dict(schema_version=1,scope='qualified_onsite_empty_hand_high50mm_trial',
        sdk_in_parent=False,installed_file_sha256=hashes,execute_requested=True,
        full_pregrasp_execution_ready=False,automatic_return=False,automatic_open=False,
        exceptions=[],worker_cleanup_timeout=False,onsite_hardware_stop_required_on_contact=True)
    def save():(folder/'parent_result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    def abort():
        path=folder/'abort.json'
        if not path.exists():path.write_text('{"abort":true}\n',encoding='ascii')
    process=None;seen=set();native_hung=False;last_phase=None;phase_began=time.monotonic();save()
    try:
        print('本次有实际动作：当前高位World-Z下降50mm，停稳后一次PREPARE。无回旧点、无接触/抓握/抬升。',flush=True)
        print('绑定本次启动的实际姿态，不要求回量尺旧位。须空手悬空、退出示教，现场全程守急停。先读取完整参数…',flush=True)
        if args.enable_arm_protection:
            print('已选择保护开启：m确认后仅补齐碰撞等级8/自碰撞True，保持开启；回读不符不移动。',flush=True)
        command=[sys.executable,str(ROOT/'deployment/high_descent_prepare_worker.py'),
            '--folder',str(folder),'--sdk-dir',str(args.sdk_dir),'--execute']
        if args.enable_arm_protection:command.append('--enable-arm-protection')
        report['worker_process']=dict(command=command,stdin='DEVNULL',close_fds=True,SDK_in_parent=False)
        with (folder/'process_stdout.log').open('wb') as out,(folder/'process_stderr.log').open('wb') as err:
            process=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=out,stderr=err,close_fds=True)
            while process.poll() is None:
                path=folder/'worker_result.json'
                if path.exists():
                    state=json.loads(path.read_text(encoding='utf8'))
                    phase=state.get('phase');index=state.get('permission_index')
                    if phase!=last_phase:
                        last_phase=phase;phase_began=time.monotonic()
                        if phase in ('preparing_arm_protection','fresh_before_move','moving_high50mm','fresh_before_prepare','preparing_empty_hand'):
                            print('PHASE:',phase,flush=True)
                    limit={'preflight':60,'preparing_arm_protection':25,'fresh_before_move':60,'fresh_before_prepare':60,
                           'preparing_empty_hand':45,'completed':10,'fault':10}.get(phase)
                    if phase=='moving_high50mm':limit=state.get('plan',{}).get('arm_timeout_seconds',45)+5
                    if limit is not None and time.monotonic()-phase_began>limit:
                        raise TimeoutError('Worker phase deadline exceeded: '+str(phase)+'; no force/retry/return')
                    if phase in ('await_move_permission','await_prepare_permission') and index not in seen:
                        seen.add(index)
                        if index==1:
                            plan=state['plan']
                            print('本次实际起点TCP(mm):',[v*1000 for v in plan['start']['pose'][:3]],flush=True)
                            print('目标TCP(m/rad):',plan['target_sdk_pose_m_rad'],flush=True)
                            print('下降50mm，按本机线速度选',plan['speed_percent'],'%，名义',plan['nominal_speed_mm_s'],'mm/s。',flush=True)
                            if args.enable_arm_protection:
                                print('本机保护:',state.get('arm_safety_reads',{}).get('Get_Collision_Stage'),
                                    state.get('arm_safety_reads',{}).get('Get_Self_Collision_Enable'),flush=True)
                            prompt=('现场人员须确认这50mm下降和PREPARE的整臂/手/相机/线缆完整范围均无遮挡、'
                                '到板/空壳/其他物件至少150mm；另一臂静止且在范围外，双手撤开、退出示教、关闭其他控制。'
                                '现场人员须明确臂和手各自的硬件停止及解除驱动力方法。'
                                '固定连接部位不算障碍。'
                                '旧量尺间隙已作废于本次姿态，不沿用360mm。看不全/无法确认就q。'
                                +('m=确认实际停稳，开启缺少的8/True保护并下降一次，q=退出：' if args.enable_arm_protection
                                  else 'm=实际下降一次，q=退出：'))
                        else:
                            print('已观察到臂到位反馈；实际World变化mm:',state.get('actual_world_delta_mm'),flush=True)
                            prompt=('现场确认臂实际已停止、仍空手、完整PREPARE手型范围无遮挡且至少150mm、双手撤开。'
                                'p=执行一次预抓手型，q=留在此处退出；接触/继续运动立即硬件急停：')
                        while True:
                            answer=input(prompt)
                            try:row=permission(index,phase,answer);break
                            except ValueError as exc:print(exc,flush=True)
                        # A child may have aborted during the human wait. Do not
                        # send a permission to a stale/dead worker or wrong phase.
                        current=json.loads(path.read_text(encoding='utf8'))
                        if process.poll() is not None or current.get('phase')!=phase or current.get('permission_index')!=index:
                            raise RuntimeError('Worker changed/aborted during operator input; no permission sent')
                        report.setdefault('operator_inputs',[]).append(row);save()
                        write_permission(folder,row)
                elif time.monotonic()-phase_began>30:
                    raise TimeoutError('Worker produced no startup report; no retry')
                time.sleep(.2)
            report['worker_return_code']=process.returncode
    except (Exception,KeyboardInterrupt) as exc:
        report['exceptions'].append(dict(error=repr(exc)))
        abort()
        print('接触或继续运动：立即现场硬件停止并安全解除手受力。正在通知worker停止后续写入；退出不等于释放。',flush=True)
        if process is not None and process.poll() is None:
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.send_signal(signal.SIGINT)
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    native_hung=True;report['worker_cleanup_timeout']=True
                    print('SDKworker未退出。必须现场硬件停止；没有把杀进程当停机，也不发送重试/张开。',flush=True)
    finally:
        if process is not None:report['worker_return_code']=process.poll()
        path=folder/'worker_result.json'
        if path.exists():report['worker_result']=json.loads(path.read_text(encoding='utf8'))
        outcome=report.get('worker_result',{}).get('outcome','worker_result_missing')
        if native_hung:outcome='SDK_worker_unresponsive_hardware_stop_required'
        report['outcome']=outcome;save()
        bundle=folder.with_suffix('.zip')
        with zipfile.ZipFile(bundle,'x',zipfile.ZIP_DEFLATED) as z:
            for path in sorted(folder.rglob('*')):
                if path.is_file():z.write(path,str(Path(folder.name)/path.relative_to(folder)))
        print('SAVED:',bundle,'OUTCOME:',outcome,flush=True)
        errors=report.get('worker_result',{}).get('exceptions',[])
        if errors:print('ERROR:',errors[-1]['error'],flush=True)
        if outcome!='completed_high50mm_then_PREPARE':
            print('试运行未完成。接触/意外继续运动立即现场硬件停止；手停止或释放未由软件证明。不要自动重试。',flush=True)
        if native_hung:print('该ZIP是仍可能写入的日志快照；原目录保留，worker并未确认停止。',flush=True)
    return 0 if report['outcome']=='completed_high50mm_then_PREPARE' and not report['exceptions'] else 1


if __name__=='__main__':raise SystemExit(main())
