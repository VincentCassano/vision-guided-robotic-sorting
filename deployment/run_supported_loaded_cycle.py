#!/usr/bin/env python3
"""Stdlib-only parent for one supervised, freshly taught same-slot low lift."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
import math
from pathlib import Path
import platform
import select
import shlex
import signal
import subprocess
import sys
import time
import zipfile


ROOT=Path(__file__).resolve().parents[1]
MANIFEST='deployment/supported-low-lift-v4-files.sha256'
SUCCESS='completed_supported_low_lift_return_release'
SUPPORTED_ONLY='completed_supported_release_lift_not_performed'
OPEN_ONLY='completed_supported_opening_empty_return_blocked'
PHASES={0:('await_prepare',('h',)),1:('await_goal',('t',)),
    2:('await_empty_lift',('u',)),4:('await_descend',('d',)),
    5:('await_grasp',('g',)),8:('await_loaded_trial',('l','s')),
    6:('await_supported_release',('r','n')),7:('await_empty_return',('u',))}


def installed():
    hashes={}
    for line in (ROOT/MANIFEST).read_text(encoding='ascii').splitlines():
        if not line:continue
        digest,name=line.split('  ',1)
        path=(ROOT/name).resolve()
        if (len(digest)!=64 or any(c not in '0123456789abcdef' for c in digest)
                or not name or ROOT not in path.parents or name in hashes
                or hashlib.sha256(path.read_bytes()).hexdigest()!=digest):
            raise ValueError('Installed low-lift payload differs: '+name+'; no connection')
        hashes[name]=digest
    if not hashes:raise ValueError('Empty low-lift manifest; no connection')
    return hashes


def permission(index,phase,answer):
    expected,values=PHASES[index]
    if phase!=expected:raise ValueError('Wrong low-lift phase')
    value=answer.strip().lower()
    row={}
    if index==1 and value!='q':
        try:gap=float(value)
        except ValueError:raise ValueError('输入当前手部最低点到板顶的垂直间隙mm，或q')
        if not math.isfinite(gap) or gap<10:
            raise ValueError('到板顶的垂直间隙须至少10mm；各指不得接触电池；不确定就q')
        value='t';row['lowest_hand_to_board_mm']=gap
    if value not in values+('q',):raise ValueError('输入 '+('/'.join(values))+' 或q')
    yes=value!='q'
    row.update(index=index,phase=phase,value=value,timestamp_utc=datetime.now(timezone.utc).isoformat())
    if index==0:row['empty_hand_high_clear_confirmed']=yes
    elif index==1:
        row.update(noncontact_same_supported_shell=yes,battery_in_grasp_space_no_contact=yes,
            teaching_exited_and_hands_removed=yes)
    elif index==2:
        row.update(whole_path_clear_and_actual_stop=yes,onsite_world_Z_corridor_rehearsed_noncontact=yes)
    elif index==4:row['same_corridor_scene_and_stop_confirmed']=yes
    elif index==5:
        row.update(only_intended_shell_contact_and_closure_clear=yes,
            current_noncontact_pose_and_same_shell_scene_confirmed=yes,
            freeze_supported_contact_while_shell_supported=yes)
    elif index==8:
        row['supervised_same_slot_20mm_lift_and_return_clear']=value=='l'
        row['thumb_and_opposed_index_or_middle_contact_observed']=value=='l'
        row['no_clicking_continued_motion_or_heating_observed']=value=='l'
    elif index==6:
        row.update(shell_fully_supported_and_prepare_release_clear=yes,
            shell_actually_lifted_clear_and_lowered=value=='r')
    elif index==7:row['shell_on_board_hand_empty_and_corridor_clear']=yes
    return row


def write_permission(folder,row):
    path=folder/'permissions'/('%d.json'%row['index'])
    if path.exists():raise RuntimeError('Permission already issued; no repeat')
    temp=path.with_suffix('.tmp')
    temp.write_text(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf8')
    temp.replace(path)


def operator_prompt(index,state):
    prompts={
        0:'当前必须空手悬空、整手准备动作范围无遮挡、退出其他控制且现场守急停。h=仅建立PREPARE手型；q=退出：',
        1:('现在才允许用现有示教摆臂：电池仍直立落在原完整槽内，放进拇指与四指之间的夹持空隙；'
            '保持PREPARE，各指与电池均有间隙。调整腕姿/接近方向，使拇指指面与食指或中指指面'
            '能从电池相对两侧夹持，避免无名指单独顶住而拇指够不到；必要时选更近的完整槽。'
            '这里不要求掌面水平，也不复用旧坐标。摆好后退出示教、停稳并撤开双手。'
            '量本次整手最低外表面到亚克力板顶的垂直间隙mm（至少10；不是到电池的间距；q退出）：'),
        2:'确认从本次预抓位向上100mm的整臂、手、腕相机与线缆扫掠无遮挡，人员撤离且现场守急停。u=空手升100mm；q=保持退出：',
        4:'同一电池与板未移动、下降通道仍空闲，人员撤离且现场守急停。d=下降回本次预抓位；q=保持退出：',
        5:('确认空壳全部重量仍由原完整凹槽承托，仅允许手指接触该空壳，人员双手退出。'
            '当前仍非接触，空壳/板/障碍未改变，手臂及手型未被其他控制改变。'
            'g=学校首组夹持，前四指中哪根先出现合法接触状态5就仅降低该指目标一次；其他指不加紧；q=保持退出：'),
        8:('空壳仍由原槽承托。仅断电测试空壳：确认整手和空壳向上20mm及原路落回均无遮挡，'
            '空壳可从槽中竖直提离且落回区能承接，现场全程观察滑落并守急停。'
            '须亲眼确认拇指与食指或中指已从相对两侧贴合，且没有喀喀声、持续转动或升温。'
            'l=同时确认上述条件并试提20mm后原路放回；s=接触不足但当前无异常，转承托张开；'
            '有异响/升温立即硬件停止，不等输入；q=保持退出：'),
        6:('先确认空壳现已由原槽/板完整承托，张开范围无遮挡且双手退出。'
            'r=亲眼确认本轮实际提离后已落回；n=未提离/未夹住但目前已承托；'
            '两者均仅授权张开，q=不张开退出：'),
        7:'确认电池仍由板承托、手已完全脱离且空手，向上100mm扫掠无遮挡。u=空手返回本轮上方；q=保持退出：'}
    if index==6 and state.get('loaded_lift_blocked') is True:
        return ('本轮出现状态3：禁止提起，原夹持权限已锁存，尚未授权张开。'
            '确认空壳全部重量由原槽/固定支撑承托，张开范围无遮挡且双手退出。'
            'n=仅申请独立承托张开（新鲜回读符合后才发指令）；q=保持退出。'
            '持续转动、异常声响或升温立即现场硬件停止，不等程序：')
    if index==6 and state.get('loaded_lift_performed') is not True:
        print('本轮尚未记录带载提离；如实际未提离请选择n，不能用r代替测试。',flush=True)
    return prompts[index]


def _live_input(prompt,process,state_path,phase,index,log):
    """Poll stdin and child together; a dead child cannot be held by input()."""
    print(prompt,end='',flush=True)
    began=time.monotonic()
    next_notice=began+60
    while True:
        if process.poll() is not None:raise RuntimeError('Worker exited while awaiting input; no stale permission')
        state=json.loads(state_path.read_text(encoding='utf8'))
        if state.get('phase')!=phase or state.get('permission_index')!=index:
            raise RuntimeError('Worker changed/aborted during input; stale permission refused')
        if time.monotonic()-began>570:
            raise TimeoutError('Operator input expired; no automatic hand opening or return')
        if phase=='await_grasp' and time.monotonic()>=next_notice:
            print('\n等待输入 g 并回车才夹持；q并回车退出。没有自动夹持。',flush=True)
            log(dict(event='operator_wait_reminder',phase=phase,index=index))
            next_notice=time.monotonic()+60
        ready,_,_=select.select([sys.stdin],[],[],.2)
        if not ready:continue
        answer=sys.stdin.readline()
        if answer=='':raise EOFError('Operator input closed; no automatic hand opening or return')
        answer=answer.rstrip('\r\n')
        log(dict(event='operator_answer',phase=phase,index=index,answer=answer,
            timestamp_utc=datetime.now(timezone.utc).isoformat()))
        return answer


def _phase_deadline(phase,state):
    if str(phase).startswith('await_'):return 600
    if phase in ('completed','fault'):return 15
    if str(phase).startswith('preflight') or phase=='final_cycle_preflight':return 120
    if phase in ('preparing_empty_hand','grasping','releasing','freezing_supported_contact'):return 120
    if isinstance(state.get('plan'),dict) and any(word in str(phase) for word in
            ('lifting','lowering','descending','returning')):
        return state['plan'].get('arm_timeout_seconds',60)+20
    return 120


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--confirm-empty-clear-exclusive',action='store_true')
    parser.add_argument('--confirm-onsite-stop',action='store_true')
    parser.add_argument('--confirm-inert-shell',action='store_true')
    parser.add_argument('--resume-before-grasp',type=Path,
        help='Reviewed015735 result ZIP; fresh strict match then wait for g, no startup arm move')
    parser.add_argument('--sdk-dir',type=Path,
        default=Path('/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package'))
    parser.add_argument('--output-parent',type=Path,default=ROOT/'captures')
    args=parser.parse_args(argv)
    if not all((args.execute,args.confirm_empty_clear_exclusive,args.confirm_onsite_stop,args.confirm_inert_shell)):
        parser.error('Requires explicit execution, empty/exclusive control, onsite stop and inert test shell; no connection')
    if sys.version_info<(3,8) or platform.system()!='Linux' or platform.machine()!='aarch64':
        parser.error('Python3.8+ on reviewed onsite Linux aarch64 only; no connection')
    try:
        hashes=installed()
        if args.resume_before_grasp:
            from supported_grasp_resume import load_resume_source
            load_resume_source(args.resume_before_grasp)
    except (OSError,ValueError) as exc:parser.error(str(exc))
    folder=args.output_parent/('supported_low_lift_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True,exist_ok=False);(folder/'permissions').mkdir()
    report=dict(schema_version=1,scope='fresh_taught_same_slot_supported_low_lift_return_release',
        timestamp_utc=datetime.now(timezone.utc).isoformat(),installed_file_sha256=hashes,
        execute_requested=True,sdk_in_parent=False,exceptions=[],operator_inputs=[],
        worker_cleanup_timeout=False,vision_used=False,camera_extrinsics_used_for_motion=False,
        previous_result_reused=bool(args.resume_before_grasp),historical_clearances_reused=False,offline_tests_executed=False,
        load_lift_or_transfer_performed=False,loaded_lift_performed=False,lateral_transfer_ready=False,
        full_pregrasp_execution_ready=False,autonomous_other_targets_ready=False,
        physical_stop_verified=False,torque_release_verified=False)
    report['resume_before_grasp_requested']=bool(args.resume_before_grasp)

    def save():
        temp=folder/'parent_result.tmp'
        temp.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf8')
        temp.replace(folder/'parent_result.json')

    def log(row):
        with (folder/'operator_input_events.jsonl').open('a',encoding='utf8') as stream:
            stream.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')

    def abort():
        path=folder/'abort.json'
        if not path.exists():path.write_text('{"abort":true}\n',encoding='ascii')

    process=None;native_hung=False;last_phase=None;seen=set()
    phase_began=time.monotonic();save()
    try:
        (ROOT/'docs').mkdir(exist_ok=True)
        with (ROOT/'docs/执行命令日志.txt').open('a',encoding='utf8') as history:
            history.write(shlex.join([sys.executable,*sys.argv])+'\n')
        if args.resume_before_grasp:
            print('续接015735：仅用已完成下降端点核对当前位置；严格匹配且手型为PREPARE后等待g。启动不升降、不摆臂、不发手目标。',flush=True)
            print('不符即退出，不自动回旧位。仅同一空壳/槽位/板及障碍未改变时才能确认g；不得因续接而搬动机器人。',flush=True)
        else:
            print('本次：准备手型 → 示教当前非接触预抓位 → 空手升100mm/下降 → 夹持 → 试提20mm/原槽放回 → 承托张开 → 空返。',flush=True)
        print('不采图、不横移；手眼重标定等支架到位。仅断电测试空壳，原完整槽承托，现场全程守急停。',flush=True)
        print('只有明确示教提示期间才允许摆臂；其余时间独占控制。意外接触/滑落/继续运动立即硬件停止；q、ACK和进程退出不是急停或释放。',flush=True)
        print('若夹住后仍持续转动、异常声响或升温，立即现场硬件停止，不等下一提示。降低位置目标不等于卸扭矩；无温度传感器监测。',flush=True)
        command=[sys.executable,str(ROOT/'deployment/supported_loaded_worker.py'),
            '--folder',str(folder),'--execute','--sdk-dir',str(args.sdk_dir)]
        if args.resume_before_grasp:
            command+=['--resume-before-grasp',str(args.resume_before_grasp.resolve())]
        report['worker_process']=dict(command=command,stdin='DEVNULL',close_fds=True,SDK_in_parent=False)
        save()
        with (folder/'process_stdout.log').open('wb') as out,(folder/'process_stderr.log').open('wb') as err:
            process=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=out,stderr=err,close_fds=True)
            while process.poll() is None:
                path=folder/'worker_result.json'
                if not path.exists():
                    if time.monotonic()-phase_began>30:raise TimeoutError('No worker startup report')
                    time.sleep(.2);continue
                state=json.loads(path.read_text(encoding='utf8'))
                phase=state.get('phase');index=state.get('permission_index')
                if phase!=last_phase:
                    last_phase=phase;phase_began=time.monotonic()
                    if not str(phase).startswith('await_'):print('PHASE:',phase,flush=True)
                if time.monotonic()-phase_began>_phase_deadline(phase,state):
                    raise TimeoutError('Worker phase deadline: '+str(phase)+'; no automatic recovery')
                if str(phase).startswith('await_') and (index,phase) not in seen:
                    if index not in PHASES or PHASES[index][0]!=phase:
                        raise RuntimeError('Worker phase/index differs from the reviewed low-lift entry')
                    seen.add((index,phase));prompt=operator_prompt(index,state)
                    log(dict(event='operator_prompt',index=index,phase=phase,prompt=prompt,
                        timestamp_utc=datetime.now(timezone.utc).isoformat()))
                    while True:
                        answer=_live_input(prompt,process,path,phase,index,log)
                        try:
                            row=permission(index,phase,answer)
                            if index==6 and state.get('loaded_lift_blocked') is True and row['value']=='r':
                                raise ValueError('状态3已禁止本轮提起；只接受n申请承托张开，或q退出')
                            break
                        except ValueError as exc:print(exc,flush=True)
                    current=json.loads(path.read_text(encoding='utf8'))
                    if process.poll() is not None or current.get('phase')!=phase or current.get('permission_index')!=index:
                        raise RuntimeError('Worker changed/exited after input; no stale permission')
                    report['operator_inputs'].append(row);save();write_permission(folder,row)
                time.sleep(.2)
            report['worker_return_code']=process.returncode
    except (Exception,KeyboardInterrupt) as exc:
        report['exceptions'].append(dict(error=repr(exc)))
        abort()
        print('异常：现场硬件停止并安全解除受力；不自动张开、回臂或重试。',flush=True)
        if process is not None and process.poll() is None:
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.send_signal(signal.SIGINT)
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    native_hung=True;report['worker_cleanup_timeout']=True
                    print('SDK worker未确认退出；须现场硬件停止，结束进程不证明停机。',flush=True)
    finally:
        if process is not None:report['worker_return_code']=process.poll()
        path=folder/'worker_result.json'
        if path.exists():report['worker_result']=json.loads(path.read_text(encoding='utf8'))
        worker=report.get('worker_result',{})
        outcome=worker.get('outcome','worker_result_missing')
        if native_hung:outcome='SDK_worker_unresponsive_hardware_stop_required'
        elif report['exceptions'] and outcome in (SUCCESS,SUPPORTED_ONLY,OPEN_ONLY):
            outcome='parent_failed_after_worker_report'
        report.update(outcome=outcome,loaded_lift_performed=worker.get('loaded_lift_performed',False),
            load_lift_or_transfer_performed=worker.get('load_lift_or_transfer_performed',False))
        for name in ('supported_release_completed','empty_return_completed','loaded_lift_observed_by_operator'):
            if name in worker:report[name]=worker[name]
        save()
        bundle=folder.with_suffix('.zip')
        with zipfile.ZipFile(bundle,'x',zipfile.ZIP_DEFLATED) as archive:
            for path in sorted(folder.rglob('*')):
                if path.is_file():archive.write(path,str(Path(folder.name)/path.relative_to(folder)))
        print('SAVED:',bundle,'OUTCOME:',outcome,flush=True)
        errors=worker.get('exceptions',[])
        if errors:print('ERROR:',errors[-1].get('error'),flush=True)
        if outcome==SUCCESS:
            print('程序记录低提离/落回/承托张开/空返完成；现场是否真正提离以本轮r/n观察记录为准。未验收横移或视觉自主抓取。',flush=True)
        elif outcome==SUPPORTED_ONLY:
            print('仅承托张开/空返完成，本轮带载提离未通过；不得当作抓取提离完成。',flush=True)
        elif outcome==OPEN_ONLY:
            print('承托张开已完成，残留保护阻止空返；提离是否完成看实际字段。本轮现场提离观察：',
                worker.get('loaded_lift_observed_by_operator',False),'；未自动重试或归位。',flush=True)
        else:print('流程未完成；按ZIP中的实际故障处理，不自动重试。',flush=True)
    clean=not report['exceptions'] and not report.get('worker_result',{}).get('exceptions') and not report.get('worker_result',{}).get('cleanup_errors')
    if clean and report.get('worker_return_code')==0 and report['outcome']==SUCCESS:return 0
    if clean and report.get('worker_return_code')==2 and report['outcome'] in (SUPPORTED_ONLY,OPEN_ONLY):return 2
    return 1


if __name__=='__main__':raise SystemExit(main())
