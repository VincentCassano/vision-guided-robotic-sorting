#!/usr/bin/env python3
"""Stdlib-only parent for one supervised school-coordinate bin cycle."""
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
MANIFEST='deployment/school-bin-cycle-v4-files.sha256'
SUCCESS='completed_school_bin_drop_feedback_and_operator_empty_confirmation'
SUPPORTED_ONLY='completed_supported_release_lift_not_performed'
OPEN_ONLY='completed_supported_opening_empty_return_blocked'
PHASES={0:('await_prepare',('h',)),1:('await_goal',('t',)),
    2:('await_empty_lift',('u',)),4:('await_descend',('d',)),
    5:('await_grasp',('g',)),8:('await_bin_lift',('l',)),
    11:('await_bin_transfer',('t',)),12:('await_empty_bin_return',('u','e')),
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
            raise ValueError('Installed school-bin payload differs: '+name+'; no connection')
        hashes[name]=digest
    if not hashes:raise ValueError('Empty school-bin manifest; no connection')
    return hashes


def permission(index,phase,answer):
    expected,values=PHASES[index]
    if phase!=expected:raise ValueError('Wrong school-bin phase')
    value=answer.strip().lower()
    row={}
    if index==1 and value!='q':
        try:gap=float(value)
        except ValueError:raise ValueError('输入当前手部最低点到板顶的垂直间隙mm，或q')
        if not math.isfinite(gap) or gap<10:
            raise ValueError('到板顶的垂直间隙须至少10mm；各指不得接触电池；不确定就q')
        value='t';row['lowest_hand_to_board_mm']=gap
    if index==11 and value!='q':
        try:clearance=float(value)
        except ValueError:raise ValueError('输入提起后手和空壳最低点高于障碍顶面的实测净空mm，或q')
        if not math.isfinite(clearance) or clearance<20:raise ValueError('带物实际净空须至少20mm；不足不横移、不自动放回')
        value='t';row['loaded_lowest_point_above_obstacle_plane_mm']=clearance
        row.update(selected_bin_normal_drop_authorized=True,whole_adapted_school_route_clear=True,
            shell_actually_lifted_and_retained=True)
    if index==12:
        row['shell_in_selected_bin_and_hand_empty']=value in ('u','e')
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
        row['opposed_grip_and_loaded_up140_clear']=yes
        row['no_uncontrolled_motion_slip_or_overheating_observed']=yes
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
            'g=一次配方夹持，前四指中哪根先出现合法接触状态5就仅降低该指目标一次；其他指不加紧；q=保持退出：'),
        8:('确认拇指与食指/中指已对向夹稳空壳，原槽可竖直退出；手、空壳、相机、线缆和整臂上升140mm扫掠无遮挡。'
            '无滑落/非受控运动/明显升温，现场守急停；接触声不单独中止。l=带物竖直提起140mm；q=保持退出：'),
        11:('确认空壳已实际离槽且未滑落。接下来将连续经过学校HIGHT_POS、SCAN_BATTERY_RESTART_POS到所选盒位，'
            '并自动张开五指投放；不执行扫描姿态/HTTP检测和原等待。学校两盒须仍在原处；确认整臂、手、相机、线缆及空壳的整条适配通道无遮挡，'
            '盒内能承接空壳、张开范围无遮挡、人员撤离。不能只看TCP高度。'
            '量当前手和持物中最低点高于横移区最高障碍顶面水平线的垂直净空mm（至少20）；'
            '输入净空即授权本轮所选盒投放，q保持退出：'),
        12:('空壳已实际落入所选盒且手完全空了吗？u=确认并空手回学校高位SCAN_BATTERY_RESTART_POS；'
            'e=确认已落盒且空手，在此结束、不回臂；仍被夹住或不确定选q，禁止用u当松手命令：'),
        6:('先确认空壳现已由原槽/板完整承托，张开范围无遮挡且双手退出。'
            'r=亲眼确认本轮实际提离后已落回；n=未提离/未夹住但目前已承托；'
            '两者均仅授权张开，q=不张开退出：'),
        7:'确认电池仍由板承托、手已完全脱离且空手，向上100mm扫掠无遮挡。u=空手返回本轮上方；q=保持退出：'}
    if index==6 and state.get('loaded_lift_blocked') is True:
        return ('本轮出现状态3：禁止提起，原夹持权限已锁存，尚未授权张开。'
            '确认空壳全部重量由原槽/固定支撑承托，张开范围无遮挡且双手退出。'
            'n=仅申请独立承托张开（新鲜回读符合后才发指令）；q=保持退出。'
            '非受控运动、滑落或明显升温立即现场硬件停止，不等程序：')
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
        if time.monotonic()-began>(60 if index in (8,11,12) else 570):
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
    parser.add_argument('--color',required=True,choices=('black','red','white'))
    parser.add_argument('--confirm-empty-clear-exclusive',action='store_true')
    parser.add_argument('--confirm-onsite-stop',action='store_true')
    parser.add_argument('--confirm-inert-shell',action='store_true')
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
    except (OSError,ValueError) as exc:parser.error(str(exc))
    folder=args.output_parent/('school_bin_cycle_v4_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True,exist_ok=False);(folder/'permissions').mkdir()
    report=dict(schema_version=1,scope='fresh_taught_same_slot_supported_low_lift_return_release',
        timestamp_utc=datetime.now(timezone.utc).isoformat(),installed_file_sha256=hashes,
        execute_requested=True,sdk_in_parent=False,exceptions=[],operator_inputs=[],
        worker_cleanup_timeout=False,vision_used=False,camera_extrinsics_used_for_motion=False,
        previous_result_reused=False,historical_clearances_reused=False,offline_tests_executed=False,
        load_lift_or_transfer_performed=False,loaded_lift_performed=False,lateral_transfer_ready=False,
        full_pregrasp_execution_ready=False,autonomous_other_targets_ready=False,
        physical_stop_verified=False,torque_release_verified=False)
    report['resume_before_grasp_requested']=False

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
        config=json.loads((ROOT/'deployment/school_battery_bins.json').read_text(encoding='utf8'))
        target=config['bins'][args.color]
        print('本轮空壳颜色:',args.color,'；学校投放点:',target['source'],target['pose'],'；单位m/rad。',flush=True)
        print('v4只从本轮空手状态开始：重启/移臂后旧持物恢复已失效，不加载旧抓取或故障坐标，不自动移到旧位。',flush=True)
        print('先空手悬空、让整手准备范围无遮挡，h明确准备手型。对掌目标采用本次实读65222，原150容差不变。随后才按提示示教新预抓位。',flush=True)
        print('本轮上方100mm与下降后g夹持；新首组[15000,30000,28000,20000,65535,65222]，中指和无名指少弯曲。',flush=True)
        print('夹稳后l竖直提起140mm；实测带物净空后输入数值，授权连续学校过渡位→所选盒→张开。最后u空返或e原地结束。',flush=True)
        print('复用实际电池RECYCLE坐标，不使用球/瓶POSE_BASKET。省略扫描姿态与检测等待的适配路线尚未真机验收。',flush=True)
        print('仅断电测试空壳，学校盒位和安装须未改变；全程独占并现场守急停。非预期碰撞/滑落/非受控运动/明显升温立即硬件停止。',flush=True)
        print('接触声不单独中止；真实过流、状态3/4、通信失败仍阻断。故障无自动松手/回臂/重试；ACK或退出不证明停止。',flush=True)
        command=[sys.executable,str(ROOT/'deployment/school_bin_cycle_v4_worker.py'),
            '--folder',str(folder),'--execute','--color',args.color,'--sdk-dir',str(args.sdk_dir)]
        report['selected_color']=args.color
        report['scope']='fresh_empty_startup_prepare_then_smaller_grasp_school_bin_drop'
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
                        raise RuntimeError('Worker phase/index differs from the reviewed school-bin entry')
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
        for name in ('supported_release_completed','empty_return_completed','loaded_lift_observed_by_operator',
                'loaded_transfer_performed','bin_release_feedback_completed','shell_in_selected_bin_observed_by_operator'):
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
            print('本轮记录提起、学校盒位到达、五指张开和现场落盒/空手确认；是否空返见empty_return_completed。未验收视觉自主或其他目标。',flush=True)
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
