"""Stdlib parent: accepted visual pregrasp, supervised grip/lift/bin/empty return."""
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


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-pregrasp',type=Path,default=ROOT/'captures/visual_pregrasp_stage_v8_20261021_174304_223863.zip')
    parser.add_argument('--capture-current-target',action='store_true',default=True)
    parser.add_argument('--vision-archive',type=Path)
    parser.add_argument('--confirm-saved-target-unchanged',action='store_true')
    parser.add_argument('--accepted-recipe',type=Path,default=ROOT/'calibration/visual_grip_accepted.json')
    parser.add_argument('--new-tune',action='store_true',help='Ignore saved successful recipe for this run')
    parser.add_argument('--align-empty-opposition-first',action='store_true')
    parser.add_argument('--recover-supported-v4-first',action='store_true')
    parser.add_argument('--empty-return-from-v6',type=Path)
    parser.add_argument('--color',choices=('black',),required=True)
    parser.add_argument('--execute',action='store_true',required=True)
    for name in ('inert-shell-exclusive','onsite-stop','fixed-setup'):
        parser.add_argument('--confirm-'+name,action='store_true',required=True)
    parser.add_argument('--sdk-dir',type=Path,default=Path('/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package'))
    args=parser.parse_args()
    if args.vision_archive and not args.confirm_saved_target_unchanged:
        parser.error('Saved vision target requires unchanged shell/head/board/mount confirmation')
    if args.align_empty_opposition_first or args.recover_supported_v4_first or args.empty_return_from_v6:
        parser.error('Live tune starts with fresh empty-hand vision; historical fault recovery is not replayed')
    if platform.system()!='Linux' or platform.machine()!='aarch64':parser.error('Actual Linux aarch64 robot only')
    if args.empty_return_from_v6 and (args.capture_current_target or args.recover_supported_v4_first):parser.error('Empty return is a separate first invocation')
    if args.capture_current_target and args.recover_supported_v4_first:parser.error('Supported recovery cannot reposition or run visual approach first')
    if args.recover_supported_v4_first:args.align_empty_opposition_first=True
    from known_cylinder_records import dump
    from visual_pick_sort_v11_source import verify_delivery,accepted_pregrasp,MANIFEST,RECIPE,COMMANDS,CAPS,accepted_recipe
    from run_school_bin_cycle_v6 import _live_input,permission,operator_prompt,write_permission,SUCCESS
    folder=ROOT/'captures'/datetime.now().strftime('visual_pick_sort_v11_%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True);(folder/'permissions').mkdir();state_path=folder/'worker_result.json'
    report=dict(scope='single_measured_black_shell_continuous_supervised_sort_v11',
        outcome='preflight',execution_ready=False,exceptions=[],SDK_in_parent=False,
        offline_tests_executed=False,source_mode='saved_vision_fresh_current_pregrasp' if args.vision_archive else 'new_capture',
        selected_color=args.color,new_cycle_real_device_accepted=False)
    process=None
    def save():dump(folder/'result.json',report)
    def log(row):
        with (folder/'operator_events.jsonl').open('a',encoding='utf8') as stream:
            stream.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
    try:
        report['delivery_sha256']=verify_delivery();save()
        print('本次视觉目标接当前状态预抓+现场小步调指，不需要向开发端回传ZIP才能继续。程序仍自动保存原始结果。',flush=True)
        answer=input('启动前确认：当前手确已空、所选空壳位置及相机/板/安装与采图时未变、空手升高手型范围清空、其他控制退出，现场守急停。e=确认；q=退出：').strip().lower()
        log(dict(event='empty_current_start_confirmation',value=answer))
        if answer!='e':raise KeyboardInterrupt('Current empty-hand start not confirmed')
        recipe_path=args.accepted_recipe if args.accepted_recipe.exists() and not args.new_tune else None
        saved=accepted_recipe(recipe_path) if recipe_path else None
        report['saved_successful_recipe']=saved;save()
        if args.capture_current_target:
            print('复用指定目标或新采S/Q；本轮新读臂手，m先保持手型竖直升高再准备对准，d下降，p确认非接触后才进入夹持。',flush=True)
            before=set((ROOT/'captures').glob('visual_pregrasp_stage_v11_cycle_*.zip'))
            command=[sys.executable,str(ROOT/'deployment/run_visual_pregrasp_stage_v11_cycle.py'),
                '--capture-current-target','--completed-run',str(args.source_pregrasp.resolve()),
                '--execute','--confirm-empty-clear-exclusive','--confirm-onsite-stop','--confirm-inert-shell',
                '--confirm-fixed-head-board-mount','--sdk-dir',str(args.sdk_dir.resolve())]
            if args.vision_archive:
                command.remove('--capture-current-target')
                command+=['--vision-archive',str(args.vision_archive.resolve()),'--confirm-saved-target-unchanged']
                report['vision_archive_reused_without_recapture']=str(args.vision_archive.resolve())
            report['visual_pregrasp_command']=command;save()
            captured=subprocess.run(command,cwd=str(ROOT),check=False)
            created=set((ROOT/'captures').glob('visual_pregrasp_stage_v11_cycle_*.zip'))-before
            if len(created)==1:
                candidate=created.pop();(folder/'input_pregrasp.zip').write_bytes(candidate.read_bytes())
                args.source_pregrasp=candidate
            if captured.returncode or len(created)>0 or not (folder/'input_pregrasp.zip').exists():
                raise RuntimeError('Fresh visual pregrasp not accepted; no grip process')
        else:(folder/'input_pregrasp.zip').write_bytes(args.source_pregrasp.read_bytes())
        source=accepted_pregrasp(folder/'input_pregrasp.zip');report['accepted_pregrasp']=source;save()
        if args.empty_return_from_v6:
            print('先核对本次投放位和空手闭合姿态；u后只张开五指、保持对掌，再空返学校高位。',flush=True)
        else:
            print('沿用已确认的视觉预抓腕位，先核对当前臂手；不自动摆位。',flush=True)
            if args.recover_supported_v4_first:
                print('当前持物恢复：先确认原槽承托全部重量，o只退前四指；取走空壳后r确认空手，再重新放回原槽后g。',flush=True)
            else:
                print('预抓p确认后先c闭指，再现场小步调；调节时空壳由板承托。',flush=True)
            print('初始配方：',saved['recipe'] if saved else RECIPE,'；c先闭指，随后现场每次250小步调；t拇指/i食指/m中指/r无名指，+增加弯曲，-减少。',flush=True)
            print('调节上限：',CAPS,'；小拇指/对掌不调。达到上限或已有status5时拒绝再加弯，不改硬件保护。',flush=True)
            print('调好后一次g授权：不重闭指→反馈合格→竖直提140mm→三个同型MoveJ-P以连接1/1/0连续下发→黑框张开；持物中不等键盘。',flush=True)
            print('黑框落点朝固定头部相机反光轴的水平机身方向后移40mm，Z/腕姿不变；提起速度5%，转运10%。',flush=True)
            print('g前确认含空壳的上升及后移黑框整段转运范围有至少20mm余量。该确认是预计路线许可，不是假造升后实测。',flush=True)
            print('张开后才u确认实际落框并空返，或e确认落框后结束。真实故障仍停止，不能保证必到终点。',flush=True)
            print('无自动故障松手/回臂/重试；状态3/4、超流、拒读阻断。现场全程守急停。',flush=True)
        command=[sys.executable,str(ROOT/'deployment/visual_pick_sort_v11_worker.py'),
            '--folder',str(folder),'--source-pregrasp',str((folder/'input_pregrasp.zip').resolve()),
            '--source-sha256',source['archive_sha256'],'--manifest-sha256',hashlib.sha256(MANIFEST.read_bytes()).hexdigest(),
            '--color',args.color,'--sdk-dir',str(args.sdk_dir.resolve())]
        if args.align_empty_opposition_first:command.append('--align-empty-opposition-first')
        if args.recover_supported_v4_first:command.append('--recover-supported-v4-first')
        if args.empty_return_from_v6:command+=['--empty-return-from-v6',str(args.empty_return_from_v6.resolve())]
        if recipe_path:command+=['--accepted-recipe',str(recipe_path.resolve())]
        report['worker_command']=command;save()
        with (folder/'worker_stdout.txt').open('wb') as output:
            process=subprocess.Popen(command,cwd=str(ROOT),stdin=subprocess.DEVNULL,stdout=output,stderr=subprocess.STDOUT)
            handled=set();last=None;began=time.monotonic()
            phases={5:'await_seed_contact',12:'await_empty_bin_return'}
            while process.poll() is None:
                if state_path.exists():
                    state=json.loads(state_path.read_text(encoding='utf8'));phase=state.get('phase')
                    if phase!=last:last=phase;began=time.monotonic();print('PHASE:',phase,flush=True)
                    if str(phase).startswith('await_'):
                        index=state['permission_index']
                        if (index,phase) not in handled:
                            if phases.get(index)!=phase and not (15<=index<115 and phase=='await_live_grip_tune'):
                                raise RuntimeError('Unknown permission phase')
                            prompt=(('先取走松开的空壳，确认手内为空、臂未移位，拇指旋转退让范围清空，双手撤开。'
                                'r=确认手已空；如对掌已合格只读通过，否则仅目标64556退让一次；臂不动；q=退出：') if index==4 else '' if index in (3,5,13) or 15<=index<115 else operator_prompt(index,state))
                            if index==12:
                                prompt=('亲眼确认结果：u=空壳确已落黑框且手空并空返；e=确已落框且手空结束；'
                                    'f=本轮没夹起/没投进且手已空，仅记录失败结束；q=不确定或仍持物，保持退出：')
                            if index==13:
                                prompt=('确认手已完全空、目前停在本次投放位、无人触碰且退出其他控制。'
                                    '五指张开范围和整臂/手/相机/线缆到学校高位的空返通道清空。'
                                    'u=一次授权空手五指张开及返回高位；q=不动退出：')
                            if index==3:
                                prompt=('仅在空壳完全落底于原凹槽/可靠固定承托全部重量，或手已空，且臂未移位时继续。'
                                    '四指退让范围清空，撤开双手；不可用手托壳等待动作。'
                                    'o=只将拇指弯曲/食指/中指/无名指目标退至8000一次，小拇指/对掌不写，臂不动；q=退出：')
                            if index==5:
                                prompt=('保持刚完成视觉预抓的空壳在原处、完全由槽/板承托，撤手，闭指范围清空。'
                                    'c=初始闭指一次，臂不动，随后现场调指；q=退出：')
                            if 15<=index<115:
                                print('当前手反馈：',state.get('live_tune_feedback'),flush=True)
                                if state.get('last_tune_step'):print('上次调节：',{k:v for k,v in state['last_tune_step'].items() if k!='groups'},flush=True)
                                prompt=('空壳继续由板承托；输入t+/t-/i+/i-/m+/m-/r+/r-每次250，只动该指。'
                                    'g=确认当前手型后连续提140mm/转运/黑框张开（含物全通道预计余量至少20mm）；'
                                    'q=退出。运行中若漏抓，f回车可请求停止后续投放；非受控运动用硬件急停：')
                            while True:
                                answer=_live_input(prompt,process,state_path,phase,index,log)
                                try:
                                    if 15<=index<115:
                                        key=answer.strip().lower()
                                        if key not in tuple(COMMANDS)+('g',):
                                            if key=='q':raise KeyboardInterrupt('Live tune quit')
                                            raise ValueError('请输入一个明确的调指命令或g/q')
                                        event=dict(index=index,phase=phase,value=key,
                                            shell_fully_bottom_supported_original_slot=True,
                                            only_selected_flex_contact_sweep_clear=True,
                                            timestamp_utc=datetime.now(timezone.utc).isoformat())
                                        if key=='g':event.update(live_tuned_grip_and_continuous_cycle_authorized=True,
                                            continuous_lift_transfer_normal_drop_authorized=True,
                                            planned_loaded_route_min20mm_clear_confirmed=True,
                                            permission_is_prospective_not_holding_sensor_proof=True)
                                    elif index==5:
                                        if answer.strip().lower()!='c':raise KeyboardInterrupt('Initial contact cancelled')
                                        event=dict(index=5,phase=phase,value='c',
                                            only_intended_shell_contact_and_closure_clear=True,
                                            current_noncontact_pose_and_same_shell_scene_confirmed=True,
                                            shell_fully_bottom_supported_original_slot=True,
                                            timestamp_utc=datetime.now(timezone.utc).isoformat())
                                    elif index==12 and answer.strip().lower()=='f':
                                        event=dict(index=12,phase=phase,value='f',hand_empty_but_pickup_failed=True,
                                            shell_in_selected_bin_and_hand_empty=False,timestamp_utc=datetime.now(timezone.utc).isoformat())
                                    elif index==13:
                                        if answer.strip().lower()!='u':raise KeyboardInterrupt('Empty return cancelled')
                                        event=dict(index=13,phase=phase,value='u',hand_empty_and_entire_return_sweep_clear=True,
                                            timestamp_utc=datetime.now(timezone.utc).isoformat())
                                    elif index==3:
                                        if answer.strip().lower()!='o':raise KeyboardInterrupt('Supported opening cancelled')
                                        event=dict(index=3,phase=phase,value='o',shell_fully_supported_or_hand_empty=True,
                                            four_flex_opening_sweep_clear=True,arm_stationary=True,
                                            timestamp_utc=datetime.now(timezone.utc).isoformat())
                                    elif index==4:
                                        if answer.strip().lower()!='r':raise KeyboardInterrupt('Empty opposition retreat cancelled')
                                        event=dict(index=4,phase=phase,value='r',empty_hand_and_opposition_sweep_clear=True,
                                            explicit_empty_opposition_retreat_before_new_cycle=True,timestamp_utc=datetime.now(timezone.utc).isoformat())
                                    else:event=permission(index,phase,answer)
                                    if index==5 and event['value']=='g':
                                        event.update(continuous_lift_transfer_normal_drop_authorized=True,
                                            planned_loaded_route_min20mm_clear_confirmed=True,
                                            permission_is_prospective_not_postlift_observation=True)
                                    break
                                except ValueError as exc:print(str(exc),flush=True)
                            current=json.loads(state_path.read_text(encoding='utf8'))
                            if process.poll() is not None or current.get('phase')!=phase or current.get('permission_index')!=index:
                                raise RuntimeError('Stale permission refused')
                            write_permission(folder,event);handled.add((index,phase));began=time.monotonic()
                    if not str(phase).startswith('await_') and select.select([sys.stdin],[],[],0)[0]:
                        key=sys.stdin.readline().strip().lower()
                        log(dict(event='running_operator_key',value=key,phase=phase))
                        if key in ('f','q'):
                            report['operator_reported_pickup_failure_during_motion']=key=='f'
                            raise RuntimeError('现场报告漏抓或请求停止；取消后续转运/投放，不自动松手')
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
        report['previous_pregrasp_replayed']=False
        report['fresh_visual_pregrasp_executed']=args.capture_current_target
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
        if report.get('new_cycle_real_device_accepted') and not report.get('exceptions'):
            try:
                target=ROOT/'calibration/visual_grip_accepted.json';target.parent.mkdir(parents=True,exist_ok=True)
                dump(target,dict(archive=str(archive.resolve()),archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
                    recipe=report['worker']['new_grip_receipt']['recipe'],actual_sort_confirmed_by_operator=True))
                print('实际成功手型已固化：',target,flush=True)
            except (OSError,KeyError) as exc:print('本机手型保存失败，原始结果仍在：',repr(exc),flush=True)
    return 0 if (report['outcome']==SUCCESS and report.get('worker_returncode')==0
        and not report['exceptions'] and not report.get('worker',{}).get('cleanup_errors')) or (report['outcome']=='completed_empty_return_from_qualified_v6_bin' and report.get('worker_returncode')==0 and not report['exceptions'] and not report.get('worker',{}).get('cleanup_errors')) else 1


if __name__=='__main__':raise SystemExit(main())
