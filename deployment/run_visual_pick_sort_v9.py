"""Stdlib parent: accepted visual pregrasp, supervised grip/lift/bin/empty return."""
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
import traceback
import zipfile

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-pregrasp',type=Path)
    parser.add_argument('--supported-v7-ring-retreat',type=Path)
    parser.add_argument('--observed-healthy-v8',type=Path,required=True)
    parser.add_argument('--capture-current-target',action='store_true')
    parser.add_argument('--align-empty-opposition-first',action='store_true')
    parser.add_argument('--recover-supported-v4-first',action='store_true')
    parser.add_argument('--empty-return-from-v6',type=Path)
    parser.add_argument('--color',choices=('black',),required=True)
    parser.add_argument('--execute',action='store_true',required=True)
    for name in ('inert-shell-exclusive','onsite-stop','fixed-setup'):
        parser.add_argument('--confirm-'+name,action='store_true',required=True)
    parser.add_argument('--sdk-dir',type=Path,default=Path('/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package'))
    args=parser.parse_args()
    if args.source_pregrasp is None and args.supported_v7_ring_retreat is None:parser.error('Source pregrasp or exact supported-v7 source required')
    if args.supported_v7_ring_retreat and (args.capture_current_target or args.align_empty_opposition_first or args.recover_supported_v4_first or args.empty_return_from_v6):
        parser.error('Supported ring withdrawal cannot mix with other positioning/recovery modes')
    if platform.system()!='Linux' or platform.machine()!='aarch64':parser.error('Actual Linux aarch64 robot only')
    if args.empty_return_from_v6 and (args.capture_current_target or args.recover_supported_v4_first):parser.error('Empty return is a separate first invocation')
    if args.capture_current_target and args.recover_supported_v4_first:parser.error('Supported recovery cannot reposition or run visual approach first')
    if args.recover_supported_v4_first:args.align_empty_opposition_first=True
    from known_cylinder_records import dump
    from visual_pick_sort_v9_source import verify_delivery,accepted_pregrasp,MANIFEST,RECIPE,qualified_supported_v7,FAULT_SHA,qualified_healthy_v8
    from run_school_bin_cycle_v6 import _live_input,permission,operator_prompt,write_permission,SUCCESS
    folder=ROOT/'captures'/datetime.now().strftime('visual_pick_sort_v9_%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True);(folder/'permissions').mkdir();state_path=folder/'worker_result.json'
    report=dict(scope='single_measured_black_shell_continuous_supervised_sort_v9',
        outcome='preflight',execution_ready=False,exceptions=[],SDK_in_parent=False,
        offline_tests_executed=False,source_mode='new_capture' if args.capture_current_target else 'accepted_pregrasp_continuation',
        selected_color=args.color,new_cycle_real_device_accepted=False)
    process=None
    def save():dump(folder/'result.json',report)
    def log(row):
        with (folder/'operator_events.jsonl').open('a',encoding='utf8') as stream:
            stream.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
    try:
        report['delivery_sha256']=verify_delivery();save()
        report['observed_healthy_v8_source']=qualified_healthy_v8(args.observed_healthy_v8)
        if args.supported_v7_ring_retreat:
            raw=args.supported_v7_ring_retreat.read_bytes()
            if hashlib.sha256(raw).hexdigest()!=FAULT_SHA:raise ValueError('Exact235346 source required')
            (folder/'input_supported_v7.zip').write_bytes(raw)
            with zipfile.ZipFile(folder/'input_supported_v7.zip') as z:
                prefix=z.namelist()[0].split('/')[0]+'/'
                (folder/'input_pregrasp.zip').write_bytes(z.read(prefix+'input_pregrasp.zip'))
            report['supported_v7_source']=qualified_supported_v7(folder/'input_supported_v7.zip',folder/'input_pregrasp.zip')
            report['source_mode']='supported_ring_withdrawal_at_completed_healthy_visual_endpoint'
        elif args.capture_current_target:
            print('采当前视觉目标：S/Q保存，m/d执行，终点必须p确认非接触才进入夹持；之后不要移动空壳。',flush=True)
            before=set((ROOT/'captures').glob('visual_pregrasp_stage_v7_cycle_*.zip'))
            command=[sys.executable,str(ROOT/'deployment/run_visual_pregrasp_stage_v7_cycle.py'),
                '--capture-current-target','--completed-run',str(args.source_pregrasp.resolve()),
                '--execute','--confirm-empty-clear-exclusive','--confirm-onsite-stop','--confirm-inert-shell',
                '--confirm-fixed-head-board-mount','--sdk-dir',str(args.sdk_dir.resolve())]
            report['visual_pregrasp_command']=command;save()
            captured=subprocess.run(command,cwd=str(ROOT),check=False)
            created=set((ROOT/'captures').glob('visual_pregrasp_stage_v7_cycle_*.zip'))-before
            if len(created)==1:
                candidate=created.pop();(folder/'input_pregrasp.zip').write_bytes(candidate.read_bytes())
                args.source_pregrasp=candidate
            if captured.returncode or len(created)>0 or not (folder/'input_pregrasp.zip').exists():
                raise RuntimeError('Fresh visual pregrasp not accepted; no grip process')
        else:(folder/'input_pregrasp.zip').write_bytes(args.source_pregrasp.read_bytes())
        source=accepted_pregrasp(folder/'input_pregrasp.zip');report['accepted_pregrasp']=source;save()
        if args.supported_v7_ring_retreat:
            print('本次不采图、不回旧位、不重闭其他手指。新读匹配本次已完成视觉预抓端点；g一次许可：原槽承托下只退无名指20500→20000，健康后提140mm→连接转运→黑框正常张开。',flush=True)
            print('v8已实读无名指保护消失。当前任何状态3/4都阻断，不清错/复位/改限流；新鲜健康手型通过后才可退让并接整轮。',flush=True)
        elif args.empty_return_from_v6:
            print('先核对本次投放位和空手闭合姿态；u后只张开五指、保持对掌，再空返学校高位。',flush=True)
        else:
            print('沿用已确认的视觉预抓腕位，先核对当前臂手；不自动摆位。',flush=True)
            if args.recover_supported_v4_first:
                print('当前持物恢复：先确认原槽承托全部重量，o只退前四指；取走空壳后r确认空手，再重新放回原槽后g。',flush=True)
            else:
                print('空手对掌确认合格后，g前重新确认新手型与空壳间隙。',flush=True)
            print('本轮新候选配方：',RECIPE,'；仅无名指候选20000，较v7的20500减少500；食指31500不再增加，对掌64556不变；闭指只写前四通道，不重复写小拇指/对掌；原六通道150/2门保留，合法status5只退不加紧。',flush=True)
            print('夹持前一次g授权：闭指→反馈合格→竖直提140mm→三个同型MoveJ-P以连接1/1/0连续下发→黑框张开；持物中不等键盘。',flush=True)
            print('黑框落点朝固定头部相机反光轴的水平机身方向后移40mm，Z/腕姿不变；提起速度5%，转运10%。',flush=True)
            print('g前确认含空壳的上升及后移黑框整段转运范围有至少20mm余量。该确认是预计路线许可，不是假造升后实测。',flush=True)
            print('张开后才u确认实际落框并空返，或e确认落框后结束。真实故障仍停止，不能保证必到终点。',flush=True)
            print('无自动故障松手/回臂/重试；状态3/4、超流、拒读阻断。现场全程守急停。',flush=True)
        command=[sys.executable,str(ROOT/'deployment/visual_pick_sort_v9_worker.py'),
            '--folder',str(folder),'--source-pregrasp',str((folder/'input_pregrasp.zip').resolve()),
            '--source-sha256',source['archive_sha256'],'--manifest-sha256',hashlib.sha256(MANIFEST.read_bytes()).hexdigest(),
            '--color',args.color,'--sdk-dir',str(args.sdk_dir.resolve())]
        if args.align_empty_opposition_first:command.append('--align-empty-opposition-first')
        if args.recover_supported_v4_first:command.append('--recover-supported-v4-first')
        if args.empty_return_from_v6:command+=['--empty-return-from-v6',str(args.empty_return_from_v6.resolve())]
        if args.supported_v7_ring_retreat:command+=['--supported-v7-ring-retreat',str((folder/'input_supported_v7.zip').resolve())]
        command+=['--observed-healthy-v8',str(args.observed_healthy_v8.resolve())]
        report['worker_command']=command;save()
        with (folder/'worker_stdout.txt').open('wb') as output:
            process=subprocess.Popen(command,cwd=str(ROOT),stdin=subprocess.DEVNULL,stdout=output,stderr=subprocess.STDOUT)
            handled=set();last=None;began=time.monotonic()
            phases={3:'await_supported_v4_open',4:'await_empty_opposition_withdrawal',5:'await_grasp',12:'await_empty_bin_return',13:'await_empty_return_after_v6',14:'await_supported_v7_ring_retreat'}
            while process.poll() is None:
                if state_path.exists():
                    state=json.loads(state_path.read_text(encoding='utf8'));phase=state.get('phase')
                    if phase!=last:last=phase;began=time.monotonic();print('PHASE:',phase,flush=True)
                    if str(phase).startswith('await_'):
                        index=state['permission_index']
                        if (index,phase) not in handled:
                            if phases.get(index)!=phase:raise RuntimeError('Unknown permission phase')
                            prompt=(('先取走松开的空壳，确认手内为空、臂未移位，拇指旋转退让范围清空，双手撤开。'
                                'r=确认手已空；如对掌已合格只读通过，否则仅目标64556退让一次；臂不动；q=退出：') if index==4 else '' if index in (3,13,14) else operator_prompt(index,state))
                            if index==14:
                                prompt=('确认同一空壳底部仍完全坐在原槽底，由板承托，臂手未移位/重启；撤手，清空无名指退让范围。'
                                    '确认含空壳的140mm上升和连接转运至后移40mm黑框路线清空，预计余量至少20mm，现场守急停。'
                                    'g=只退无名指至20000，反馈健康后直接提起/连续转运/正常投放；q=保持退出：')
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
                                prompt=('刚完成视觉m/d/p时，保持采图空壳在刚确认的位置，不要搬回旧槽，撤开双手。'
                                    '确认新手型各处无接触、拇指与食指能对向夹持；'
                                    '闭合范围及含空壳的140mm上升/学校过渡至黑框路线清空，预计余量至少20mm。'
                                    '现场守急停。g=一次授权夹持、提起、转运及到黑框正常张开；q=退出：')
                            while True:
                                answer=_live_input(prompt,process,state_path,phase,index,log)
                                try:
                                    if index==14:
                                        if answer.strip().lower()!='g':raise KeyboardInterrupt('Supported ring/normal cycle cancelled')
                                        event=dict(index=14,phase=phase,value='g',
                                            shell_fully_bottom_supported_original_slot=True,ring_withdrawal_sweep_clear=True,
                                            same_slot_support_and_healthy_feedback_before_lift_required=True,
                                            only_intended_shell_contact_and_closure_clear=True,
                                            supported_ring_retreat_and_continuous_cycle_authorized=True,
                                            continuous_lift_transfer_normal_drop_authorized=True,
                                            planned_loaded_route_min20mm_clear_confirmed=True,
                                            permission_is_prospective_not_postlift_observation=True,
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
        report['previous_pregrasp_replayed']=args.capture_current_target
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
    return 0 if (report['outcome']==SUCCESS and report.get('worker_returncode')==0
        and not report['exceptions'] and not report.get('worker',{}).get('cleanup_errors')) or (report['outcome']=='completed_empty_return_from_qualified_v6_bin' and report.get('worker_returncode')==0 and not report['exceptions'] and not report.get('worker',{}).get('cleanup_errors')) else 1


if __name__=='__main__':raise SystemExit(main())
