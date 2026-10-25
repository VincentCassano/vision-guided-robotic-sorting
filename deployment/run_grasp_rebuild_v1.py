"""Sequential onsite teaching, automatic180mm pickup test and empty return.

No SDK connection is retained at a teaching prompt. Each stage has its own
raw evidence; a human verdict never substitutes for completed motion feedback.
"""
from datetime import datetime, timezone
import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/battery_grasp/src'))
MANIFEST=ROOT/'deployment/grasp-rebuild-v1-files.sha256'
RETIRED=ROOT/'calibration/new_wrist_reference_pending.json'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--resume-session',type=Path,
        help='Reuse records only after the known configuration-log failure before SDK/motor writes')
    args=parser.parse_args()
    from known_cylinder_records import dump
    from grasp_rebuild_records_v1 import targets,hand_snapshot,arm_reference,assemble_success
    from battery_grasp.real_grasp import orientation_angle_rad
    folder=ROOT/'captures'/datetime.now().strftime('grasp_rebuild_v1_%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True,exist_ok=False)
    report=dict(version=1,scope='start_grip_prepare_wrist_automatic180mm_lift_and_empty_return',
        recording_order=['start','grip_shape','prepare_shape','wrist','automatic_lift180mm'],
        stage='start',outcome='recording',lift_mm=180,iterations=[],exceptions=[],
        parent_SDK_loaded=False,controller_configuration_writes=0,
        holding_sensor_verified=False,complete_visual_pick_sort_accepted=False,
        offline_project_tests_executed=False,execution_ready=False)
    start=None;cycle=None;child=None;phase='start'
    grip=prepare=wrist=lift_result=lift_request=lift_path=lift_output=None
    done=False

    def save():
        report['stage']=phase
        dump(folder/'result.json',report)

    def note(error):
        report['exceptions'].append(dict(stage=phase,error=repr(error),
            timestamp_utc=datetime.now(timezone.utc).isoformat()))
        save();print(str(error)+'；当前步骤保留。',flush=True)

    def run_child(command,motion_output=None):
        nonlocal child
        child=subprocess.Popen(command,cwd=str(ROOT))
        try:
            while child.poll() is None:
                try:child.wait(timeout=.25)
                except subprocess.TimeoutExpired:continue
                except KeyboardInterrupt:
                    if motion_output is not None:
                        try:
                            dump(motion_output.parent/'abort.json',dict(operator_requested_stop=True,
                                timestamp_utc=datetime.now(timezone.utc).isoformat()))
                            print('已请求停止，等待动作进程关闭连接。',flush=True)
                        except Exception as exc:
                            print('停止请求文件未写入：'+str(exc)+'；仍等待子进程退出，不启动第二个控制。',flush=True)
                    else:
                        print('等待当前记录进程退出后再继续；不会启动第二个控制。',flush=True)
            return child.returncode
        finally:
            if child is not None and child.poll() is not None:child=None

    def hand_read(destination):
        from manual_hand_bus_v2 import ManualHandBus
        def log(row):
            with destination.with_suffix('.jsonl').open('a',encoding='utf8') as stream:
                stream.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
        bus=ManualHandBus(log)
        try:
            row=bus.connect_read();dump(destination,row);return row
        finally:bus.close()

    def new_cycle():
        nonlocal cycle,grip,prepare,wrist,lift_result,lift_request,lift_path,lift_output
        number=len(report['iterations'])+1
        cycle=folder/('cycle_%03d'%number);cycle.mkdir()
        report['iterations'].append(dict(number=number,folder=cycle.name,outcome='recording'))
        grip=prepare=wrist=lift_result=lift_request=lift_path=lift_output=None
        save()

    def same_arm(actual,expected,label):
        differences=dict(joints_deg=[a-b for a,b in zip(actual['joints'],expected['joints'])],
            position_mm=sum((a-b)**2 for a,b in zip(actual['pose'][:3],expected['pose'][:3]))**.5*1000,
            orientation_rad=orientation_angle_rad(actual['pose'][3:],expected['pose'][3:]))
        if (actual['frame_signatures']!=expected['frame_signatures'] or actual['limits']!=expected['limits']
                or max(abs(v) for v in differences['joints_deg'])>.08
                or differences['position_mm']>.5 or differences['orientation_rad']>.005):
            raise ValueError(label+'不符：'+str(differences))
        return differences

    def read_pose(role):
        attempt=cycle/(role+'_'+datetime.now().strftime('%H%M%S_%f'))
        attempt.mkdir()
        print('读取'+('开始位置' if role=='start' else '腕姿')+'，保持静止。',flush=True)
        before=hand_read(attempt/'hand_before.json');first=hand_snapshot(before)
        code=run_child([sys.executable,str(ROOT/'deployment/new_grasp_reference_worker_v1.py'),
            '--output',str(attempt/'arm.json')])
        if not (attempt/'arm.json').exists():raise ValueError('臂位读取未返回结果')
        arm=json.loads((attempt/'arm.json').read_bytes())
        if code:raise ValueError('臂位读取未完成：'+str(arm.get('exceptions',[])))
        current=arm_reference(arm)
        after=hand_read(attempt/'hand_after.json');last=hand_snapshot(after)
        if any(abs(a-b)>2 for a,b in zip(first['targets'],last['targets'])):
            raise ValueError('记录期间手指目标改变，请停稳后再记录')
        if role=='wrist':
            if any(abs(a-b)>2 for a,b in zip(last['targets'],prepare['targets'])):
                raise ValueError('当前手型已改变，请先返回预抓取手势记录')
            original=arm_reference(start['arm'])
            if current['frame_signatures']!=original['frame_signatures'] or current['limits']!=original['limits']:
                raise ValueError('本轮开始后坐标系或关节限位改变')
        record=dict(arm=arm,hand=after,source_folder=str(attempt),
            timestamp_utc=datetime.now(timezone.utc).isoformat())
        dump(attempt/'record.json',record)
        print('已记录，SDK及手连接已关闭。XYZ(mm)：'+str([round(v*1000,3) for v in current['pose'][:3]]),flush=True)
        return record

    def capture_shape(role):
        output=cycle/(role+'_shape.json')
        if output.exists():output.replace(cycle/(role+'_superseded_'+datetime.now().strftime('%H%M%S_%f')+'.json'))
        code=run_child([sys.executable,str(ROOT/'deployment/grasp_rebuild_hand_v1.py'),
            '--role',role,'--output',str(output),'--manifest',str(MANIFEST)])
        if code or not output.exists():return None
        record=json.loads(output.read_bytes())
        if record.get('recorded') is not True or record.get('cleanup_errors'):
            raise ValueError('手势尚未记录或连接清理未完成')
        targets(record.get('targets'))
        return record

    def common_request():
        return dict(start_record=start,wrist_record=wrist,grip_targets=grip['targets'],
            prepare_targets=prepare['targets'],manifest_sha256=sha(MANIFEST))

    def resume_before_motion(source):
        nonlocal start,grip,prepare,wrist,phase
        source=source.resolve()
        captures=(ROOT/'captures').resolve()
        if (source.parent!=captures or not source.name.startswith('grasp_rebuild_v1_')
                or source==folder or not source.is_dir()):
            raise ValueError('续接必须指向captures内原grasp_rebuild_v1会话目录')
        previous=json.loads((source/'result.json').read_bytes())
        iterations=previous.get('iterations',[])
        if not iterations:raise ValueError('原会话没有已记录的本轮数据')
        last=iterations[-1]
        request_path=(source/last['lift_request']).resolve()
        source_cycle=(source/last['folder']).resolve()
        if (source not in request_path.parents or source not in source_cycle.parents
                or request_path.parent.parent!=source_cycle):
            raise ValueError('原始记录路径越出会话目录')
        result_path=request_path.with_name('worker_result.json')
        request=json.loads(request_path.read_bytes());result=json.loads(result_path.read_bytes())
        known='AttributeError("\'function\' object has no attribute \'write\'")'
        errors=result.get('exceptions',[])
        actions=last.get('actions',[])
        if (request.get('mode')!='lift' or result.get('request_sha256')!=sha(request_path)
                or len(errors)!=1 or errors[0].get('error')!=known
                or 'wire.write' not in errors[0].get('traceback','')
                or result.get('failed_phase')!='preflight' or previous.get('stage')!='verdict'
                or not actions or actions[-1].get('mode')!='lift'
                or (source/actions[-1].get('result_file','')).resolve()!=result_path
                or result.get('phase')!='fault'
                or result.get('outcome')!='lift_not_completed_waiting_operator_recovery'
                or any(type(result.get(key)) is not int or result[key]!=0 for key in
                    ('arm_actuator_writes','hand_register_writes','arm_stop_writes','controller_configuration_writes'))
                or any(result.get(key) for key in
                    ('sdk_calls','motion_requests','hand_commands','cleanup_errors'))
                or any(key in result for key in
                    ('sdk_socket_state_raw','sdk_environment','hand_connect_read','fresh_startup','protection_before'))
                or any(key.startswith('qualified_') for key in result)):
            raise ValueError('原结果不是本次SDK加载/动作前的日志接口错误，不能按此选项续接')
        old_grip=json.loads((source_cycle/'grip_shape.json').read_bytes())
        old_prepare=json.loads((source_cycle/'prepare_shape.json').read_bytes())
        if (old_grip.get('recorded') is not True or old_prepare.get('recorded') is not True
                or old_grip.get('cleanup_errors') or old_prepare.get('cleanup_errors')
                or targets(old_grip.get('targets'))!=targets(request.get('grip_targets'))
                or targets(old_prepare.get('targets'))!=targets(request.get('prepare_targets'))):
            raise ValueError('原手势记录与提起请求不一致')
        old_start=request['start_record'];old_wrist=request['wrist_record']
        saved_wrist=json.loads((source_cycle/'wrist_record.json').read_bytes())
        if last.get('start_record')!=old_start or saved_wrist!=old_wrist:
            raise ValueError('原开始点或腕姿记录与动作请求不一致')
        arm_reference(old_start['arm'],'原本轮开始位置')
        arm_reference(old_wrist['arm'],'原本轮健康腕姿')
        hand_snapshot(old_start['hand'],'原开始手型')
        old_hand=hand_snapshot(old_wrist['hand'],'原抓前手型')
        if any(abs(a-b)>2 for a,b in zip(old_hand['targets'],old_prepare['targets'])):
            raise ValueError('原抓前手型与预抓记录不一致')
        destination=folder/'resumed_source'/source.name
        shutil.copytree(source,destination)
        start=old_start;grip=old_grip;prepare=old_prepare;wrist=old_wrist
        dump(cycle/'grip_shape.json',grip);dump(cycle/'prepare_shape.json',prepare)
        dump(cycle/'wrist_record.json',wrist)
        report['resume_source']=dict(session=str(source),preserved_copy=str(destination.relative_to(folder)),
            failed_request_sha256=sha(request_path),failed_result_sha256=sha(result_path),
            original_healthy_wrist_used=True,fault_last_state_used=False,
            prior_SDK_never_loaded=True,prior_arm_and_hand_writes=0)
        report['iterations'][-1].update(start_record=start,outcome='resumed_at_test_before_any_prior_action')
        phase='test';save()
        print('已恢复开始位置、两组手势和健康腕姿；直接从第⑤步继续。',flush=True)
        print('t前重新读取并匹配当前臂/手，原失败点不作新起点。',flush=True)

    def perform(mode):
        nonlocal phase,lift_result,lift_request,lift_path,lift_output
        action=cycle/(mode+'_'+datetime.now().strftime('%H%M%S_%f'));action.mkdir()
        request=common_request()
        request['mode']=mode
        if mode=='lift':
            request['operator_confirms_lift_path_and_sweep_clear']=True
        else:
            request.update(operator_confirms_empty_original_shell_supported_and_return_sweep_clear=True,
                previous_lift_request_file=str(lift_path),previous_lift_result_file=str(lift_output),
                previous_lift_request_sha256=sha(lift_path))
            if lift_result.get('outcome')=='lift_completed_waiting_operator_result':
                request['expected_upper']=lift_result['qualified_upper']
        request_path=action/'request.json';output=action/'worker_result.json'
        dump(request_path,request)
        if mode=='lift':
            lift_request=request;lift_path=request_path;lift_output=output
            phase='verdict';report['iterations'][-1]['lift_request']=str(request_path.relative_to(folder));save()
        code=run_child([sys.executable,str(ROOT/'deployment/grasp_rebuild_motion_v1_worker.py'),
            '--request',str(request_path),'--output',str(output),'--manifest',str(MANIFEST)],output)
        result=json.loads(output.read_bytes()) if output.exists() else dict(
            phase='fault',outcome='worker_result_missing',SDK_cleanup_finished=False)
        report['iterations'][-1].setdefault('actions',[]).append(dict(mode=mode,
            result_file=str(output.relative_to(folder)),exit_code=code,outcome=result.get('outcome')))
        if mode=='lift':
            lift_result=result
            if code==0 and result.get('outcome')=='lift_completed_waiting_operator_result':
                print('已完成180mm上升。现在看空壳是否确实夹起且稳定。',flush=True)
            else:
                reasons=[row.get('error',str(row)) for row in result.get('exceptions',[]) if isinstance(row,dict)]
                print('本次未完成：'+'；'.join(reasons or [str(result.get('outcome'))]),flush=True)
                print('界面保留；没有自动张开或回臂。',flush=True)
            save();return
        if code or result.get('outcome')!='returned_to_recorded_start':
            reasons=[row.get('error',str(row)) for row in result.get('exceptions',[]) if isinstance(row,dict)]
            raise ValueError('未确认复原完成：'+'；'.join(reasons or [str(result.get('outcome'))]))
        report['iterations'][-1]['outcome']='pickup_failed_returned_to_start'
        print('已复原开始臂位和开始手型，重新记录本轮开始位置。',flush=True)
        phase='start';new_cycle();save()

    def success():
        nonlocal done
        record=assemble_success(start,grip,prepare,wrist,lift_result,True)
        record['timestamp_utc']=datetime.now(timezone.utc).isoformat()
        record['source_session']=str(folder)
        dump(cycle/'success_record.json',record)
        pointer=ROOT/'calibration/grasp_rebuild_success.json';pointer.parent.mkdir(exist_ok=True)
        if pointer.exists():(cycle/'previous_success_pointer.json').write_bytes(pointer.read_bytes())
        dump(pointer,record)
        report.update(outcome='operator_confirmed_pickup_and180mm_lift_saved',
            success_file=str((cycle/'success_record.json').relative_to(folder)))
        report['iterations'][-1]['outcome']='operator_confirmed_pickup_success'
        save();done=True
        print('成功手势、腕姿、开始点和180mm终点已保存。现在仍持物，不自动张开。',flush=True)

    try:
        inventory={}
        for line in MANIFEST.read_text(encoding='ascii').splitlines():
            digest,name=line.split('  ',1);item=(ROOT/name).resolve()
            if ROOT not in item.parents or name in inventory or sha(item)!=digest:
                raise ValueError('交付文件不符：'+name)
            inventory[name]=digest
        if not inventory:raise ValueError('交付清单为空')
        report['delivery_manifest_sha256']=sha(MANIFEST);report['delivery_source_sha256']=inventory
        RETIRED.parent.mkdir(exist_ok=True)
        if RETIRED.exists():(folder/'previous_retirement_marker.json').write_bytes(RETIRED.read_bytes())
        dump(RETIRED,dict(old_reference_retired=True,session=str(folder),
            reason='operator_requested_new_start_shapes_wrist_and180mm_lift',
            motion_integration_pending=True,execution_ready=False))
        print('开始位置 → 抓取手势 → 预抓取手势 → 腕姿 → 自动提起180mm。',flush=True)
        print('仅同一断电空壳；退出其他控制。示教只在主界面等待时操作，记录前退出示教并停稳。',flush=True)
        print('y成功保存；n=未夹起、手为空、空壳仍由原位承托且整段回程/调指范围清空，自动复原再循环。',flush=True)
        new_cycle()
        if args.resume_session is not None:resume_before_motion(args.resume_session)
        while not done:
            try:
                if phase=='start':
                    answer=input('① 空手置于无障碍开始位置；s记录 | q退出：').strip().lower()
                    if answer=='q':break
                    if answer not in ('s',''):continue
                    current=read_pose('start')
                    if start is not None:
                        same_arm(arm_reference(current['arm']),arm_reference(start['arm']),'复原开始位置')
                        if any(abs(a-b)>2 for a,b in zip(current['hand']['snapshot']['targets'],
                                start['hand']['snapshot']['targets'])):
                            raise ValueError('复原后的开始手型与原始记录不符')
                    else:start=copy.deepcopy(current)
                    report['iterations'][-1]['start_record']=current
                    phase='grip';save()
                elif phase in ('grip','prepare'):
                    print('② 记录抓取手势，x完成。' if phase=='grip' else '③ 改成预抓取手势，x完成。',flush=True)
                    record=capture_shape(phase)
                    if record is None:
                        if input('本阶段未保存；回车继续 | q退出：').strip().lower()=='q':break
                        continue
                    if phase=='grip':grip=record;phase='prepare'
                    else:prepare=record;phase='wrist'
                    save()
                elif phase=='wrist':
                    answer=input('④ SDK已关。保持预抓手型，示教摆好抓前腕姿；退出示教停稳后r记录 | p重录预抓 | q退出：').strip().lower()
                    if answer=='q':break
                    if answer=='p':phase='prepare';wrist=None;save();continue
                    if answer not in ('r',''):continue
                    wrist=read_pose('wrist');dump(cycle/'wrist_record.json',wrist)
                    phase='test';save()
                elif phase=='test':
                    answer=input('⑤ 确认空壳原位承托、上升180mm及整手扫掠区清空；t自动抓取提起 | r重摆腕姿 | q退出：').strip().lower()
                    if answer=='q':break
                    if answer=='r':wrist=None;phase='wrist';save();continue
                    if answer in ('t',''):perform('lift')
                elif phase=='verdict':
                    answer=input('y确实夹起且稳定，保存 | n未夹起且满足空手复原条件，自动复原 | q保持退出：').strip().lower()
                    if answer=='q':break
                    if answer=='y':success()
                    elif answer=='n':
                        if lift_result is None:raise ValueError('动作结果缺失，不能自动复原')
                        perform('return_empty')
            except KeyboardInterrupt:
                print('当前步骤保留；不重发动作。',flush=True)
            except EOFError:break
            except Exception as exc:note(exc)
    except Exception as exc:
        report['outcome']='startup_not_completed';note(exc)
    finally:
        save()
        hashes={p.relative_to(folder).as_posix():sha(p) for p in folder.rglob('*')
            if p.is_file() and p!=folder/'source_hashes.json'}
        dump(folder/'source_hashes.json',hashes)
        archive=folder.with_suffix('.zip')
        with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as out:
            for p in sorted(folder.rglob('*')):
                if p.is_file():out.write(p,folder.name+'/'+p.relative_to(folder).as_posix())
        print('SAVED:',archive,'SHA256:',sha(archive),flush=True)


if __name__=='__main__':main()
