"""Fresh image target -> new-wrist ordered grasp/lift/normal black-bin opening.

The parent has no SDK. One upfront g authorizes the complete supervised route.
"""
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
import zipfile

ROOT=Path(__file__).resolve().parents[1]


def main():
    from known_cylinder_records import dump
    from visual_pick_sort_v13_source import verify_delivery,accepted_anchor,read_point_archive,SOURCE_ANCHOR_SHA,MANIFEST
    from new_grasp_task_anchor_v1 import target_candidate
    folder=ROOT/'captures'/datetime.now().strftime('visual_pick_sort_v13_%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True,exist_ok=False)
    report=dict(scope='fresh_visual_target_new_wrist_ordered_single_black_shell_sort',
        version=13,revision='fix2',outcome='started',parent_SDK_loaded=False,execution_ready=False,
        source_anchor_sha256=SOURCE_ANCHOR_SHA,exceptions=[],attempts=[],
        automatic_motion_or_object_retention_accepted=False,offline_project_tests_executed=False)
    process=None;point=candidate=None;selected_archive=None

    def save():dump(folder/'result.json',report)

    def child(command,worker_folder=None):
        nonlocal process
        process=subprocess.Popen(command,cwd=str(ROOT))
        last=None
        try:
            while process.poll() is None:
                if worker_folder is not None:
                    state=worker_folder/'worker_result.json'
                    try:
                        phase=json.loads(state.read_bytes()).get('phase')
                        if phase!=last:last=phase;print('PHASE:',phase,flush=True)
                    except (FileNotFoundError,ValueError):pass
                time.sleep(.2)
            return process.returncode
        except KeyboardInterrupt:
            if worker_folder is not None:
                dump(worker_folder/'abort.json',dict(operator_interrupt=True,
                    timestamp_utc=datetime.now(timezone.utc).isoformat()))
                print('已发软件中止请求；现场确认停止，意外运动使用硬件急停。',flush=True)
                # Let the worker reach its existing stop/SDK cleanup path.
                while process.poll() is None:
                    try:process.wait(timeout=1)
                    except subprocess.TimeoutExpired:continue
                    except KeyboardInterrupt:print('仍在等待子进程清理；现场硬件停止优先。',flush=True)
            else:
                if process.poll() is None:process.terminate()
                try:process.wait(timeout=10)
                except subprocess.TimeoutExpired:process.kill();process.wait()
            raise
        finally:process=None

    def capture_target():
        nonlocal point,candidate,selected_archive
        point=candidate=selected_archive=None
        before=set((ROOT/'captures').glob('visual_pick_target_v13_*.zip'))
        command=[sys.executable,str(ROOT/'deployment/run_visual_pick_target_v13.py'),
            '--position-session',str(ROOT/'calibration/position_rgbd_20261020_001259_216986'),
            '--anchor',str(ROOT/'deployment/visual_reference_201634/task_anchor.json'),
            '--confirm-fixed-camera','--confirm-measured-shell-only','--confirm-unchanged-setup',
            '--confirm-upright-equal-height-recesses','--finish-after-first-reference',
            '--inference-python','/usr/bin/python3','--device','auto']
        code=child(command)
        created=set((ROOT/'captures').glob('visual_pick_target_v13_*.zip'))-before
        if code or len(created)!=1:raise ValueError('本轮目标尚未保存；s可重新选择，未发臂动作。')
        source=created.pop();point,evidence=read_point_archive(source)
        candidate=target_candidate(point,anchor)
        copy_path=folder/('vision_target_%03d.zip'%(len(report.get('targets',[]))+1))
        copy_path.write_bytes(source.read_bytes());selected_archive=copy_path
        item=dict(vision_file=copy_path.name,vision_evidence=evidence,point_record=point,candidate=candidate)
        report.setdefault('targets',[]).append(item);dump(folder/'latest_target.json',item);save()
        print('本轮视觉预抓TCP(mm):',[round(v*1000,3) for v in candidate['pregrasp_tcp_candidate_m_rad'][:3]],flush=True)
        print('新腕姿(rad):',candidate['fixed_wrist_rpy_rad'],'；无旧2.8+4.5mm偏移。目标保存后别再移动空壳。',flush=True)

    def execute():
        nonlocal point,candidate,selected_archive
        if candidate is None:
            print('先s选择本轮当前电池；旧目标不会自动补位。',flush=True);return
        print('已收到 g；开始读取当前设备状态，随后执行整轮。',flush=True)
        attempt=folder/('cycle_%03d'%(len(report['attempts'])+1));attempt.mkdir()
        record=dict(anchor=anchor,source_anchor_sha256=SOURCE_ANCHOR_SHA,
            point_record=point,candidate=candidate,
            vision_archive_sha256=hashlib.sha256(selected_archive.read_bytes()).hexdigest(),
            scene_permission=dict(hand_empty=True,inert_shell=True,fixed_head_board_mount=True,
                exclusive=True,onsite_stop=True,whole_route_min20mm_clear=True,
                continuous_cycle_authorized=True,timestamp_utc=datetime.now(timezone.utc).isoformat()))
        target_path=attempt/'target_record.json';dump(target_path,record)
        # Invalidate this scene immediately. No retry after a possibly acted-on target.
        point=candidate=selected_archive=None
        item=dict(folder=attempt.name,started_utc=datetime.now(timezone.utc).isoformat())
        report['attempts'].append(item);save()
        command=[sys.executable,str(ROOT/'deployment/visual_pick_sort_v13_worker.py'),
            '--folder',str(attempt),'--target-record',str(target_path),'--execute',
            '--manifest-sha256',hashlib.sha256(MANIFEST.read_bytes()).hexdigest(),
            '--sdk-dir','/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package']
        item['command']=command;save()
        code=child(command,attempt);item['worker_exit_code']=code
        result_file=attempt/'worker_result.json'
        result=json.loads(result_file.read_bytes()) if result_file.exists() else {}
        item.update(worker_result=result,worker_result_sha256=hashlib.sha256(result_file.read_bytes()).hexdigest() if result_file.exists() else None);save()
        if code or result.get('exceptions') or result.get('cleanup_errors') or result.get('phase')!='completed':
            report['outcome']='worker_not_completed_no_automatic_retry';save()
            print('动作进程结果：退出码=%s，阶段=%s。'%(code,result.get('phase','未返回阶段')),flush=True)
            for error in result.get('exceptions',[])+result.get('cleanup_errors',[]):
                print('具体原因：',error.get('error',str(error)) if isinstance(error,dict) else str(error),flush=True)
            diagnostic=result.get('joint_reserve_diagnostic',{})
            if diagnostic.get('failed_joints'):
                print('触发阶段：',diagnostic.get('phase'),flush=True)
                for row in diagnostic.get('joints',[]):
                    if row.get('joint') in diagnostic['failed_joints']:
                        print('J%s：当前%.3f°，范围[%.3f°, %.3f°]，距下限%.3f°/上限%.3f°。'%(
                            row['joint'],row['angle_degrees'],row['minimum_degrees'],row['maximum_degrees'],
                            row['lower_margin_degrees'],row['upper_margin_degrees']),flush=True)
            print('本轮未完成，原始数据已保存；未自动张开/回臂/重发。先现场确认状态；q保存退出。',flush=True)
            return
        report['outcome']='motion_cycle_completed_result_pending';save()
        print('指令流程已到投放结束。请按现场事实回答，不用再重跑。',flush=True)
        answer=input('y=确实夹起并投入黑框、手已空；f=漏抓/未入框且手已空；q=不确定或仍持物：').strip().lower()
        item['operator_result']=answer
        report['outcome']=('new_visual_pick_sort_observed_success' if answer=='y'
            else 'motion_cycle_completed_pickup_or_drop_failed' if answer=='f' else 'motion_cycle_completed_result_unconfirmed')
        report['automatic_motion_or_object_retention_accepted']=answer=='y'
        report['operator_observation_source']='onsite_after_normal_drop_not_sensor_or_ACK'
        save();return True

    try:
        report['checked_files']=len(verify_delivery());anchor=accepted_anchor();save()
        print('v13-fix2：使用已成功绑定的新腕姿。仅同一断电黑壳、直立同高工作面；头/板/安装不变，退出其他控制。',flush=True)
        print('g授权整轮前：手必须为空；当前竖直升至100mm中间点、五指准备、移向目标并升至140mm及整段含物路线均清空，预计余量≥20mm，现场守急停。',flush=True)
        print('s：点当前电池，S保存；g：100mm中间点→移向目标并升至140mm→下降→小/无名保持→拇→食→中→提起→黑框张开。夹住后不等键盘。',flush=True)
        initial_capture=True
        while True:
            try:
                if initial_capture:
                    initial_capture=False;capture_target();continue
                answer=input('s新采当前目标 | g确认条件并执行整轮 | q保存退出（输入后回车）：').strip().lower()
                if answer=='q':break
                if answer=='s':capture_target()
                elif answer=='g':
                    if execute():break
                else:print('输入s、g或q。',flush=True)
            except KeyboardInterrupt:
                report.setdefault('operator_interrupts',[]).append(datetime.now(timezone.utc).isoformat());save()
                print('返回界面；未自动重试。',flush=True)
            except EOFError:break
            except Exception as exc:
                report['exceptions'].append(dict(error=repr(exc),timestamp_utc=datetime.now(timezone.utc).isoformat()));save()
                print(str(exc),'；界面保留。',flush=True)
    except Exception as exc:
        report['exceptions'].append(dict(error=repr(exc)));report['outcome']='preflight_or_initial_capture_not_completed';save()
        print('未开始运动：',str(exc),flush=True)
    finally:
        report['execution_ready']=False;save()
        hashes={p.relative_to(folder).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
            for p in folder.rglob('*') if p.is_file() and p.name!='source_hashes.json'}
        dump(folder/'source_hashes.json',hashes)
        archive=folder.with_suffix('.zip')
        with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as out:
            for p in sorted(folder.rglob('*')):
                if p.is_file():out.write(p,folder.name+'/'+p.relative_to(folder).as_posix())
        print('SAVED:',archive,'SHA256:',hashlib.sha256(archive.read_bytes()).hexdigest(),flush=True)


if __name__=='__main__':main()
