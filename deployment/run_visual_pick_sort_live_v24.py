"""Two actual wrists, image-region selection and fresh-camera AUTO batches."""
from datetime import datetime,timezone
import hashlib
import copy
import json
import os
from pathlib import Path
import select
import subprocess
import sys
import time
import zipfile

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/battery_grasp/src'))
from known_cylinder_records import dump
from visual_task_control_v24 import atomic_json
from visual_pick_sort_v23_source import (MANIFEST,SUCCESS_SHA,NEAR_SHA,verify_delivery,accepted_anchor,
    target_candidate,success_for,make_near_binding,accepted_near_binding,grid_matrix,post_drop_raise_mm)
from visual_pick_sort_v13_source import read_point_archive


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    # The UI owns the terminal; native SDK use stays in the original child.
    # Reject a lost descriptor before creating a task or launching any child.
    try:
        if sys.stdin is None or sys.stdin.closed or not sys.stdin.readable():
            raise RuntimeError('terminal input unavailable')
        os.fstat(sys.stdin.fileno())
    except (AttributeError,OSError,RuntimeError,ValueError):
        print('终端输入不可用。请从新终端直接启动fix11入口，勿经SDK恢复启动器交接。',file=sys.stderr,flush=True)
        return
    folder=ROOT/'captures'/datetime.now().strftime('visual_pick_sort_v23_%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True)
    workspace=ROOT/'calibration/visual_pick_sort_v20_workspace.json'
    anchor_path=ROOT/'calibration/visual_pick_sort_v14_anchor.json'
    near_path=ROOT/'calibration/visual_pick_sort_v21_near_binding.json'
    grid_path=ROOT/'calibration/visual_pick_sort_v21_wrist_regions.json'
    near_binding=regions=None
    report=dict(version=23,delivery_version='visual-task-control-v24-fix11-motion-lift40-20261026',state='IDLE',batches=[],attempts=[],captures=[],exceptions=[],
        physical_bin_delivery_sensor_verified=False,three_color_sorting_requested=True,
        offline_project_tests_executed=False,parent_SDK_loaded=False)
    stop_requested=False;active_batch=None;process=None;active_abort=None

    def save():
        dump(folder/'result.json',report)
        pointer=os.environ.get('BATTERY_CONTROL_POINTER')
        if pointer:
            atomic_json(pointer,dict(result_file=str(folder/'result.json'),
                stop_file=str(active_batch/'stop.json') if active_batch is not None else None,
                abort_file=str(active_abort) if active_abort is not None else None))

    def stop():
        nonlocal stop_requested
        stop_requested=True
        if active_batch is not None:dump(active_batch/'stop.json',dict(operator_requested_stop=True))

    def child(command,abort_file=None,progress_path=None,console_path=None):
        nonlocal process,active_abort
        stop_latch=os.environ.get('BATTERY_CONTROL_STOP')
        if stop_latch and Path(stop_latch).exists():
            stop();return 130
        active_abort=abort_file;save()
        log=console_path.open('w',encoding='utf8') if console_path is not None else None
        try:
            process=subprocess.Popen(command,cwd=str(ROOT),stdin=subprocess.DEVNULL,
                stdout=log,stderr=subprocess.STDOUT if log is not None else None)
        except BaseException:
            if log is not None:log.close()
            raise
        last_phase=None;last_progress=0.
        labels=dict(preflight='读取当前状态',opening_current_five_flex_before_arm_motion='张开手指',
            parking_empty_at_verified_upper='返回观察位',positioning_empty_at_selected_wrist_upper='高位切换腕姿',
            aligning_live_visual_target_at_high='高位对准目标',descending_live_visual_target='下降对准',
            preparing_after_descent_at_visual_target='预抓手型',grouped_grip_then_immediate180mm_lift='夹持',
            lifting_live_grip180mm='提起',carrying_to_verified_common_upper='转向投放路线',
            connected_school_selected_bin_transfer='连贯转运',opening_five_flex_at_current_selected_bin='投放张开',
            clearing_open_hand_above_selected_bin='投放后竖直抬升',
            checking_post_drop_open_positions_before_scan_return='核对张开',completed='收尾核对',fault='动作暂停',paused='等待设备反馈')
        labels['qualifying_original_prepared_pick_resume']='原位续抓核对'
        labels['qualifying_failed_transfer_common_upper']='核对持物共同高位后续投'
        labels['checking_selected_bin_reach']='核对本机臂展与转运目标'
        labels['checking_and_enabling_startup_protection']='核对并开启缺少的保护'
        labels['checking_hand_bus_after_restart']='准备末端通信并读取当前手部'
        try:
            while process.poll() is None:
                try:
                    if stop_latch and Path(stop_latch).exists():stop()
                    if progress_path is not None and time.monotonic()-last_progress>=.5:
                        last_progress=time.monotonic()
                        try:
                            progress=json.loads(progress_path.read_bytes());current=progress.get('phase')
                            if isinstance(current,str) and current and current!=last_phase:
                                label=labels.get(current,current)
                                if current=='clearing_open_hand_above_selected_bin':
                                    label+=str(int(progress.get('post_drop_vertical_raise_required_mm',80)))+'mm'
                                print('当前：'+label,flush=True);last_phase=current
                        except (OSError,ValueError,TypeError,AttributeError):pass
                    if select.select([sys.stdin],[],[],.1)[0]:
                        line=sys.stdin.readline()
                        if not line or line.strip().lower() in ('q','stop'):stop()
                    if stop_requested and abort_file is not None and not abort_file.exists():
                        dump(abort_file,dict(operator_requested_stop=True))
                    process.wait(timeout=.1)
                except subprocess.TimeoutExpired:pass
                except KeyboardInterrupt:stop()
            return process.returncode
        finally:
            if process.poll() is not None:process=None
            if log is not None:log.close()
            active_abort=None;save()

    def motion(mode,point=None,vision_archive=None):
        if mode not in ('park_empty','visual_cycle','resume_failed_transfer'):
            raise ValueError('Only fresh visual cycles/empty observation parking are offered by v23')
        attempt=folder/('cycle_%03d'%(len(report['attempts'])+1))
        permission=dict(exclusive=True,onsite_stop=True,inert_shell=True,fixed_head_board_mount=True,
            whole_route_and_sweep_clear=True,fresh_actual_open_posture_required=True,
            initial_stationary_group_open_authorized=True,initial_opening_and_inert_shell_drop_zone_clear=True,
            object_absence_verified=False,batch_route_and_scan_return_authorized=True,
            profile_carry_common_upper_authorized=True,enable_missing_arm_protection=True,
            initialize_hand_bus=mode in ('park_empty','visual_cycle'))
        profile='far';candidate=None
        if mode=='visual_cycle':
            candidate=target_candidate(point,anchor,near_binding,regions);profile=candidate['wrist_profile']
            print('本次腕姿：'+profile+'；颜色：'+candidate['target_color']+'；投放：'+candidate['logical_bin'],flush=True)
        elif mode=='resume_failed_transfer':
            from visual_transfer_failure_v23 import load_failed_transfer
            original,proof=load_failed_transfer(ROOT)
            profile=original['wrist_profile'];candidate=copy.deepcopy(original['candidate'])
            print('续投当前'+original['target_classification']['color']+'空壳；核对已完成共同高位，保持夹持，不重新抓取。',flush=True)
        _,source_sha=success_for(profile)
        request=dict(mode=mode,wrist_profile=profile,success_source_sha256=source_sha,manifest_sha256=sha(MANIFEST),
            scene_permission=permission,timestamp_utc=datetime.now(timezone.utc).isoformat())
        if mode=='visual_cycle':
            permission.update(target_unchanged_since_capture=True,continuous_pick_sort_authorized=True,
                post_drop_vertical_raise_and_return_authorized=True)
            request.update(anchor=anchor,near_binding=near_binding,wrist_regions=regions,point_record=point,candidate=candidate,
                vision_archive_sha256=sha(vision_archive),return_to_scan_upper_after_drop=True,
                target_classification=candidate['target_classification'],post_drop_vertical_raise_mm=post_drop_raise_mm(candidate['target_color']))
        elif mode=='resume_failed_transfer':
            for key in ('anchor','near_binding','wrist_regions','point_record','candidate','target_classification'):
                request[key]=copy.deepcopy(original[key])
            permission.update(same_shell_held_no_move_release_or_restart=True,
                resume_failed_transfer_authorized=True)
            request.update(transfer_failure_provenance=proof,return_to_scan_upper_after_drop=True,
                post_drop_vertical_raise_mm=post_drop_raise_mm(request['target_classification']['color']))
        attempt.mkdir()
        request_path=attempt/'request.json';output=attempt/'worker_result.json';dump(request_path,request)
        began=time.monotonic()
        row=dict(mode=mode,wrist_profile=profile,folder=attempt.name,request_sha256=sha(request_path))
        if candidate is not None:
            row.update(target_color=candidate['target_color'],logical_bin=candidate['logical_bin'])
            if point is not None:
                row['display_target']=dict(measurement_bbox_px=point['measurement_bbox_px'],
                    top_center_world_m=point['top_center_world_m'],coordinates_locked=True)
        report['attempts'].append(row);report['state']='MOVING';save()
        code=child([sys.executable,str(ROOT/'deployment/visual_pick_sort_v23_worker.py'),
            '--request',str(request_path),'--output',str(output),'--manifest',str(MANIFEST)],attempt/'abort.json',
            progress_path=output,console_path=attempt/'worker_console.log')
        value=json.loads(output.read_bytes()) if output.exists() else {}
        row.update(exit_code=code,outcome=value.get('outcome'),phase=value.get('phase'),
            result_sha256=sha(output) if output.exists() else None,elapsed_seconds=time.monotonic()-began)
        okay=(not code and value.get('phase')=='completed' and not value.get('exceptions')
            and not value.get('cleanup_errors') and value.get('SDK_cleanup_finished') is True)
        if mode in ('visual_cycle','resume_failed_transfer'):okay=okay and value.get('return_to_scan_automatic') is True and value.get('post_drop_clearance_completed') is True
        row['completed']=okay;save()
        if not okay:
            report['last_action_reason']='; '.join(str(e.get('error')) for e in value.get('exceptions',[])) or str(value.get('outcome') or ('worker exit '+str(code)))
            print('动作已暂停，保留主界面与本轮记录；不自动重抓、松手或回臂。',flush=True)
            for error in value.get('exceptions',[]):print(str(error.get('error')),flush=True)
            if not value.get('exceptions'):print(report['last_action_reason']+'；完整输出：'+str(attempt/'worker_console.log'),flush=True)
        return okay

    def camera(define=False,observe_only=False,binding=False):
        job=active_batch/('vision_%03d'%(len(report['captures'])+1));job.mkdir()
        response=job/'response.json';ledger=active_batch/'ledger.json'
        command=[sys.executable,str(ROOT/'deployment/visual_pick_sort_live_v24_vision.py'),
            '--role','target','--position-session',str(ROOT/'calibration/position_rgbd_20261020_001259_216986'),
            '--anchor',str(ROOT/'deployment/visual_reference_201634/task_anchor.json'),
            '--confirm-fixed-camera','--confirm-measured-shell-only','--confirm-unchanged-setup',
            '--confirm-upright-shell-on-this-board','--finish-after-first-reference',
            '--workspace-config',str(workspace),'--scan-response',str(response),
            '--stop-file',str(active_batch/'stop.json'),'--inference-python','/usr/bin/python3','--device','auto']
        if binding:command.append('--reference-black-only')
        if define:command.append('--define-workspace')
        elif not binding:
            command+=['--automatic','--completed-ledger',str(ledger)]
            if observe_only:command.append('--observe-only')
        report['state']='DEFINING_WORKSPACE' if define else 'OBSERVING';save()
        code=child(command)
        value=json.loads(response.read_bytes()) if response.exists() else dict(outcome='camera_response_missing')
        record=dict(exit_code=code,**value);report['captures'].append(record)
        archive=None
        if value.get('archive'):
            original=Path(value['archive']).resolve()
            if ROOT/'captures' not in original.parents:raise ValueError('Camera archive outside captures')
            archive=job/'camera.zip';archive.write_bytes(original.read_bytes())
            record.update(archive=str(archive.relative_to(folder)),sha256=sha(archive))
            with zipfile.ZipFile(archive) as evidence:
                if evidence.testzip() is not None:raise ValueError('Camera archive CRC failed')
                manifests=[n for n in evidence.namelist() if n.endswith('/source_hashes.json')]
                if len(manifests)!=1:raise ValueError('Camera evidence manifest is not unique')
                prefix=manifests[0][:-len('source_hashes.json')]
                hashes=json.loads(evidence.read(manifests[0]))
                for name,digest in hashes.items():
                    if '/' in name or '\\' in name or hashlib.sha256(evidence.read(prefix+name)).hexdigest()!=digest:
                        raise ValueError('Camera evidence file differs: '+name)
                captured=json.loads(evidence.read(prefix+'result.json'))
                if captured.get('outcome')!=value.get('outcome'):
                    raise ValueError('Camera response differs from archived result')
                for name,digest in captured.get('runtime_source_sha256',{}).items():
                    if sha(ROOT/'deployment'/name)!=digest:raise ValueError('Camera runtime source differs: '+name)
                record['verified_camera_evidence_files']=len(hashes)
        save()
        if code or value.get('error') or value.get('cleanup_errors'):
            value['outcome']='camera_not_completed'
        return value,archive

    def bind_near():
        nonlocal active_batch,stop_requested,near_binding,regions
        stop_requested=False
        attempts=report.setdefault('near_binding_attempts',[])
        active_batch=folder/('near_binding_%03d'%(len(attempts)+1));active_batch.mkdir()
        attempts.append(dict(folder=active_batch.name))
        dump(active_batch/'ledger.json',dict(completed=[]))
        if not motion('park_empty'):
            report['state']='PAUSED';save();return
        print('把刚才成功提起的同一空壳放回近处成功抓前的原位置/原支撑高度，落底撤手；头和板不动。',flush=True)
        print('窗口点这只空壳并按S；这里只记录新腕姿的视觉参考，不会执行旧腕姿抓取。',flush=True)
        result,archive=camera(binding=True)
        if stop_requested or archive is None or result.get('outcome') not in ('new_wrist_reference_point_saved','target_qualified'):
            report['state']='STOPPED' if stop_requested else 'IDLE';save();return
        point,evidence=read_point_archive(archive)
        binding=make_near_binding(point,anchor,True)
        image=active_batch/'binding_rgb.png';context=active_batch/'camera_context.json'
        with zipfile.ZipFile(archive) as z:
            marker=next(n for n in z.namelist() if n.endswith('/source_hashes.json'))
            prefix=marker[:-len('source_hashes.json')]
            image.write_bytes(z.read(prefix+point['observation']+'.png'))
        dump(context,point['binding_camera_context'])
        output=active_batch/'wrist_regions.json'
        print('接着只点四个角槽中心：近排左、近排右、远排右、远排左。S保存绿色五槽分配；只选择腕姿，XYZ仍实时识别。',flush=True)
        code=child([sys.executable,str(ROOT/'deployment/visual_wrist_regions_v21.py'),
            '--image',str(image),'--context',str(context),'--output',str(output),
            '--stop-file',str(active_batch/'stop.json')])
        if code or stop_requested or not output.exists():
            report['state']='STOPPED' if stop_requested else 'IDLE';save();return
        configured=json.loads(output.read_bytes());grid_matrix(configured,point['binding_camera_context'])
        for path,name in ((near_path,'previous_near_binding.json'),(grid_path,'previous_wrist_regions.json')):
            if path.exists():(active_batch/name).write_bytes(path.read_bytes())
        dump(near_path,binding);dump(grid_path,configured)
        near_binding=binding;regions=configured
        dump(active_batch/'near_binding.json',binding)
        report.update(state='IDLE',near_binding_recorded=True,near_binding_evidence=evidence,
            near_cells=[[0,0],[0,1],[0,2],[1,1],[1,2]])
        save();print('近处视觉绑定和五槽腕姿分配已保存。a自动，m单块；无需逐槽选择。',flush=True)

    def run_batch(limit,redefine=False):
        nonlocal active_batch,stop_requested
        if not redefine and (near_binding is None or regions is None):
            print('先b完成一次近处原位视觉绑定与槽区标记。',flush=True);return
        stop_requested=False
        active_batch=folder/('batch_%03d'%(len(report['batches'])+1));active_batch.mkdir()
        batch=dict(mode='WORKSPACE' if redefine else 'MANUAL' if limit==1 else 'AUTO',limit=limit,completed=[],
            started_utc=datetime.now(timezone.utc).isoformat(),
            command_cycles_completed=0,targets_absent_in_fresh_scan=0,state='STARTED')
        report['batches'].append(batch);dump(active_batch/'ledger.json',dict(completed=[]));save()
        if not redefine and not motion('park_empty'):
            batch['state']='PAUSED';report['state']='PAUSED';save();return
        if stop_requested:batch['state']='STOPPED';report['state']='STOPPED';save();return
        if redefine or not workspace.exists():
            result,_=camera(define=True)
            if result['outcome']!='workspace_defined' or stop_requested:
                batch['state']='WORKSPACE_NOT_DEFINED';report['state']='IDLE';save();return
        if redefine:
            batch['state']='WORKSPACE_DEFINED';report['state']='IDLE';save();return
        while not stop_requested:
            result,archive=camera(observe_only=batch['command_cycles_completed']>=limit)
            if stop_requested:break
            outcome=result['outcome']
            if outcome=='operator_stopped':stop();break
            if result.get('processed_targets_absent') is True:
                batch['targets_absent_in_fresh_scan']=batch['command_cycles_completed']
            if outcome=='workspace_no_supported_cylinder_observed':
                batch['state']='WORKSPACE_OBSERVED_EMPTY';report['state']='DONE'
                print('工作区连续观察未检出支持的圆柱电池，本批结束；方形不抓取。',flush=True);break
            if outcome=='workspace_observed' and batch['command_cycles_completed']>=limit:
                batch.update(state='LIMIT_COMPLETED',remaining_cylinder_detections=result['workspace_cylinder_count'])
                report['state']='DONE';break
            if outcome=='automatic_visual_recovery_exhausted':
                batch.update(state='VISION_PENDING',reason=outcome,
                    remaining_cylinder_detections=result.get('workspace_cylinder_count'),
                    visual_recovery=result.get('visual_recovery'))
                report['state']='VISION_PENDING';save()
                print('已完成%d轮；当前检出%d个，视觉尚未确认清空；未把待处理项计为完成，原图已保存。'%(
                    batch['command_cycles_completed'],result.get('workspace_cylinder_count',0)),flush=True)
                break
            if outcome!='target_qualified' or archive is None:
                batch.update(state='PAUSED',reason=outcome);report['state']='PAUSED'
                print('本批暂停：'+str(outcome)+'；原图与结果保留。',flush=True);break
            point,evidence=read_point_archive(archive)
            if batch['completed'] and result.get('processed_targets_absent') is True:
                batch['targets_absent_in_fresh_scan']=batch['command_cycles_completed']
            batch['current_target_evidence']=evidence;save()
            if not motion('visual_cycle',point,archive):batch['state']='PAUSED';report['state']='PAUSED';break
            batch['command_cycles_completed']+=1
            batch['completed'].append(dict(id=batch['command_cycles_completed'],wrist_profile=report['attempts'][-1]['wrist_profile'],
                measurement_bbox_px=point['measurement_bbox_px'][:],top_center_world_m=point['top_center_world_m'][:],
                target_classification=point['target_classification']))
            dump(active_batch/'ledger.json',dict(completed=batch['completed']))
            print('已完成取放并回观察位：%d；自动识别下一目标。'%batch['command_cycles_completed'],flush=True)
            save()
        if stop_requested:batch['state']='STOPPED';report['state']='STOPPED'
        batch['actual_bin_success_not_inferred_from_commands']=True
        save()

    try:
        report['verified_files']=len(verify_delivery())
        anchor=accepted_anchor(json.loads(anchor_path.read_bytes()))
        report['anchor_sha256']=sha(anchor_path)
        if near_path.exists() and grid_path.exists():
            try:
                near_binding=accepted_near_binding(json.loads(near_path.read_bytes()),anchor)
                regions=json.loads(grid_path.read_bytes());grid_matrix(regions,anchor['camera_context'])
            except Exception as exc:
                near_binding=regions=None
                report.setdefault('saved_binding_not_enabled',[]).append(dict(reason=repr(exc)))
                print('已保存的近处绑定未通过核对；b可重新绑定，未生成替代坐标。',flush=True)
        save()
        print('v24网页入口：持续实时画面；沿用fix11原动作与40/80mm抬升。移动30%、下降/提起15%。',flush=True)
        print('黑色→正常品盒；红色/白色→学校废品盒对应落点。仅45.8×98mm同规格直立断电圆柱，方形不抓。',flush=True)
        print('a/m授权整批/单件完整取放及返回；整段整手/含物路线和投放区清空，独占控制并现场守急停。',flush=True)
        print('q回车/Ctrl+C请求停止，软件停止不等同硬件急停。',flush=True)
        print('重启后空手使用a；c只用于未重启、未松手或移臂的原持物续投。',flush=True)
        while True:
            try:
                print('CONTROL_READY',flush=True)
                choice=input('a 三色自动 | m 单块 | c 当前持物续投 | q 退出 | w 工作区 | b 绑定 | r 重置：').strip().lower()
                if choice=='q':break
                if choice=='r':
                    stop_requested=False;report['state']='IDLE';save()
                    print('仅重置软件任务，未移动或清设备保护。',flush=True);continue
                if choice=='c':
                    stop_requested=False
                    okay=motion('resume_failed_transfer')
                    report['state']='IDLE' if okay else 'PAUSED';save()
                    if okay:print('当前持物续投及返回完成，a继续识别剩余目标。',flush=True)
                    continue
                if choice=='b':
                    if report['state'] in ('PAUSED','STOPPED'):
                        print('任务仍暂停；r只重置软件状态。',flush=True);continue
                    bind_near();continue
                if choice in ('a','m','w'):
                    if report['state'] in ('PAUSED','STOPPED'):
                        print('任务仍暂停；r仅重置软件任务。具体原因：'+str(report.get('last_action_reason',report['state'])),flush=True);continue
                    run_batch(9 if choice=='a' else 1,redefine=choice=='w')
                else:print('输入a自动处理，m只处理一块。',flush=True)
            except KeyboardInterrupt:stop();report['state']='STOPPED';save()
            except EOFError:break
            except Exception as exc:
                report['exceptions'].append(dict(error=repr(exc)));report['state']='PAUSED'
                report['last_action_reason']=str(exc);save()
                if isinstance(exc,RuntimeError) and str(exc)=='input(): lost sys.stdin':
                    report['state']='INPUT_UNAVAILABLE';save()
                    print('终端输入已丢失，保存记录后退出主界面；请从新终端直接启动。',flush=True)
                    break
                print('任务暂停，主界面保留：'+str(exc),flush=True)
    except Exception as exc:
        report['exceptions'].append(dict(error=repr(exc)));report['state']='STARTUP_NOT_COMPLETED';save()
        print('启动未完成：'+str(exc),flush=True)
    finally:
        save();hashes={p.relative_to(folder).as_posix():sha(p) for p in folder.rglob('*')
            if p.is_file() and p!=folder/'source_hashes.json'}
        dump(folder/'source_hashes.json',hashes)
        archive=folder.with_suffix('.zip')
        with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
            for p in sorted(folder.rglob('*')):
                if p.is_file():z.write(p,folder.name+'/'+p.relative_to(folder).as_posix())
        print('SAVED:',archive,'SHA256:',sha(archive),flush=True)


if __name__=='__main__':main()
