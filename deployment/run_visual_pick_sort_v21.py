"""Two actual wrists, image-region selection and fresh-camera AUTO batches."""
from datetime import datetime,timezone
import hashlib
import copy
import json
from pathlib import Path
import select
import subprocess
import sys
import time
import zipfile

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/battery_grasp/src'))
from known_cylinder_records import dump
from visual_pick_sort_v21_source import (MANIFEST,SUCCESS_SHA,NEAR_SHA,verify_delivery,accepted_anchor,
    target_candidate,success_for,make_near_binding,accepted_near_binding,grid_matrix)
from visual_pick_sort_v13_source import read_point_archive


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    folder=ROOT/'captures'/datetime.now().strftime('visual_pick_sort_v21_%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True)
    workspace=ROOT/'calibration/visual_pick_sort_v20_workspace.json'
    anchor_path=ROOT/'calibration/visual_pick_sort_v14_anchor.json'
    near_path=ROOT/'calibration/visual_pick_sort_v21_near_binding.json'
    grid_path=ROOT/'calibration/visual_pick_sort_v21_wrist_regions.json'
    near_binding=regions=None
    report=dict(version=21,state='IDLE',batches=[],attempts=[],captures=[],exceptions=[],
        physical_bin_delivery_sensor_verified=False,other_color_work_skipped_by_user=True,
        offline_project_tests_executed=False,parent_SDK_loaded=False)
    stop_requested=False;active_batch=None;process=None

    def save():dump(folder/'result.json',report)

    def stop():
        nonlocal stop_requested
        stop_requested=True
        if active_batch is not None:dump(active_batch/'stop.json',dict(operator_requested_stop=True))

    def child(command,abort_file=None):
        nonlocal process
        process=subprocess.Popen(command,cwd=str(ROOT),stdin=subprocess.DEVNULL)
        try:
            while process.poll() is None:
                try:
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

    def motion(mode,point=None,vision_archive=None,resume=None):
        attempt=folder/('cycle_%03d'%(len(report['attempts'])+1))
        permission=dict(exclusive=True,onsite_stop=True,inert_shell=True,fixed_head_board_mount=True,
            whole_route_and_sweep_clear=True,fresh_actual_open_posture_required=True,
            initial_stationary_group_open_authorized=True,initial_opening_and_inert_shell_drop_zone_clear=True,
            object_absence_verified=False,batch_route_and_scan_return_authorized=True,
            profile_carry_common_upper_authorized=True)
        profile='near' if mode in ('deliver_held','resume_transfer','deliver_current_held') else 'far'
        candidate=None
        if mode=='visual_cycle':
            candidate=target_candidate(point,anchor,near_binding,regions);profile=candidate['wrist_profile']
            print('本次腕姿：'+profile+'；区域：'+str(candidate['wrist_region']),flush=True)
        _,source_sha=success_for(profile)
        request=dict(mode=mode,wrist_profile=profile,success_source_sha256=source_sha,manifest_sha256=sha(MANIFEST),
            scene_permission=permission,timestamp_utc=datetime.now(timezone.utc).isoformat())
        if mode=='visual_cycle':
            permission.update(target_unchanged_since_capture=True,continuous_pick_sort_authorized=True)
            request.update(anchor=anchor,near_binding=near_binding,wrist_regions=regions,point_record=point,candidate=candidate,
                vision_archive_sha256=sha(vision_archive),return_to_scan_upper_after_drop=True)
        if mode=='deliver_held':
            permission.update(same_current_held_shell=True,no_release_move_or_restart_since_success=True)
            request['return_to_scan_upper_after_drop']=True
        if mode in ('resume_transfer','deliver_current_held'):
            original,proof=resume[:2]
            permission.update(same_current_held_shell=True,no_release_move_or_restart_since_success=True,
                resume_current_held_after_completed_lift_confirmed=True)
            if mode=='deliver_current_held':
                permission.pop('no_release_move_or_restart_since_success')
                permission.pop('resume_current_held_after_completed_lift_confirmed')
                permission.update(operator_confirms_current_shell_is_stably_held=True,
                    current_pose_takeover_not_original_endpoint_resume=True,
                    current_whole_held_route_to_common_upper_clear=True)
            request=copy.deepcopy(original)
            request.update(mode=mode,manifest_sha256=sha(MANIFEST),scene_permission=permission,
                transfer_resume_provenance=proof,return_to_scan_upper_after_drop=True,
                timestamp_utc=datetime.now(timezone.utc).isoformat())
            if mode=='deliver_current_held':request['current_held_takeover_context']=resume[2]
        attempt.mkdir()
        if mode in ('resume_transfer','deliver_current_held'):
            source=Path(resume[1]['original_result_file']).parent
            preserved=attempt/'resume_origin';preserved.mkdir()
            copies={}
            for item in source.iterdir():
                if item.is_file():
                    target=preserved/item.name;target.write_bytes(item.read_bytes())
                    copies[item.name]=sha(target)
            old_archive=source.parent.with_suffix('.zip')
            if old_archive.is_file():
                (preserved/'original_parent.zip').write_bytes(old_archive.read_bytes())
                copies['original_parent.zip']=sha(preserved/'original_parent.zip')
            dump(preserved/'preserved_files_sha256.json',copies)
            if mode=='deliver_current_held':
                rejected=Path(resume[2]['rejected_result_file']).parent
                retained=attempt/'rejected_resume';retained.mkdir()
                hashes={}
                for item in rejected.iterdir():
                    if item.is_file():
                        target=retained/item.name;target.write_bytes(item.read_bytes())
                        hashes[item.name]=sha(target)
                dump(retained/'preserved_files_sha256.json',hashes)
        request_path=attempt/'request.json';output=attempt/'worker_result.json';dump(request_path,request)
        began=time.monotonic()
        row=dict(mode=mode,wrist_profile=profile,folder=attempt.name,request_sha256=sha(request_path))
        report['attempts'].append(row);report['state']='MOVING';save()
        code=child([sys.executable,str(ROOT/'deployment/visual_pick_sort_v21_worker.py'),
            '--request',str(request_path),'--output',str(output),'--manifest',str(MANIFEST)],attempt/'abort.json')
        value=json.loads(output.read_bytes()) if output.exists() else {}
        row.update(exit_code=code,outcome=value.get('outcome'),phase=value.get('phase'),
            result_sha256=sha(output) if output.exists() else None,elapsed_seconds=time.monotonic()-began)
        okay=(not code and value.get('phase')=='completed' and not value.get('exceptions')
            and not value.get('cleanup_errors') and value.get('SDK_cleanup_finished') is True)
        if mode in ('visual_cycle','deliver_held','resume_transfer','deliver_current_held'):okay=okay and value.get('return_to_scan_automatic') is True
        row['completed']=okay;save()
        if not okay:
            report['last_action_reason']='; '.join(str(e.get('error')) for e in value.get('exceptions',[])) or str(value.get('outcome'))
            print('动作已暂停，保留主界面与本轮记录；不自动重抓、松手或回臂。',flush=True)
            for error in value.get('exceptions',[]):print(str(error.get('error')),flush=True)
        return okay

    def camera(define=False,observe_only=False,binding=False):
        job=active_batch/('vision_%03d'%(len(report['captures'])+1));job.mkdir()
        response=job/'response.json';ledger=active_batch/'ledger.json'
        command=[sys.executable,str(ROOT/'deployment/visual_pick_sort_v20_vision.py'),
            '--role','target','--position-session',str(ROOT/'calibration/position_rgbd_20261020_001259_216986'),
            '--anchor',str(ROOT/'deployment/visual_reference_201634/task_anchor.json'),
            '--confirm-fixed-camera','--confirm-measured-shell-only','--confirm-unchanged-setup',
            '--confirm-upright-shell-on-this-board','--finish-after-first-reference',
            '--workspace-config',str(workspace),'--scan-response',str(response),
            '--stop-file',str(active_batch/'stop.json'),'--inference-python','/usr/bin/python3','--device','auto']
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
            if outcome=='workspace_no_black_observed':
                batch['state']='WORKSPACE_OBSERVED_EMPTY';report['state']='DONE'
                print('工作区连续观察未检出黑壳，本批结束。',flush=True);break
            if outcome=='workspace_observed' and batch['command_cycles_completed']>=limit:
                batch.update(state='LIMIT_COMPLETED',remaining_black_detections=result['workspace_black_count'])
                report['state']='DONE';break
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
                measurement_bbox_px=point['measurement_bbox_px'][:],top_center_world_m=point['top_center_world_m'][:]))
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
                print('已保存的近处绑定未通过核对；仍可h处理当前持物，b重新绑定，未生成替代坐标。',flush=True)
        save()
        print('v21-fix2：c因原端点与当前姿态不同而暂停后，当前仍稳夹同一空壳，输入t从本次新鲜健康姿态投放；不重抓、不追加抬升。',flush=True)
        print('t授权当前持物起点至共同高位、原投放路线、张开和回观察位；整段扫掠/落物区清空，预计余量≥20mm，独占控制并守急停。原绑定/分区沿用。',flush=True)
        print('c/h仅限同一持物且报错/成功后未移臂、松手或重启，整段含物路线清空；c续本轮已提起点，h只认原140429点。b前手须空。',flush=True)
        print('运行中q回车或Ctrl+C请求停止；软件停止不等于硬件急停。',flush=True)
        while True:
            try:
                choice=input('t当前持物投放 | c原端点续投 | h原录制持物 | b绑定/分区 | a AUTO | m单块 | w工作区 | r重置 | q退出：').strip().lower()
                if choice=='q':break
                if choice=='r':
                    stop_requested=False;report['state']='IDLE';save()
                    print('仅重置软件任务，未移动或清设备保护。',flush=True);continue
                if choice in ('h','b','c','t'):
                    if report['state'] in ('PAUSED','STOPPED'):
                        print('任务仍暂停；r仅重置软件任务。具体原因：'+str(report.get('last_action_reason',report['state'])),flush=True);continue
                    if choice=='b':bind_near()
                    elif choice=='t':
                        from visual_transfer_resume_v21 import load_current_held_takeover
                        original,proof,context=load_current_held_takeover(ROOT)
                        print('已收到t：以本次新鲜稳定回读建立持物起点；旧端点和拒绝记录只作来源，不作起点。',flush=True)
                        stop_requested=False
                        report['state']='IDLE' if motion('deliver_current_held',resume=(original,proof,context)) else 'PAUSED';save()
                    elif choice=='c':
                        from visual_transfer_resume_v21 import load_completed_lift
                        original,proof=load_completed_lift(ROOT)
                        print('续接已完成提起的原记录：'+proof['original_result_file'],flush=True)
                        stop_requested=False
                        report['state']='IDLE' if motion('resume_transfer',resume=(original,proof)) else 'PAUSED';save()
                    else:
                        stop_requested=False
                        report['state']='IDLE' if motion('deliver_held') else 'PAUSED';save()
                    continue
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
