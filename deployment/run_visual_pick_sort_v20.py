"""Fresh-camera AUTO batches using the source-qualified v19 pickup sequence."""
from datetime import datetime,timezone
import hashlib
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
from visual_pick_sort_v20_source import (MANIFEST,SUCCESS_SHA,verify_delivery,accepted_anchor,target_candidate)
from visual_pick_sort_v13_source import read_point_archive


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    folder=ROOT/'captures'/datetime.now().strftime('visual_pick_sort_v20_%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True)
    workspace=ROOT/'calibration/visual_pick_sort_v20_workspace.json'
    anchor_path=ROOT/'calibration/visual_pick_sort_v14_anchor.json'
    report=dict(version=20,state='IDLE',batches=[],attempts=[],captures=[],exceptions=[],
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

    def motion(mode,point=None,vision_archive=None):
        attempt=folder/('cycle_%03d'%(len(report['attempts'])+1));attempt.mkdir()
        permission=dict(exclusive=True,onsite_stop=True,inert_shell=True,fixed_head_board_mount=True,
            whole_route_and_sweep_clear=True,fresh_actual_open_posture_required=True,
            initial_stationary_group_open_authorized=True,initial_opening_and_inert_shell_drop_zone_clear=True,
            object_absence_verified=False,batch_route_and_scan_return_authorized=True)
        request=dict(mode=mode,success_source_sha256=SUCCESS_SHA,manifest_sha256=sha(MANIFEST),
            scene_permission=permission,timestamp_utc=datetime.now(timezone.utc).isoformat())
        if mode=='visual_cycle':
            permission.update(target_unchanged_since_capture=True,continuous_pick_sort_authorized=True)
            candidate=target_candidate(point,anchor)
            request.update(anchor=anchor,point_record=point,candidate=candidate,
                vision_archive_sha256=sha(vision_archive),return_to_scan_upper_after_drop=True)
        request_path=attempt/'request.json';output=attempt/'worker_result.json';dump(request_path,request)
        began=time.monotonic()
        row=dict(mode=mode,folder=attempt.name,request_sha256=sha(request_path))
        report['attempts'].append(row);report['state']='MOVING';save()
        code=child([sys.executable,str(ROOT/'deployment/visual_pick_sort_v20_worker.py'),
            '--request',str(request_path),'--output',str(output),'--manifest',str(MANIFEST)],attempt/'abort.json')
        value=json.loads(output.read_bytes()) if output.exists() else {}
        row.update(exit_code=code,outcome=value.get('outcome'),phase=value.get('phase'),
            result_sha256=sha(output) if output.exists() else None,elapsed_seconds=time.monotonic()-began)
        okay=(not code and value.get('phase')=='completed' and not value.get('exceptions')
            and not value.get('cleanup_errors') and value.get('SDK_cleanup_finished') is True)
        if mode=='visual_cycle':okay=okay and value.get('return_to_scan_automatic') is True
        row['completed']=okay;save()
        if not okay:
            print('动作已暂停，保留主界面与本轮记录；不自动重抓、松手或回臂。',flush=True)
            for error in value.get('exceptions',[]):print(str(error.get('error')),flush=True)
        return okay

    def camera(define=False,observe_only=False):
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
        else:
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

    def run_batch(limit,redefine=False):
        nonlocal active_batch,stop_requested
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
            batch['completed'].append(dict(id=batch['command_cycles_completed'],
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
        report['anchor_sha256']=sha(anchor_path);save()
        print('v20：a自动连续1～9块；m单块。首次沿工作板依次点四角、S保存，以后自动选目标，无需逐块s/g。',flush=True)
        print('仅同规格45.8×98mm直立断电黑壳；a/m授权本批完整路线、回高位及张开/落物区已清空，独占控制并守急停。',flush=True)
        print('运行中q回车或Ctrl+C请求停止；软件停止不等于硬件急停。',flush=True)
        while True:
            try:
                choice=input('a AUTO | m单块 | w工作区 | r重置任务 | q保存退出：').strip().lower()
                if choice=='q':break
                if choice=='r':
                    stop_requested=False;report['state']='IDLE';save()
                    print('仅重置软件任务，未移动或清设备保护。',flush=True);continue
                if choice in ('a','m','w'):
                    if report['state'] in ('PAUSED','STOPPED'):
                        print('现场处理后r重置任务；未自动重试。',flush=True);continue
                    run_batch(9 if choice=='a' else 1,redefine=choice=='w')
                else:print('输入a自动处理，m只处理一块。',flush=True)
            except KeyboardInterrupt:stop();report['state']='STOPPED';save()
            except EOFError:break
            except Exception as exc:
                report['exceptions'].append(dict(error=repr(exc)));report['state']='PAUSED';save()
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
