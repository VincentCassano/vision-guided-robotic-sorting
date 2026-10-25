#!/usr/bin/env python3
"""Read-only freehand teaching continuation. No arm/hand actuator is permitted."""
import argparse
import copy
from datetime import datetime,timezone
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback
import zipfile

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/battery_grasp/src'))
from battery_grasp.manual_teaching import reviewed_resume,reviewed_teaching_resume,make_teaching_draft,endpoint_delta,check_taught_point,PREPARE
from run_localized_pregrasp import load_provenance,close_capture_resources,input_event
from supported_contact_transport import SupportedContactTransport,PhaseAuthority
from school_battery_transport import MULTI,SINGLE
from continuous_hand_cycle import ContinuousHand


class ReadOnlyTeachingTransport(SupportedContactTransport):
    def set_phase(self,*args,**kwargs):
        self.latch_fault('Freehand teaching is read-only; no phase transition')
        raise RuntimeError('No teaching actuator phase')
    def enable_motion(self):
        self.latch_fault('Freehand teaching cannot authorize motion')
        raise RuntimeError('No teaching motion permission')
    def write_group(self,*args,**kwargs):
        self.latch_fault('Freehand teaching rejects every register write')
        raise RuntimeError('No teaching register writes')
    def exchange(self,message):
        if message.get('command') not in (MULTI,SINGLE):
            self.latch_fault('Freehand teaching rejects every non-read command')
            raise RuntimeError('Only register reads permitted')
        return super().exchange(message)


def sdk_identity(prior,robot):
    metadata=prior.accepted.baseline.collect_sdk_environment(robot)
    if (metadata.get('metadata_complete') is not True
            or metadata.get('loaded_native',{}).get('sha256')!=prior.accepted.baseline.NATIVE_SHA
            or metadata.get('api_version')!='4.3.1' or metadata.get('host_machine')!='aarch64'
            or metadata.get('controller_run_mode_raw',{}).get('data')!=[1]
            or metadata.get('controller_software_info',{}).get('data',[{}])[0].get('product_version')!='RM65-BI'):
        raise RuntimeError('Current SDK/controller identity or real mode changed')
    return metadata


def descriptor_state():
    """Diagnostics only: native lifecycle may affect descriptors in its process."""
    rows=[]
    for fd in (0,1,2):
        try:
            stat=os.fstat(fd)
            rows.append(dict(fd=fd,open=True,device=stat.st_dev,inode=stat.st_ino,mode=stat.st_mode))
        except OSError as exc: rows.append(dict(fd=fd,open=False,error=repr(exc)))
    return rows


def cleanup_with_descriptor_probes(sequence,robot,report,save):
    probes=report.setdefault('sdk_cleanup_descriptor_probes',[])
    probes.append(dict(stage='before_cleanup',descriptors=descriptor_state()));save()
    if sequence is not None:
        try:sequence.close_hand()
        except (Exception,KeyboardInterrupt) as exc:
            report['cleanup_errors'].append(dict(method='capture_hand.close',error=repr(exc)))
        probes.append(dict(stage='after_hand_close',descriptors=descriptor_state()));save()
    if robot is not None:
        for method in ('Arm_Socket_Close','RM_API_UnInit'):
            try:getattr(robot,method)()
            except (Exception,KeyboardInterrupt) as exc:
                report['cleanup_errors'].append(dict(method=method,error=repr(exc)))
            probes.append(dict(stage='after_'+method,descriptors=descriptor_state()));save()


def capture_one(args,role,gap,provenance,source,report,save,samples,wire,arm_wire):
    import socket
    if role not in ('approach_waypoint','operator_pregrasp') or type(gap) not in (int,float) or not math.isfinite(gap) or gap<10:
        raise ValueError('Valid endpoint role and measured clearance >=10mm required before connection')
    prior,done,profile,observation,hover,latest=provenance
    # A new READ-ONLY connection does not restore old write authority.
    authority=PhaseAuthority();authority._terminal_latched=True
    robot=sequence=None
    def append(out,row): out.write(json.dumps(row)+'\n');out.flush()
    def factory(seq,guarded):
        if not guarded or seq.io is not None: raise RuntimeError('Require one fresh read-only hand connection')
        seq.io=ReadOnlyTeachingTransport(socket.create_connection(('169.254.128.18',8080),timeout=3),
            lambda row:append(wire,row),authority)
        return ContinuousHand(seq,seq.io,report,save,samples)
    class TeachingSequence(prior.SupportedSequence):
        def permit(self,*args,**kwargs): raise RuntimeError('Teaching never restores actuator authority')
        def fault(self):
            authority._terminal_latched=True
            if self.io is not None: self.io.disable_motion()
            report.update(fault_latched=True,next_action='Hardware stop for unexpected contact/continuing motion; '
                'no automatic open, release, return or retry')
            save()
    try:
        robot=prior.connect(args.sdk_dir)
        metadata=sdk_identity(prior,robot)
        sequence=TeachingSequence(robot,observation,profile,report,save,samples,factory,
            target_reference=hover,latest_document=latest,authority=authority,
            protection_client=prior.ProtectionClient(lambda row:append(arm_wire,row)))
        # Parent supplies read helpers, never its historical force/route intent.
        report.update(execution_scope='read_only_manual_teaching_endpoints',arm_motion_allowed=False,
            hand_pose_writes_allowed=[],force_control_allowed=False,force_targets_mn=[0]*5,
            force_target_source='no force command; fresh device targets must be zero',intended_contact=False,
            old_fault_write_authority_restored=False,sdk_environment=metadata,
            current_role='manually_taught_endpoint',trajectory_planned=False)
        report.pop('route',None)
        sequence.force_pid=copy.deepcopy(source['force_pid_on_device'])
        before=sequence.full_arm()
        if before['frame_signatures']!=hover['frame_signatures'] or before['limits']!=hover['limits']:
            raise RuntimeError('World/Arm_Tip configuration or ordinary limits changed')
        sequence.safety_reads()
        if any(min(q-a,b-q)<3 for q,a,b in zip(before['joints'],sequence.drive_low,sequence.drive_high)):
            raise RuntimeError('Current endpoint drive reserve below3 degrees')
        sequence.settled(before['joints'])
        stable=sequence.full_arm()
        prior.accepted.baseline.check_resumed_reference(stable,before)
        sequence.guard_target,sequence.guard_pose=stable['joints'][:],stable['pose'][:]
        report['pending_endpoint']=dict(role=role,tool_world_mm=[x*1000 for x in stable['pose'][:3]],
            tool_rpy_rad=stable['pose'][3:],joint_degrees=stable['joints'],clearance_mm=gap)
        save()  # Keep the measured arm even if a subsequent hand read fails.
        hand=sequence.checked_hand()
        first=hand.preflight(expected=PREPARE)
        angles=hand.read(role+'_angle_raw',1165,6)
        forces=hand.read(role+'_force_raw',1175,5)
        after=sequence.full_arm()
        prior.accepted.baseline.check_resumed_reference(after,stable)
        last=hand.preflight(expected=PREPARE)
        point=dict(data_pass=True,role=role,source='fresh_controller_and_hand_readonly_brackets',
            tool_world_mm=[x*1000 for x in after['pose'][:3]],tool_rpy_rad=after['pose'][3:],
            joint_degrees=after['joints'],joint_reserve_degrees=after['reserve'],
            drive_reserve_degrees=[min(q-a,b-q) for q,a,b in zip(after['joints'],sequence.drive_low,sequence.drive_high)],
            frame_signatures=after['frame_signatures'],ordinary_limits=after['limits'],
            drive_limits=dict(minimum=sequence.drive_low,maximum=sequence.drive_high),
            hand_target=PREPARE[:],hand_before=first,hand_after=last,
            hand_angle_raw=angles,hand_force_raw=forces,hand_force_pid=copy.deepcopy(sequence.force_pid),
            clearance_mm=gap,clearance_source='operator_measured_minimum_whole_hand_to_scene_mm',
            operator_pregrasp_placement_confirmed=role=='operator_pregrasp',
            grasp_geometry_verified=False,manual_path_between_points_recorded=False,
            measured_by_controller=True,readings_are_sequential=True,physical_stop_verified=False,
            timestamp_utc=datetime.now(timezone.utc).isoformat())
        check_taught_point(point)
        previous=report['points'][-1] if report['points'] else report['upper_reference']
        point['delta_from_previous']=endpoint_delta(previous,point)
        report['points'].append(point);report.pop('pending_endpoint',None);save()
        print('已记录',role,'TCP(mm)=',point['tool_world_mm'],'腕姿(°)=',
            [round(math.degrees(x),3) for x in point['tool_rpy_rad']],flush=True)
        print('相对上一点变化：',json.dumps(point['delta_from_previous'],ensure_ascii=False),flush=True)
        return point
    except BaseException:
        if sequence is not None: sequence.fault()
        raise
    finally:
        cleanup_with_descriptor_probes(sequence,robot,report,save)


def read_inputs(report,save,reader=input):
    print('程序只读取。徒手示教允许改变腕姿；每次停稳、退出示教并撤开双手后才采集。',flush=True)
    print('a=接近/中间点；p=真正预抓位（电池在拇指与四指夹持空隙内，各指留间隙，尚未收紧）；q=保存退出。',flush=True)
    print('手在电池顶上只能选a。不得接触电池/桌面；接触或继续运动立即现场硬件停止。',flush=True)
    while True:
        action=input_event(report,save,'摆好并停稳后输入 a/p/q：',reader).lower()
        if action=='q': return None
        if action not in ('a','p'):
            print('请输入a、p或q。',flush=True);continue
        while True:
            text=input_event(report,save,'整手到电池/桌面/障碍的最小间隙mm（须≥10，q退出；可填小数，不加mm）：',reader)
            if text.lower()=='q': return None
            try: gap=float(text)
            except ValueError: gap=float('nan')
            if math.isfinite(gap) and gap>=10: break
            print('间隙必须是≥10的有限数字；尚未连接设备。',flush=True)
        return ('approach_waypoint' if action=='a' else 'operator_pregrasp'),gap


def verified_installation():
    manifest=Path(__file__).with_name('manual-teaching-files.sha256')
    if not manifest.is_file():raise ValueError('Installed manifest missing')
    files={}
    for line in manifest.read_text().splitlines():
        digest,name=line.split('  ',1)
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest:
            raise ValueError('Installed file differs: '+name)
        files[name]=digest
    return files,hashlib.sha256(manifest.read_bytes()).hexdigest()


def isolated_capture(args,folder,role,gap,report,save):
    """The interactive parent never initializes/uninitializes the native SDK."""
    count=len(report['points']);worker=folder/('point_%03d'%(count+1));worker.mkdir(exist_ok=False)
    request=dict(schema_version=1,role=role,clearance_mm=gap,sdk_dir=str(args.sdk_dir),
        resume_report=str((folder/'resume_source.json').resolve()),report=copy.deepcopy(report))
    request_path=worker/'request.json'
    request_path.write_text(json.dumps(request,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8')
    command=[sys.executable,str(ROOT/'deployment/battery_teaching_worker.py'),'--request',str(request_path.resolve())]
    status=dict(command=command,stdin='DEVNULL',close_fds=True,timeout_seconds=90,
        parent_descriptors_before=descriptor_state(),sdk_in_parent=False)
    print('只读采集第%d点，SDK在独立进程运行…'%(count+1),flush=True)
    failure=None
    started=time.monotonic()
    try:
        with (worker/'worker.log').open('wb') as log:
            process=subprocess.run(command,cwd=str(ROOT),stdin=subprocess.DEVNULL,stdout=log,
                stderr=subprocess.STDOUT,close_fds=True,timeout=90)
        status['return_code']=process.returncode
    except (Exception,KeyboardInterrupt) as exc:
        failure=exc;status['error']=repr(exc)
    finally:
        status.update(elapsed_seconds=time.monotonic()-started,parent_descriptors_after=descriptor_state())
        (worker/'process_status.json').write_text(json.dumps(status,indent=2)+'\n',encoding='utf-8')
        report.setdefault('sdk_worker_processes',[]).append(dict(directory=worker.name,**status));save()
    result=worker/'worker_result.json'
    if result.is_file():
        child=json.loads(result.read_text(encoding='utf-8'))
        points=child.get('points',[])
        if (child.get('target')!=report['target'] or child.get('upper_reference')!=report['upper_reference']
                or child.get('execution_ready') is not False or child.get('hardware_arm_writes')!=0
                or child.get('hand_writes_allowed') is not False or points[:count]!=report['points']):
            raise RuntimeError('Worker result provenance/scope differs; no subsequent capture')
        processes=report['sdk_worker_processes'];report.update(child);report['sdk_worker_processes']=processes;save()
    if failure is not None:raise failure
    if status.get('return_code')!=0 or not result.is_file() or report.get('worker_capture_pass') is not True:
        raise RuntimeError('Read-only worker failed; see worker.log/result/raw frames; no retry')
    if report['cleanup_errors'] or len(report['points'])!=count+1:
        raise RuntimeError('Worker cleanup/point count failed; no subsequent capture')
    point=report['points'][-1];check_taught_point(point)
    print('已记录',point['role'],'TCP(mm)=',point['tool_world_mm'],'腕姿(°)=',
        [round(math.degrees(x),3) for x in point['tool_rpy_rad']],flush=True)
    print('相对上一点变化：',json.dumps(point['delta_from_previous'],ensure_ascii=False),flush=True)
    return point


def run_teaching_loop(args,folder,report,save):
        while True:
            choice=read_inputs(report,save)
            if choice is None:
                report['outcome']='saved_partial_teaching' if report['points'] else 'cancelled_no_connections'
                return
            role,gap=choice
            isolated_capture(args,folder,role,gap,report,save)
            report['draft']=make_teaching_draft(report['target'],report['upper_reference'],report['points']);save()
            if role=='operator_pregrasp':
                if report['draft']['draft_ready'] is not True: raise RuntimeError('Recorded point failed draft data validation')
                report['diagnostic_pass']=True;report['outcome']='taught_pregrasp_endpoint_saved_path_unverified';return


def run(args,folder,report,save,provenance,source):
    import fcntl
    # Only this parent owns the cooperation lock. Workers receive no lock FD.
    with (ROOT/'captures/hand_stage.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        run_teaching_loop(args,folder,report,save)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--resume-report',type=Path,required=True)
    parser.add_argument('--confirm-empty-clear-exclusive',action='store_true')
    parser.add_argument('--confirm-onsite-stop',action='store_true')
    parser.add_argument('--confirm-same-scene',action='store_true',help='Same shell/position/orientation and unchanged camera/head/base/lift/scene')
    parser.add_argument('--sdk-dir',type=Path,default=Path('/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package'))
    parser.add_argument('--output-parent',type=Path,default=ROOT/'captures')
    args=parser.parse_args(argv)
    if not all((args.confirm_empty_clear_exclusive,args.confirm_onsite_stop,args.confirm_same_scene)):
        parser.error('Require empty/exclusive, onsite stop and same scene; no connection')
    try:
        files,manifest_sha=verified_installation()
        provenance=load_provenance()
        raw=args.resume_report.read_bytes()
        original_raw=Path(__file__).with_name('localized_pregrasp_193957.json').read_bytes()
        source,target,upper,points=reviewed_teaching_resume(raw,original_raw,provenance[2],provenance[4])
    except (OSError,ValueError) as exc: parser.error(str(exc)+'; no connection')
    folder=args.output_parent/('battery_teaching_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True,exist_ok=False)
    report=dict(schema_version=1,outcome='starting',diagnostic_pass=False,execution_ready=False,
        automatic_descent=False,automatic_grip=False,automatic_lift=False,hardware_arm_writes=0,
        hand_writes_allowed=False,physical_stop_verified=False,torque_release_verified=False,
        old_fault_write_authority_restored=False,exceptions=[],cleanup_errors=[],points=points,
        target=target,upper_reference=upper,resume_source_sha256=hashlib.sha256(raw).hexdigest(),
        source_camera_capture_sha256=source['vision_source']['detections_sha256'],
        vision_reused_without_execution_authority=True,same_scene_operator_confirmed=True,
        accepted_upper_clearance_mm=upper['clearance_mm'],installed_file_sha256=files,
        installed_manifest_sha256=manifest_sha,native_sdk_isolated=True,sdk_in_parent=False,
        reused_approach_points=len(points),sdk_worker_processes=[],
        timestamp_utc=datetime.now(timezone.utc).isoformat())
    def save():
        tmp=folder/'result.tmp';tmp.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
        tmp.replace(folder/'result.json')
    (folder/'resume_source.json').write_bytes(raw);save()
    print('已复用定位与上方点139.7mm；不重新拍照，不重复手型，不自动移动。',flush=True)
    if points:print('已复用成功接近点24.4mm；继续摆下一点，无需重采该点。',flush=True)
    try: run(args,folder,report,save,provenance,source)
    except (Exception,KeyboardInterrupt) as exc:
        report['outcome']='aborted_readonly_no_actuator_writes'
        report['exceptions'].append(dict(error=repr(exc),traceback=traceback.format_exc()))
        print('ABORT:',exc,'现场硬件停止用于接触或继续运动；无自动张开/回臂。',flush=True)
    finally:
        if report['cleanup_errors']: report['diagnostic_pass']=False;report['outcome']='failed_cleanup'
        report['draft']=make_teaching_draft(target,upper,report['points']);save()
        with zipfile.ZipFile(folder.with_suffix('.zip'),'x',zipfile.ZIP_DEFLATED) as z:
            for path in sorted(folder.rglob('*')):
                if path.is_file(): z.write(path,str(Path(folder.name)/path.relative_to(folder)))
        print('SAVED:',folder.with_suffix('.zip'),'OUTCOME:',report['outcome'],'EXECUTION_READY: False',flush=True)
    return 1 if report['exceptions'] or report['cleanup_errors'] else 0


if __name__=='__main__': raise SystemExit(main())
