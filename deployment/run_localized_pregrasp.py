#!/usr/bin/env python3
"""Combined World target / prepared-hand / taught noncontact endpoints / plan.

Collect mode permits one known empty-hand prepare at the accepted hover ONLY.
Arm positioning is by the onsite operator's existing teaching controls, never
by this program. Plan mode is offline. No automatic descent/grip/lift exists.
"""
import argparse
import copy
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/battery_grasp/src'))
from battery_grasp.localized_pregrasp import select_surface, derive_reference, build_localized_plan

COMPLETED_NAME = 'supported_contact_release_184936.json'
COMPLETED_SHA = '58c5bcc1e8fd8fc7a9f14b6b61dae9f35fd01db7aa4bcd45e5d7771ecf1be0a1'
CALIBRATION_SHA = '70809f5b27269949f42977fbec9923c8d384f728d8eef1653317edeb320bc764'


def input_event(report,save,prompt,reader=input):
    started = time.monotonic()
    answer = reader(prompt).strip()
    report.setdefault('operator_inputs',[]).append(dict(prompt=prompt,value=answer,
        wait_seconds=time.monotonic()-started,monitored=False))
    save()
    return answer


def capture_vision(args,folder,report,save):
    """Reuse the installed, accepted RGB-D/YOLO preview; no second vision chain."""
    before = {p.parent for p in (ROOT/'captures').glob('*/detections.json')}
    command = ['/bin/bash',str(ROOT/'start_robot_d435.sh'),'--device',args.device,
        '--depth','--world-calibration',str(args.world_calibration),'--confirm-fixed-camera']
    report['vision_process'] = dict(command=command,started_utc=datetime.now(timezone.utc).isoformat())
    save()
    if getattr(args,'follow_vision_report',None) is not None:
        report['vision_capture_role']='after_complete_groove_relocation_only'
        report['before_move_reference_source']='successful_reference_report_not_a_new_capture'
        save()
        print('本次只保存【挪动后】的一张图；移前基准已经从成功记录读取，不需要再采移前图。',flush=True)
        print('若空壳已经搬到相邻完整槽并站稳：保持现在位置，不要再搬一次。若还没搬：先搬好、完全落底。',flush=True)
        print('确认相机实时画面显示的是新槽位后，才按S保存一次，再按Q退出；保存后不再移动空壳。',flush=True)
        print('不要先按S再搬。头部、底座、升降、工作板和机械臂保持不动，现场守急停。',flush=True)
    else:
        print('摄像头窗口：只保留目标空壳；稳定后按S保存一次，再按Q退出。不要移动机器人头部、底座或升降机构。',flush=True)
    process = None
    code = None
    command_log = ROOT/'docs/执行命令日志.txt'
    log_offset = command_log.stat().st_size if command_log.exists() else 0
    env = os.environ.copy()
    env['LOCALIZED_VISION_DIAGNOSTICS'] = str((folder/'vision_runtime.json').resolve())
    try:
        with (folder/'vision_process.log').open('w',encoding='utf-8') as out:
            process = subprocess.Popen(command,cwd=str(ROOT),env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
            for line in process.stdout:
                out.write(line);out.flush();print(line,end='',flush=True)
            code = process.wait()
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try: process.wait(timeout=3)
            except subprocess.TimeoutExpired: process.kill();process.wait()
        if process is not None:
            code = process.returncode
            if process.stdout is not None: process.stdout.close()
        if command_log.is_file():
            with command_log.open('rb') as source:
                source.seek(log_offset)
                (folder/'vision_commands.log').write_bytes(source.read())
        report['vision_process'].update(return_code=code,finished_utc=datetime.now(timezone.utc).isoformat())
        save()
    if code != 0: raise RuntimeError('Vision process failed; raw command/result retained')
    new = sorted({p.parent for p in (ROOT/'captures').glob('*/detections.json')}-before)
    if len(new)!=1: raise RuntimeError('Save exactly one new World capture in this invocation; found '+str(len(new)))
    source = new[0]
    raw = (source/'detections.json').read_bytes()
    document = json.loads(raw)
    destination = folder/'vision'; destination.mkdir()
    report['vision_source'] = dict(directory=str(source),detections_sha256=hashlib.sha256(raw).hexdigest(),
        captured_in_this_invocation=True,point_kind='visible_surface_not_tcp_target',
        calibration_absolute_accuracy_certified=False)
    save()
    for name in ('detections.json','original.png','detections.png','aligned_depth_u16.png'):
        if not (source/name).is_file(): raise RuntimeError('Incomplete RGB-D evidence: '+name)
        (destination/name).write_bytes((source/name).read_bytes())
    for i,row in enumerate(document.get('detections',[])):
        print(i,row.get('class_name'),'World(mm)=',row.get('position_world_mm'),
            'stable=',row.get('visual_confirmed'),'depth=',row.get('depth_valid'),flush=True)
    index = int(input_event(report,save,'输入目标空壳前面的序号（不是track_id）：'))
    target = select_surface(document,index)
    if target['world_transform_sha256'] != CALIBRATION_SHA: raise RuntimeError('Vision/calibration SHA changed')
    report['target'] = target
    save()
    return target


def load_provenance():
    import run_supported_battery_contact as prior
    def pin(name,digest):
        raw = (Path(__file__).parent/name).read_bytes()
        if hashlib.sha256(raw).hexdigest()!=digest: raise ValueError('Pinned source mismatch: '+name)
        return json.loads(raw)
    done = pin(COMPLETED_NAME,COMPLETED_SHA)
    references = pin('grasp_references_accepted_20261009.json',prior.accepted.baseline.REFERENCE_SHA)
    profile = pin('battery_hand_verified_profile.json',references['profile_sha256'])
    profile['sha256'] = references['profile_sha256']
    if (done.get('outcome')!='completed_explicit_supported_open' or done.get('exceptions')
            or done.get('cleanup_errors') or done.get('arm_command_attempted') is not False
            or done.get('configuration')!=profile['observed_configuration']
            or done.get('final_hand',{}).get('targets')!=profile['poses']['open']
            or done['final_hand'].get('force_targets')!=[0]*5):
        raise ValueError('Require completed184936 open/zero-force evidence')
    safe = prior.accepted.baseline.accepted_reference(references,profile)
    previous = pin(prior.accepted.measured.LATEST_NAME,prior.accepted.measured.LATEST_SHA)
    observation = prior.accepted.measured.accepted_latest_reference(previous,safe,profile)
    hover = prior.accepted.measured.accepted_pick_reference(references,profile,observation)
    completed = pin(prior.accepted.LATEST_NAME,prior.accepted.LATEST_SHA)
    prior.accepted.accepted_completion(completed,profile,hover)
    latest = pin(prior.LATEST_NAME,prior.LATEST_SHA)
    prior.accepted_continuous_completion(latest,profile,hover)
    return prior,done,profile,observation,hover,latest


def close_capture_resources(sequence,robot,report):
    # A hand close failure must not skip SDK socket close/API uninitialization.
    if sequence is not None:
        try: sequence.close_hand()
        except (Exception,KeyboardInterrupt) as exc:
            report['cleanup_errors'].append(dict(method='capture_hand.close',error=repr(exc)))
    if robot is not None:
        for method in ('Arm_Socket_Close','RM_API_UnInit'):
            try: getattr(robot,method)()
            except (Exception,KeyboardInterrupt) as exc:
                report['cleanup_errors'].append(dict(method=method,error=repr(exc)))


def collect(args,folder,report,save):
    prior,done,profile,observation,hover,latest = load_provenance()
    from continuous_hand_cycle import ContinuousHand
    from supported_contact_transport import SupportedContactTransport,PhaseAuthority
    from run_school_battery_hand import ADAPTED_PLAN
    from capture_installed_hand_geometry import _vector
    import socket
    import fcntl

    class PrepareOnlyTransport(SupportedContactTransport):
        def set_phase(self,phase,authority):
            if phase != 'prepare_empty':
                self.latch_fault('Only single empty prepare permitted')
                raise RuntimeError('No contact, force-zero, reset or release phase in this entry')
            return super().set_phase(phase,authority)

    class CaptureSequence(prior.SupportedSequence):
        def fresh_taught_arm(self):
            # Entry identity/real-mode checked once; endpoint frames/ordinary and
            # drive limits/protection/reserve re-read after each human wait.
            arm = self.full_arm()
            if arm['frame_signatures']!=hover['frame_signatures'] or arm['limits']!=hover['limits']:
                raise RuntimeError('Taught frame/limits changed')
            self.safety_reads()
            if any(min(q-a,b-q)<3 for q,a,b in zip(arm['joints'],self.drive_low,self.drive_high)):
                raise RuntimeError('Taught endpoint lacks3 degrees drive reserve')
            self.settled(arm['joints'])
            after = self.full_arm()
            prior.accepted.baseline.check_resumed_reference(after,arm)
            self.guard_target,self.guard_pose = after['joints'][:],after['pose'][:]
            return after

        def taught_point(self,label,minimum):
            if self.io is not None: raise RuntimeError('Hand connection must be closed before teaching')
            prompt = ('用现有示教控制将空手移到电池上方的悬停点；保持当前腕姿。' if label=='upper'
                else '用现有示教控制仅沿World -Z降低到非接触预抓点，X/Y和腕姿保持不变。')
            print(prompt+' 不用人手托电池，不得接触电池/桌面；任何接触立即现场硬件停止。',flush=True)
            answer = input_event(report,save,'实际停稳、退出示教后，输入整手到电池/桌面/障碍的最小实测间隙mm（须≥%s，q退出）：'%minimum)
            if answer.lower()=='q': raise EOFError('Operator cancelled; pose retained, no return')
            gap = float(answer)
            import math
            if not math.isfinite(gap) or gap<minimum: raise ValueError('Insufficient measured whole-hand scene gap')
            before = self.fresh_taught_arm()
            if any(abs(math.atan2(math.sin(a-b),math.cos(a-b)))>.005
                   for a,b in zip(before['pose'][3:],hover['pose'][3:])):
                raise RuntimeError('Taught wrist differs from initial accepted wrist')
            hand = self.checked_hand()
            try:
                state = hand.preflight(expected=ADAPTED_PLAN[0]['values'])
                angles = hand.read(label+'_angle_raw',1165,6)
                forces = hand.read(label+'_force_raw',1175,5)
                after = self.full_arm()
                prior.accepted.baseline.check_resumed_reference(after,before)
                final = hand.preflight(expected=ADAPTED_PLAN[0]['values'])
                row = dict(accepted=True,tool_world_mm=[x*1000 for x in after['pose'][:3]],
                    tool_rpy_rad=after['pose'][3:],joint_degrees=after['joints'],joint_reserve_degrees=after['reserve'],
                    frame_signatures=after['frame_signatures'],ordinary_limits=after['limits'],
                    drive_limits=dict(minimum=self.drive_low,maximum=self.drive_high),
                    clearance_mm=gap,clearance_source='operator_measured_minimum_whole_hand_to_scene_mm',
                    hand_target=ADAPTED_PLAN[0]['values'][:],hand_before=state,hand_after=final,
                    hand_angle_raw=angles,hand_force_raw=forces,hand_force_pid=self.force_pid,
                    measured_by_controller=True,readings_are_sequential=True,
                    arm_trajectory_recorded=False,swept_path_verified=False,
                    timestamp_utc=datetime.now(timezone.utc).isoformat())
                report[label] = row;save()
                return row
            finally: self.close_hand()

    robot = sequence = None
    with (ROOT/'captures/hand_stage.lock').open('a') as lock, \
         (folder/'wire.jsonl').open('w',encoding='utf-8') as wire, \
         (folder/'samples.jsonl').open('w',encoding='utf-8') as samples, \
         (folder/'arm_protection_wire.jsonl').open('w',encoding='utf-8') as arm_wire:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        capture_vision(args,folder,report,save)
        if input_event(report,save,'空手仍在已验收hover，电池不在当前手型扫掠范围，现场守急停。p=一次预抓手型；q退出：').lower()!='p':
            report['outcome']='cancelled_no_writes';save();return
        authority = PhaseAuthority()
        def append(out,row): out.write(json.dumps(row)+'\n');out.flush()
        def factory(seq,guarded):
            if not guarded or seq.io is not None: raise RuntimeError('Fresh single guarded session required')
            seq.io = PrepareOnlyTransport(socket.create_connection(('169.254.128.18',8080),timeout=3),lambda r:append(wire,r),authority)
            return ContinuousHand(seq,seq.io,report,save,samples)
        try:
            robot = prior.connect(args.sdk_dir)
            sequence = CaptureSequence(robot,observation,profile,report,save,samples,factory,
                target_reference=hover,latest_document=latest,authority=authority,
                protection_client=prior.ProtectionClient(lambda r:append(arm_wire,r)))
            report.update(execution_scope='single_empty_prepare_then_manual_noncontact_reference_capture',
                supported_contact_live_verified=False,force_control_allowed=False,
                automatic_descent=False,automatic_grip=False,automatic_lift=False,
                hand_pose_writes_allowed=[[1135,[8000,8000,8000,8000,65535,65535]]],
                arm_motion_allowed=False,hand_target_retained_after_capture=True)
            sequence.force_pid = copy.deepcopy(done['force_pid_on_device'])
            sequence.fresh_arm(expected_role='pick_hover')
            hand = sequence.checked_hand()
            try:
                state = hand.preflight(expected=profile['poses']['open'])
                sequence.permit('prepare_empty')
                hand.write_pose(ADAPTED_PLAN[0],profile['poses']['open'][:])
                report['empty_prepare_feedback_pass']=True;save()
            finally: sequence.close_hand()
            sequence.taught_point('upper',50)
            sequence.taught_point('lower',10)
            report['mapping'] = derive_reference(report['target'],report['upper'],report['lower'])
            report['diagnostic_pass'] = True
            config = json.loads(args.geometry.read_text(encoding='utf-8'))
            report['plan'] = build_localized_plan(report,config)
            report['outcome'] = 'references_captured_offline_plan_ready' if report['plan']['planning_ready'] else 'references_captured_path_blocked'
            save()
        except BaseException:
            if sequence is not None: sequence.fault()
            raise
        finally:
            close_capture_resources(sequence,robot,report)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--collect',action='store_true')
    mode.add_argument('--plan-from',type=Path)
    parser.add_argument('--vision-json',type=Path,help='Offline explicit new World detections.json')
    parser.add_argument('--target-index',type=int)
    parser.add_argument('--confirm-same-battery-orientation',action='store_true')
    parser.add_argument('--confirm-empty-clear-exclusive',action='store_true')
    parser.add_argument('--confirm-onsite-stop',action='store_true')
    parser.add_argument('--confirm-fixed-camera',action='store_true')
    parser.add_argument('--world-calibration',type=Path,default=Path(__file__).with_name('handeye_frozen_candidate.json'))
    parser.add_argument('--geometry',type=Path,default=Path(__file__).with_name('vertical_pregrasp_geometry.json'))
    parser.add_argument('--sdk-dir',type=Path,default=Path('/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package'))
    parser.add_argument('--device',default='cuda:0')
    parser.add_argument('--output-parent',type=Path,default=ROOT/'captures')
    args = parser.parse_args(argv)
    if args.collect and not all((args.confirm_empty_clear_exclusive,args.confirm_onsite_stop,args.confirm_fixed_camera)):
        parser.error('Collection requires empty/exclusive, onsite stop and unchanged camera; no connection')
    if args.collect and args.vision_json: parser.error('Collection requires a new live capture, not an old vision file')
    if args.plan_from and args.vision_json and (args.target_index is None or not args.confirm_same_battery_orientation):
        parser.error('New target requires explicit index and same battery/orientation confirmation')
    if hashlib.sha256(args.world_calibration.read_bytes()).hexdigest()!=CALIBRATION_SHA:
        parser.error('Frozen calibration mismatch; no connection')
    manifest = Path(__file__).with_name('localized-pregrasp-files.sha256')
    entries = []
    if manifest.is_file():
        entries = [line.split('  ',1) for line in manifest.read_text().splitlines()]
        for digest,name in entries:
            if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest:
                parser.error('Installed source differs from package: '+name+'; no connection')
    folder = args.output_parent/('localized_pregrasp_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True,exist_ok=False)
    report = dict(schema_version=1,outcome='starting',diagnostic_pass=False,execution_ready=False,
        hardware_arm_writes=0,automatic_descent=False,automatic_grip=False,automatic_lift=False,
        exceptions=[],cleanup_errors=[],timestamp_utc=datetime.now(timezone.utc).isoformat(),
        geometry_sha256=hashlib.sha256(args.geometry.read_bytes()).hexdigest(),
        geometry_unknowns_are_not_zero=True,physical_stop_verified=False,torque_release_verified=False,
        source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (
            Path(__file__),ROOT/'src/battery_grasp/src/battery_grasp/localized_pregrasp.py',
            ROOT/'src/battery_grasp/src/battery_grasp/pregrasp.py',ROOT/'src/battery_grasp/src/battery_grasp/real_grasp.py')})
    if manifest.is_file():
        report['installed_manifest_sha256'] = hashlib.sha256(manifest.read_bytes()).hexdigest()
        report['installed_file_sha256'] = {name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
            for digest,name in entries}
    def save():
        tmp = folder/'result.tmp'
        tmp.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
        tmp.replace(folder/'result.json')
    save()
    try:
        if args.collect: collect(args,folder,report,save)
        else:
            source = json.loads(args.plan_from.read_text(encoding='utf-8'))
            target = None
            if args.vision_json:
                target = select_surface(json.loads(args.vision_json.read_text(encoding='utf-8')),args.target_index)
                target['same_inert_battery_and_orientation_confirmed'] = True
            report['plan'] = build_localized_plan(source,json.loads(args.geometry.read_text(encoding='utf-8')),target)
            report['input_sha256'] = {'reference':hashlib.sha256(args.plan_from.read_bytes()).hexdigest()}
            if args.vision_json: report['input_sha256']['vision']=hashlib.sha256(args.vision_json.read_bytes()).hexdigest()
            report['outcome']='offline_plan_ready' if report['plan']['planning_ready'] else 'offline_path_blocked'
    except (Exception,KeyboardInterrupt) as exc:
        report['outcome']='aborted_no_further_writes'
        report['exceptions'].append(dict(error=repr(exc),traceback=traceback.format_exc()))
        print('ABORT:',exc,'接触或继续运动立即现场硬件停止；无自动张开或回臂。',flush=True)
    finally:
        if report['cleanup_errors']:
            report['diagnostic_pass']=False;report['outcome']='failed_cleanup'
        save()
        with zipfile.ZipFile(folder.with_suffix('.zip'),'x',zipfile.ZIP_DEFLATED) as archive:
            for item in sorted(folder.rglob('*')):
                if item.is_file(): archive.write(item,str(Path(folder.name)/item.relative_to(folder)))
        print('SAVED:',folder.with_suffix('.zip'),'OUTCOME:',report['outcome'],'EXECUTION_READY: False',flush=True)
        for blocker in report.get('plan',{}).get('blockers',[]): print('BLOCK:',blocker,flush=True)
    return 1 if report['exceptions'] or report['cleanup_errors'] else 0


if __name__=='__main__': raise SystemExit(main())
