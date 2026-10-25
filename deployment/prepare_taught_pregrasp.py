#!/usr/bin/env python3
"""Compile both approach candidates and run one local native calculation; no actuation."""
import argparse
import csv
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
import traceback
import zipfile

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/battery_grasp/src'))
from battery_grasp.taught_path import compile_candidate
from battery_grasp.taught_dh_model import reviewed_controller_model


def verify_installation():
    path=ROOT/'deployment/taught-path-files.sha256';raw=path.read_bytes();rows={}
    for line in raw.decode('ascii').splitlines():
        digest,name=line.split('  ',1)
        target=(ROOT/name).resolve()
        if ROOT.resolve() not in target.parents or name in rows or len(digest)!=64:
            raise ValueError('Invalid installation manifest')
        if hashlib.sha256(target.read_bytes()).hexdigest()!=digest:raise ValueError('Installed source differs: '+name)
        rows[name]=digest
    if not rows:raise ValueError('Empty installation manifest')
    return rows,hashlib.sha256(raw).hexdigest()


def export_csv(path,rows):
    with path.open('w',encoding='utf-8',newline='') as f:
        writer=csv.writer(f);writer.writerow(['index','segment','fraction','J1_deg','J2_deg','J3_deg','J4_deg','J5_deg','J6_deg',
            'X_mm','Y_mm','Z_mm','Rx_rad','Ry_rad','Rz_rad','ordinary_reserve_deg','drive_reserve_deg','executable'])
        for i,row in enumerate(rows):
            pose=row.get('calculated',row);xyz=pose.get('tool_world_mm') or [None]*3;rpy=pose.get('tool_rpy_rad') or [None]*3
            writer.writerow([i,row.get('segment'),row.get('fraction'),*row['joint_degrees'],*xyz,*rpy,
                row['ordinary_minimum_reserve_degrees'],row['drive_minimum_reserve_degrees'],False])


def run_worker(folder,sdk_dir,runner=subprocess.run):
    command=[sys.executable,str(ROOT/'deployment/taught_path_native_worker.py'),'--output',str(folder),'--sdk-dir',str(sdk_dir)]
    status=dict(command=command,stdin='DEVNULL',close_fds=True,timeout_seconds=45,sdk_in_parent=False)
    with (folder/'native_worker.log').open('wb') as log:
        try:
            p=runner(command,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,
                close_fds=True,timeout=45,check=False)
            status['return_code']=p.returncode
        except subprocess.TimeoutExpired as exc:status.update(error=repr(exc),timed_out=True)
    (folder/'process_status.json').write_text(json.dumps(status,indent=2)+'\n',encoding='utf-8')
    if status.get('return_code')!=0:
        detail=''
        path=folder/'native_result.json'
        if path.is_file():
            try:
                errors=json.loads(path.read_text(encoding='utf-8')).get('exceptions',[])
                if errors:detail=': '+str(errors[0].get('error','unknown'))
            except (OSError,ValueError,AttributeError,IndexError):pass
        raise RuntimeError('Numerical worker failed'+detail+'; complete stdout/process data retained')
    result=json.loads((folder/'native_result.json').read_text(encoding='utf-8'))
    if (result.get('worker_completed') is not True or result.get('execution_ready') is not False
            or result.get('hardware_writes')!=0 or result.get('robot_instance_constructed') is not False
            or result.get('network_connection_requested') is not False):
        raise ValueError('Numerical worker result or scope invalid')
    return result


def collect_requested_model_parameters(args,folder):
    if not args.read_model_parameters:return None
    if not args.confirm_exclusive_read:raise ValueError('Explicit exclusive read confirmation required')
    if platform.system()!='Linux' or platform.machine()!='aarch64':
        raise ValueError('Model reads only permitted on onsite Linux aarch64')
    import fcntl
    from taught_model_readonly import collect_model_parameters
    (ROOT/'captures').mkdir(exist_ok=True)
    with (ROOT/'captures/hand_stage.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        return collect_model_parameters(folder)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compile-only',action='store_true',help='Export audited joint candidate without loading any SDK')
    parser.add_argument('--sdk-dir',type=Path,default=Path('/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package'))
    parser.add_argument('--output-parent',type=Path,default=ROOT/'captures')
    parser.add_argument('--read-model-parameters',action='store_true',help='Explicit controller identity/DH GETs only; no configuration or motion')
    parser.add_argument('--confirm-exclusive-read',action='store_true',help='Other teaching/control scripts closed for the short read-only model query')
    args=parser.parse_args(argv)
    if args.read_model_parameters:
        if args.compile_only or not args.confirm_exclusive_read:
            parser.error('Model reads require --confirm-exclusive-read and cannot be combined with --compile-only; no connection')
        if platform.system()!='Linux' or platform.machine()!='aarch64':
            parser.error('Model reads only permitted on onsite Linux aarch64; no connection')
    try:
        files,manifest_sha=verify_installation()
        raw=(ROOT/'deployment/battery_teaching_212012.json').read_bytes();candidate=compile_candidate(raw)
        reviewed_model=reviewed_controller_model((ROOT/'deployment/taught_model_parameters_192049.json').read_bytes())
    except (OSError,ValueError) as exc:parser.error(str(exc)+'; no SDK loaded')
    folder=args.output_parent/('taught_pregrasp_plan_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True,exist_ok=False)
    report=dict(schema_version=1,outcome='starting',execution_ready=False,hardware_writes=0,
        timestamp_utc=datetime.now(timezone.utc).isoformat(),robot_motion_allowed=False,hand_motion_allowed=False,
        installed_file_sha256=files,installed_manifest_sha256=manifest_sha,exceptions=[],candidate=candidate,
        controller_DH_source=reviewed_model,
        model_read_requested=args.read_model_parameters,network_connection_requested=False,controller_configuration_writes=0)
    def save():(folder/'result.json').write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8')
    (folder/'teaching_source.json').write_bytes(raw);save()
    print('复用已采三处示教位。计算关节候选和固定腕姿下降候选；不移动臂或手。',flush=True)
    if args.read_model_parameters:
        print('明确只读模式：关闭其他示教/控制脚本；只获取本机身份与DH，保留原始报文，不修改参数。',flush=True)
    else:print('纯数值模式：不请求机器人连接。',flush=True)
    print('SDK只在独立数值计算子进程加载，无需摆位、按n、重拍或重新上电。',flush=True)
    print('14mm仅为手部最低外表面到桌面的间隙；手到电池/其他障碍的间隙未知。',flush=True)
    print('复用192049完整DH实读，按D1参数差异换算本地数值模型，不修改设备或重复采集。',flush=True)
    try:
        export_csv(folder/'joint_candidate.csv',candidate['joint_samples'])
        if args.read_model_parameters:
            model=collect_requested_model_parameters(args,folder);report['model_parameters']=model
            report['network_connection_requested']=model['network_connection_requested'];save()
            print('MODEL PARAMETERS:',model['outcome'],'DH valid=',model['dh_valid'],flush=True)
            if model['dh_valid'] and [model['dh_response']['joint_'+str(i)] for i in range(1,7)]!=reviewed_model['rows']:
                raise ValueError('Fresh controller DH changed; pinned numerical model not used')
        if args.compile_only:report['outcome']='compiled_candidate_native_not_run'
        else:
            assessment=run_worker(folder,args.sdk_dir.resolve());report['native_assessment']=assessment
            if assessment.get('algorithm_model_source')!=reviewed_model:
                raise ValueError('Numerical worker used a different controller DH provenance')
            if not assessment['native_model_agreement']:
                report['outcome']='native_model_mismatch_no_route_computation'
            elif assessment['joint_candidate_calculated'] and assessment['vertical_candidate_calculated']:
                report['outcome']='both_numerical_candidates_calculated_geometry_pending'
            else:report['outcome']='partial_numerical_candidates_geometry_pending'
            if assessment['joint_path']:export_csv(folder/'joint_candidate.csv',assessment['joint_path'])
            if assessment['vertical_path']:export_csv(folder/'vertical_candidate.csv',assessment['vertical_path'])
            for witness in assessment['witnesses']:
                print('MODEL:',witness['role'],'error_mm=',witness.get('translation_error_mm'),
                    'rotation_deg=',witness.get('rotation_error_degrees'),'match=',witness['pass_agreement'],flush=True)
            if not assessment['native_model_agreement']:
                print('模型与已采端点不一致；完整差异已保存。未拟合补偿、未修改DH、未继续路径逆解。',flush=True)
    except (Exception,KeyboardInterrupt) as exc:
        report['outcome']=('numerical_worker_failed_readonly_model_queries' if report['network_connection_requested']
            else 'numerical_worker_failed_no_hardware_connection')
        report['exceptions'].append(dict(error=repr(exc),traceback=traceback.format_exc()))
        print('ERROR:',exc,flush=True)
    finally:
        save()
        with zipfile.ZipFile(folder.with_suffix('.zip'),'x',zipfile.ZIP_DEFLATED) as z:
            for p in sorted(folder.rglob('*')):
                if p.is_file():z.write(p,str(Path(folder.name)/p.relative_to(folder)))
        print('SAVED:',folder.with_suffix('.zip'),'OUTCOME:',report['outcome'],'EXECUTION_READY: False',flush=True)
        print('尚缺整手/小拇指/臂身线缆扫掠、桌面与障碍边界、实际控制器路径验收；数值结果不开放运动。',flush=True)
    return 1 if report['exceptions'] else 0


if __name__=='__main__':raise SystemExit(main())
