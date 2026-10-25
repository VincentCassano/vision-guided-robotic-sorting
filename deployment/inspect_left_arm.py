"""One-shot read-only SDK snapshot. No movement or configuration writes."""
import argparse
import ctypes
import hashlib
import importlib
import json
import types
from pathlib import Path
import subprocess
import sys
from datetime import datetime, timezone

ROOT=Path(__file__).resolve().parents[1]
QUERIES=('Get_Current_Work_Frame','Get_Current_Tool_Frame','Get_Current_Arm_State',
         'Get_Joint_Speed','Get_Joint_Acc','Get_Joint_Min_Pos','Get_Joint_Max_Pos')


def encode(value):
    if isinstance(value,ctypes.Structure):
        return {name:encode(getattr(value,name)) for name,*_ in value._fields_}
    if isinstance(value,bytes): return value.decode('utf-8',errors='replace')
    if isinstance(value,(tuple,list,ctypes.Array)): return [encode(v) for v in value]
    return value


def snapshot(robot):
    result={}
    for name in QUERIES:
        try:
            answer=getattr(robot,name)()
            if not isinstance(answer,(tuple,list)) or not answer:
                raise ValueError('Unexpected SDK response')
            result[name]={'status':encode(answer[0]),'ok':answer[0]==0,
                          'data':encode(answer[1:]) if answer[0]==0 else None}
        except Exception as exc:
            result[name]={'ok':False,'error':str(exc)}
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sdk-dir',type=Path,help='Existing robotic_arm_package directory')
    parser.add_argument('--ip',default='169.254.128.18')
    args=parser.parse_args()
    (ROOT/'docs').mkdir(exist_ok=True)
    with (ROOT/'docs/执行命令日志.txt').open('a',encoding='utf-8') as f:
        f.write(subprocess.list2cmdline([sys.executable,*sys.argv])+'\n')
    if args.sdk_dir:
        sdk_dir=args.sdk_dir.resolve()
    else:
        with (ROOT/'docs/执行命令日志.txt').open('a',encoding='utf-8') as f: f.write('rospack find grasp_demo\n')
        package=subprocess.check_output(['rospack','find','grasp_demo'],text=True,timeout=10).strip()
        sdk_dir=Path(package)/'scripts/robotic_arm_package'
    expected=json.loads((ROOT/'deployment/arm_sdk_reviewed_hashes.json').read_text())
    for name,digest in expected.items():
        if hashlib.sha256((sdk_dir/name).read_bytes()).hexdigest()!=digest:
            raise RuntimeError('SDK source differs from reviewed version: '+str(sdk_dir/name)+
                               '; no connection opened. Send this file for review.')
    package=types.ModuleType('battery_readonly_sdk')
    package.__path__=[str(sdk_dir)]
    sys.modules[package.__name__]=package
    # Import only the reviewed SDK; never import demonstration startup modules.
    sdk=importlib.import_module('battery_readonly_sdk.robotic_arm')
    robot=None
    try:
        robot=sdk.Arm(sdk.RM65,args.ip)
        if robot.Arm_Socket_State()!=0: raise RuntimeError('Left arm connection failed')
        readings=snapshot(robot)
        record=dict(schema_version=1,read_only=True,execution_ready=False,
                    timestamp_utc=datetime.now(timezone.utc).isoformat(),ip=args.ip,
                    sdk_directory=str(sdk_dir),sdk_source_sha256=expected,
                    pose_units={'position':'m','orientation':'rad'},joint_angle_unit='degree',
                    note='Sequential readings, not an atomic calibration sample. No camera transform inferred.',
                    queries=readings)
        folder=ROOT/'captures'/('left_arm_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        folder.mkdir(parents=True)
        path=folder/'left_arm_frames.json'
        path.write_text(json.dumps(record,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
        print(json.dumps(record,ensure_ascii=False,indent=2,allow_nan=False))
        print('Saved:',path)
        if not all(q['ok'] for q in readings.values()):
            raise RuntimeError('Some queries failed; report saved. Do not use incomplete results for calibration.')
    finally:
        if robot is not None:
            robot.Arm_Socket_Close()
            robot.RM_API_UnInit()


if __name__=='__main__':
    try: main()
    except Exception as exc:
        print('ERROR:',exc,file=sys.stderr)
        sys.exit(1)
