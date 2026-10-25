"""Read existing terminal Modbus configuration only; no setup or motion writes."""
import argparse
import hashlib
import importlib
import json
import subprocess
import sys
import types
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BLOCKS = [('identity', 1000, 8), ('status', 1085, 6),
          ('current', 1105, 6), ('position', 1145, 6)]


def connect(sdk_dir):
    expected = json.loads((ROOT/'deployment/arm_sdk_reviewed_hashes.json').read_text())
    for name, digest in expected.items():
        if hashlib.sha256((sdk_dir/name).read_bytes()).hexdigest() != digest:
            raise ValueError('Unreviewed SDK: '+name)
    module = types.ModuleType('battery_hand_readonly_sdk')
    module.__path__ = [str(sdk_dir)]
    sys.modules[module.__name__] = module
    sdk = importlib.import_module(module.__name__+'.robotic_arm')
    robot = sdk.Arm(sdk.RM65, '169.254.128.18')
    if robot.Arm_Socket_State() != 0:
        try: robot.Arm_Socket_Close()
        finally: robot.RM_API_UnInit()
        raise RuntimeError('Left arm connection failed')
    return robot


def collect(robot):
    result = dict(schema_version=1, read_only=True, execution_ready=False,
                  port=1, device=2, blocks=[], communication_response_valid=False,
                  note='Raw bytes only. No verified finger mapping, units, health or grasp verdict.')
    for name, address, count in BLOCKS:
        item = dict(name=name, address=address, count=count)
        result['blocks'].append(item)
        try:
            status, data = robot.Read_Multiple_Holding_Registers(1, address, count, 2)
            item.update(status=status, raw_bytes=data)
            if status != 0:
                raise ValueError('SDK read failed; check power/cable/existing Modbus mode; no automatic setup')
            if len(data) != count * 2 or any(type(v) is not int or not 0 <= v <= 255 for v in data):
                raise ValueError('Unexpected SDK byte layout')
            item['words_big_endian_candidate'] = [(data[i] << 8) | data[i+1] for i in range(0, len(data), 2)]
            if name == 'identity' and not any(data):
                raise ValueError('All-zero identity is not accepted as device identification')
        except Exception as exc:
            item['error'] = str(exc)
            return result
    result['communication_response_valid'] = True
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sdk-dir', type=Path, default=Path('/home/rm/catkin_ws/src/grasp_demo/scripts/robotic_arm_package'))
    parser.add_argument('--confirm-hand-installed', action='store_true')
    args = parser.parse_args()
    if not args.confirm_hand_installed:
        parser.error('Install/connect the left hand using the hardware procedure before reading it')
    (ROOT/'docs').mkdir(exist_ok=True)
    with (ROOT/'docs/执行命令日志.txt').open('a', encoding='utf-8') as log:
        log.write(subprocess.list2cmdline([sys.executable, *sys.argv])+'\n')
    out = ROOT/'captures'/('left_hand_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    out.mkdir(parents=True, exist_ok=False)
    robot = None
    result = dict(schema_version=1, read_only=True, execution_ready=False, communication_response_valid=False)
    try:
        robot = connect(args.sdk_dir)
        result = collect(robot)
    except Exception as exc:
        result['error'] = str(exc)
    finally:
        if robot is not None:
            for method in ('Arm_Socket_Close', 'RM_API_UnInit'):
                try:
                    getattr(robot, method)()
                except Exception as exc:
                    result.setdefault('cleanup_errors', []).append(str(exc))
        (out/'result.json').write_text(json.dumps(result, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps(result, indent=2))
    print('SAVED:', out)
    return 0 if result['communication_response_valid'] else 1


if __name__ == '__main__':
    sys.exit(main())
