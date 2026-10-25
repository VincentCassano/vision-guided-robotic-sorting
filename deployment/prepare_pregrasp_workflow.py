#!/usr/bin/env python3
"""One integrated pregrasp packet; never connects to hardware."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/battery_grasp/src'))
from battery_grasp.pregrasp_workflow import compile_workflow


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--route', choices=['taught_joint', 'fixed_wrist_vertical'], default='taught_joint')
    p.add_argument('--output-parent', type=Path, default=ROOT/'captures')
    p.add_argument('--execute', action='store_true', help='Refused before SDK import/connection until an actual physical route is accepted')
    args = p.parse_args(argv)
    manifest = ROOT/'deployment/pregrasp-workflow-files.sha256'
    for line in manifest.read_text(encoding='ascii').splitlines():
        digest, name = line.split('  ', 1)
        path = (ROOT/name).resolve()
        if ROOT.resolve() not in path.parents or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            p.error('Installed workflow file changed: '+name+'; no connection')
    native = (ROOT/'deployment/taught_pregrasp_200357.json').read_bytes()
    review = (ROOT/'deployment/pregrasp_measurements_162445_review/review.json').read_bytes()
    plan = compile_workflow(native, review, args.route)
    plan['execute_requested'] = args.execute
    plan['outcome'] = 'execution_refused_before_hardware_connection' if args.execute else 'integrated_workflow_saved_no_hardware_connection'
    folder = args.output_parent/('pregrasp_workflow_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    folder.mkdir(parents=True, exist_ok=False)
    (folder/'workflow.json').write_text(json.dumps(plan, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    with zipfile.ZipFile(folder.with_suffix('.zip'), 'x', zipfile.ZIP_DEFLATED) as z:
        z.write(folder/'workflow.json', folder.name+'/workflow.json')
    print('SAVED:', folder.with_suffix('.zip'))
    print('已复用数值与量尺；候选臂命令数:', len(plan['command_specification']), 'EXECUTION_READY: False')
    for row in plan['blockers']:
        print('BLOCK:', row['reason'])
    if args.execute:
        print('本包包含控制适配源码，但没有已验收的实体路径，连接前拒绝执行；没有移动设备。')
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
