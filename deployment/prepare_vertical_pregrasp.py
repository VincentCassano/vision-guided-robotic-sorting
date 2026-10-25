#!/usr/bin/env python3
"""Offline-only descent development using the accepted hover; no SDK/network."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/battery_grasp/src'))
from battery_grasp.pregrasp import build_pregrasp_plan

HOVER_SHA = 'd24e097e8b5f5740f8ab3119f8731a9a0a03736f94abb5fcc8e58ef48a947e77'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=Path(__file__).with_name('vertical_pregrasp_geometry.json'))
    parser.add_argument('--output',type=Path)
    args = parser.parse_args(argv)
    reference = Path(__file__).with_name('arm_hand_continuous_latest_120502.json')
    raw = reference.read_bytes()
    if hashlib.sha256(raw).hexdigest() != HOVER_SHA: parser.error('Accepted hover source mismatch')
    document = json.loads(raw)
    if document.get('outcome') != 'completed_continuous_hand_open' or document.get('current_role') != 'pick_hover':
        parser.error('Accepted hover/open reference missing')
    pose = document['last_arm_state']['pose']
    start = dict(tool_world_mm=[v*1000 for v in pose[:3]],tool_rpy_rad=pose[3:],source=reference.name)
    result = build_pregrasp_plan(start,json.loads(args.config.read_text(encoding='utf-8')))
    result['input_sha256'] = dict(hover=HOVER_SHA,config=hashlib.sha256(args.config.read_bytes()).hexdigest())
    result['source_sha256'] = {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in (Path(__file__),ROOT/'src/battery_grasp/src/battery_grasp/pregrasp.py',
                  ROOT/'src/battery_grasp/src/battery_grasp/real_grasp.py')}
    output = args.output or ROOT/'captures'/('vertical_pregrasp_development_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))/'plan.json'
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print('SAVED:',output,'PLANNING_READY:',result['planning_ready'],'EXECUTION_READY: False')
    for blocker in result['blockers']: print('BLOCK:',blocker)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
