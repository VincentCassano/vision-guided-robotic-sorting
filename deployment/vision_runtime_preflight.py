#!/usr/bin/env python3
"""Read-only Jetson inference diagnostics; no camera or robot connection."""
import argparse
import json
import os
from pathlib import Path
import platform
import sys
import traceback


def inspect_runtime(root, weights, device):
    result = dict(python=sys.executable, python_version=sys.version,
                  architecture=platform.machine(), weights=str(weights),
                  requested_device=device, automatic_install=False)
    print('VISION RUNTIME: '+json.dumps(result), flush=True)
    for path in [weights, root/'src/battery_grasp/scripts/yolo_worker.py',
                 root/'src/battery_grasp/src/battery_grasp/jit_inference.py']:
        if not path.is_file():
            raise RuntimeError('Missing packaged vision dependency: '+str(path))
    if weights.suffix != '.torchscript':
        raise RuntimeError('Jetson launcher requires the accepted TorchScript model')
    import cv2
    import numpy
    import torch
    result.update(torch=str(torch.__version__), opencv=cv2.__version__,
                  numpy=numpy.__version__, cuda_available=torch.cuda.is_available())
    selected = ('cuda:0' if result['cuda_available'] else 'cpu') if device=='auto' else device
    if selected.isdigit(): selected='cuda:'+selected
    parsed = torch.device(selected)
    if parsed.type == 'cuda':
        if not result['cuda_available']:
            raise RuntimeError('Requested CUDA is unavailable in '+sys.executable)
        result['gpu'] = torch.cuda.get_device_name(parsed)
    result['selected_device'] = str(parsed)
    sys.path.insert(0,str(root/'src/battery_grasp/src'))
    from battery_grasp.jit_inference import BatteryDetector
    result.update(dependency_check_pass=True, model_execution_tested=False,
                  camera_streaming_tested=False, robot_connected=False)
    return result


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--weights',type=Path,required=True)
    parser.add_argument('--device',default='auto')
    args=parser.parse_args()
    result={}
    try:
        result=inspect_runtime(args.root,args.weights,args.device)
        print('VISION DEPENDENCIES: '+json.dumps(result),flush=True)
    except Exception as exc:
        result.update(dependency_check_pass=False,error=repr(exc),traceback=traceback.format_exc(),
                      python=sys.executable,architecture=platform.machine(),
                      weights=str(args.weights),requested_device=args.device)
        print(json.dumps(result),file=sys.stderr,flush=True)
        return 1
    finally:
        destination=os.environ.get('LOCALIZED_VISION_DIAGNOSTICS')
        if destination:
            Path(destination).write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    return 0


if __name__=='__main__':
    raise SystemExit(main())
