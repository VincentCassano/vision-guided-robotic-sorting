#!/usr/bin/env python3
"""Persistent inference process using an explicitly selected Python environment."""
import argparse
import base64
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--repository', required=True)
    parser.add_argument('--weights', required=True)
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--square-confidence', type=float, default=0.25)
    parser.add_argument('--class-confidences', type=float, nargs=4)
    args = parser.parse_args()
    protocol_output = sys.stdout
    sys.stdout = sys.stderr
    import cv2
    import numpy as np
    if Path(args.weights).suffix == '.torchscript':
        from battery_grasp.jit_inference import BatteryDetector
    else:
        from battery_grasp.yolo_inference import BatteryDetector
    detector = BatteryDetector(args.repository, args.weights, args.device, square_confidence=args.square_confidence, class_confidences=args.class_confidences)
    protocol_output.write(json.dumps({'ready': True}) + '\n')
    protocol_output.flush()
    for line in sys.stdin:
        try:
            request = json.loads(line)
            encoded = np.frombuffer(base64.b64decode(request['image'], validate=True), dtype=np.uint8)
            result = {'detections': detector.detect(cv2.imdecode(encoded, cv2.IMREAD_COLOR))}
        except Exception as exc:
            result = {'error': str(exc)}
        protocol_output.write(json.dumps(result, allow_nan=False) + '\n')
        protocol_output.flush()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
