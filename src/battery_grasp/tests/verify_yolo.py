#!/usr/bin/env python3
import argparse
import json
import sys
import os
import signal
from pathlib import Path
import cv2
import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'src'))
from battery_grasp.inference_client import InferenceClient
from battery_grasp.yolo_inference import NAMES, CATEGORIES, detection_message


def check_image(path, detections):
    height, width = cv2.imread(str(path)).shape[:2]
    labels = path.parent.parent / 'labels' / (path.stem + '.txt')
    truth = [list(map(float, line.split())) for line in labels.read_text().splitlines()]
    assert len(truth) == len(detections), (path.name, len(truth), len(detections))
    available = set(range(len(truth)))
    for detection in detections:
        cls = detection['class_id']
        assert detection['class_name'] == NAMES[cls]
        assert detection['category'] == CATEGORIES[cls]
        assert detection['grasp_supported'] == (cls != 3)
        assert 0.25 <= detection['confidence'] <= 1
        x, y, w, h = detection['bbox']
        assert 0 <= x < x+w <= width and 0 <= y < y+h <= height
        assert detection['u'] == x+w/2 and detection['v'] == y+h/2
        candidates = []
        for index in available:
            target_cls, cx, cy, tw, th = truth[index]
            if target_cls != cls:
                continue
            tx, ty, tw, th = (cx-tw/2)*width, (cy-th/2)*height, tw*width, th*height
            intersection = max(0, min(x+w, tx+tw)-max(x, tx))*max(0, min(y+h, ty+th)-max(y, ty))
            iou = intersection / (w*h + tw*th - intersection)
            candidates.append((iou, index))
        assert candidates and max(candidates)[0] >= 0.5, (path.name, detection)
        available.remove(max(candidates)[1])
    return len(truth)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--repository', required=True)
    parser.add_argument('--python', default=sys.executable)
    parser.add_argument('--square-confidence', type=float, default=0.25)
    parser.add_argument('--output', default=str(PACKAGE.parents[1] / 'docs/yolo_verification.json'))
    args = parser.parse_args()
    client = InferenceClient(args.python, str(PACKAGE/'scripts/yolo_worker.py'), args.repository,
                             str(PACKAGE/'models/battery_yolov5s_v1.pt'), square_confidence=args.square_confidence)
    records = []
    try:
        for path in sorted((PACKAGE/'test_data/battery_v1/images').glob('*.JPG')):
            detections = client.detect(cv2.imread(str(path)))
            count = check_image(path, detections)
            message = detection_message(detections, 1234567890123456789, 'replay')
            assert json.loads(json.dumps(message, allow_nan=False))['stamp_ns'] == 1234567890123456789
            records.append({'image': path.name, 'matched': count, 'detections': detections})
        assert len(records) == 5 and sum(r['matched'] for r in records) == 18
        blank_detections = client.detect(np.zeros((480, 640, 3), dtype=np.uint8))
        dark_frames = [{'gray_level': 0, 'detections': blank_detections}]
        for level in [8, 16]:
            dark_frames.append({'gray_level': level, 'detections': client.detect(np.full((480, 640, 3), level, np.uint8))})
        if args.square_confidence >= 0.60:
            assert all(not frame['detections'] for frame in dark_frames), dark_frames
        assert detection_message([], 0, 'empty')['detections'] == []
        try:
            client.detect(np.zeros((8, 8, 3), dtype=np.float64))
        except (ValueError, cv2.error):
            pass
        else:
            raise AssertionError('Invalid image accepted')
        assert client.process.poll() is None
        sigint_checked = False
        if os.name == 'posix':
            client.process.send_signal(signal.SIGINT)
            assert client.process.wait(timeout=10) == 0, 'Worker did not exit cleanly on SIGINT'
            sigint_checked = True
        result = {'passed': True, 'device': 'cpu', 'scope': 'Actual checkpoint and persistent subprocess; ROS pending',
                  'images': records, 'blank_image_empty': not blank_detections,
                  'blank_image_detections': blank_detections,
                  'square_confidence': args.square_confidence, 'dark_frames': dark_frames,
                  'sigint_clean_exit_checked': sigint_checked,
                  'model_ready_for_sorting': False}
        Path(args.output).write_text(json.dumps(result, indent=2), encoding='utf-8')
        print('PASS interface: 5 images, 18 matched objects, all four classes and JSON timestamps')
        print('Blank-image false positives:', len(blank_detections))
    finally:
        client.close()


if __name__ == '__main__':
    main()
