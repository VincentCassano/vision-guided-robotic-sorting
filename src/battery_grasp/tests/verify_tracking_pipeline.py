#!/usr/bin/env python3
import argparse
import json
import sys
import unittest
from pathlib import Path
import cv2
import numpy as np
import yaml
from verify_tracking import TrackingTests, ObservationTracker
from verify_rgbd import RgbdTests, project_frame, detection_message, InferenceClient, check_image, PACKAGE


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--repository', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    suite = unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromTestCase(cls)
                               for cls in (TrackingTests, RgbdTests))
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    if not result.wasSuccessful():
        raise SystemExit(1)
    cfg = yaml.safe_load((PACKAGE/'config/sorting_simulation.yaml').read_text())['battery_grasp_sim']['camera']
    tracker = ObservationTracker(cfg['rotation'], cfg['translation'])
    client = InferenceClient(sys.executable, str(PACKAGE/'scripts/yolo_worker.py'), args.repository,
                             str(PACKAGE/'models/battery_yolov5s_v1.pt'), square_confidence=.60)
    matched, seen = 0, set()
    try:
        for index, path in enumerate(sorted((PACKAGE/'test_data/battery_v1/images').glob('*.JPG'))):
            pixels = cv2.imread(str(path))
            height, width = pixels.shape[:2]
            objects = client.detect(pixels)
            matched += check_image(path, objects)
            stamp = (index * 5 + 1) * 1000000000
            frame_id = 'battery_replay_optical_frame'
            detection = detection_message(objects, stamp, frame_id, width, height)
            depth = dict(stamp_ns=stamp, frame_id=frame_id, encoding='16UC1',
                         image=np.full((height, width), 800, np.uint16))
            camera = dict(stamp_ns=stamp, frame_id=frame_id, width=width, height=height,
                          K=[width, 0, width/2, 0, width, height/2, 0, 0, 1])
            output = tracker.process(project_frame(detection, depth, camera), stamp)
            assert output['valid'] and not output['execution_ready']
            assert len(output['objects']) == len(objects)
            ids = set()
            for obj in output['objects']:
                np.testing.assert_allclose(obj['position_base_m'],
                    [.65 - (obj['v'] - height/2)/width*.8, -(obj['u']/width-.5)*.8, .3], atol=1e-7)
                assert obj['association'] in ('new', 'ambiguous')
                if obj['track_id'] is not None:
                    assert obj['track_id'] not in seen and obj['track_id'] not in ids
                    ids.add(obj['track_id'])
                if obj['class_id'] == 3:
                    assert not obj['grasp_supported']
            seen.update(ids)
        assert matched == 18
    finally:
        client.close()
    Path(args.output).write_text(json.dumps(dict(passed=True, unit_tests=result.testsRun,
        model_matched_objects=matched, simulation_only=True, execution_ready=False, ros_tested=False,
        scope='Actual model -> synthetic RGB-D -> configured base transform; five independent photos'), indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
