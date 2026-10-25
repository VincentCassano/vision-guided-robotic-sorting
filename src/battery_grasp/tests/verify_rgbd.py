#!/usr/bin/env python3
import argparse
import json
import sys
import unittest
from pathlib import Path
import cv2
import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'src'))
from battery_grasp.rgbd import FrameMatcher, project_frame
from battery_grasp.sorting_core import UnsafePlan
from battery_grasp.inference_client import InferenceClient
from battery_grasp.yolo_inference import detection_message
from verify_yolo import check_image


def fixture():
    detection = detection_message([{'category': 'NORMAL_BLACK', 'u': 400., 'v': 240.}], 100, 'optical', 640, 480)
    depth = {'stamp_ns': 100, 'frame_id': 'optical', 'encoding': '16UC1', 'image': np.full((480, 640), 800, np.uint16)}
    camera = {'stamp_ns': 100, 'frame_id': 'optical', 'width': 640, 'height': 480,
              'K': [600., 0., 320., 0., 600., 240., 0., 0., 1.], 'D': [0.]*5}
    return detection, depth, camera


class RgbdTests(unittest.TestCase):
    def test_known_optical_point(self):
        output = project_frame(*fixture())
        np.testing.assert_allclose(output['objects'][0]['position_optical_m'], [0.1066666667, 0., .8])
        self.assertTrue(output['valid'] and output['simulation_only'])
        self.assertFalse(output['execution_ready'])

    def test_metres_and_millimetres_agree(self):
        detection, depth, camera = fixture()
        reference = project_frame(detection, depth, camera)
        depth.update(encoding='32FC1', image=np.full((480, 640), .8, np.float32))
        actual = project_frame(detection, depth, camera)
        np.testing.assert_allclose(actual['objects'][0]['position_optical_m'], reference['objects'][0]['position_optical_m'], atol=1e-7)

    def test_invalid_depths_are_not_objects(self):
        for value in [0., float('nan'), float('inf'), .1, 2.]:
            with self.subTest(value=value):
                detection, depth, camera = fixture()
                depth.update(encoding='32FC1', image=np.full((480, 640), value, np.float32))
                result = project_frame(detection, depth, camera)
                self.assertFalse(result['valid'])
                self.assertEqual(result['objects'], [])
                self.assertEqual(result['rejected'][0]['detection_index'], 0)

    def test_invalid_samples_excluded_from_median(self):
        detection, depth, camera = fixture()
        depth['image'][:] = 0
        depth['image'][240, 400] = 800
        self.assertTrue(project_frame(detection, depth, camera)['valid'])

    def test_out_of_bounds_and_nan_pixel(self):
        for value in [-100., 1000., float('nan')]:
            detection, depth, camera = fixture()
            detection['detections'][0]['u'] = value
            self.assertFalse(project_frame(detection, depth, camera)['valid'])

    def test_stamp_and_frame_mismatch(self):
        for key, value in [('stamp_ns', 101), ('frame_id', 'different')]:
            detection, depth, camera = fixture()
            depth[key] = value
            with self.assertRaises(UnsafePlan):
                project_frame(detection, depth, camera)

    def test_size_mismatch(self):
        for target in ['depth', 'rgb']:
            detection, depth, camera = fixture()
            if target == 'depth':
                depth['image'] = depth['image'][:100]
            else:
                detection['image_width'] = 320
            with self.assertRaises(UnsafePlan):
                project_frame(detection, depth, camera)

    def test_encoding_and_dtype_rejected(self):
        for encoding in ['mono16', '32FC1']:
            detection, depth, camera = fixture()
            depth['encoding'] = encoding
            with self.assertRaises(UnsafePlan):
                project_frame(detection, depth, camera)

    def test_camera_matrix_and_distortion_rejected(self):
        for index, value in [(0, 0), (4, float('nan')), (1, 1)]:
            detection, depth, camera = fixture()
            camera['K'][index] = value
            with self.assertRaises(UnsafePlan):
                project_frame(detection, depth, camera)
        detection, depth, camera = fixture()
        camera['D'][0] = .1
        with self.assertRaises(UnsafePlan):
            project_frame(detection, depth, camera)

    def test_empty_frame_and_square_retained(self):
        detection, depth, camera = fixture()
        detection['detections'][0].update(category='UNSUPPORTED_SQUARE', grasp_supported=False)
        self.assertFalse(project_frame(detection, depth, camera)['objects'][0]['grasp_supported'])
        detection['detections'] = []
        result = project_frame(detection, depth, camera)
        self.assertTrue(result['valid'])
        self.assertEqual(result['objects'], [])

    def test_exact_pairing_and_no_repeated_output(self):
        matcher = FrameMatcher()
        self.assertIsNone(matcher.add(0, 100, 'detection'))
        self.assertIsNone(matcher.add(1, 101, 'wrong_depth'))
        self.assertIsNone(matcher.add(2, 100, 'camera'))
        self.assertEqual(matcher.add(1, 100, 'depth'), ('detection', 'depth', 'camera'))
        self.assertIsNone(matcher.add(0, 100, 'duplicate'))
        self.assertIsNone(matcher.add(0, 99, 'old'))

    def test_pending_frames_bounded(self):
        matcher = FrameMatcher()
        for stamp in range(1, 101):
            matcher.add(1, stamp, 'depth')
        self.assertEqual(len(matcher.buffers[1]), 4)
        self.assertIsNone(matcher.add(0, 1, 'expired'))
        self.assertIsNone(matcher.add(2, 1, 'expired'))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--repository')
    parser.add_argument('--output', default=str(PACKAGE.parents[1]/'docs/rgbd_verification.json'))
    args = parser.parse_args()
    outcome = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(RgbdTests))
    if not outcome.wasSuccessful():
        raise SystemExit(1)
    matched = None
    if args.repository:
        client = InferenceClient(sys.executable, str(PACKAGE/'scripts/yolo_worker.py'), args.repository,
                                 str(PACKAGE/'models/battery_yolov5s_v1.pt'), square_confidence=.60)
        matched = 0
        try:
            for index, path in enumerate(sorted((PACKAGE/'test_data/battery_v1/images').glob('*.JPG'))):
                pixels = cv2.imread(str(path))
                height, width = pixels.shape[:2]
                objects = client.detect(pixels)
                matched += check_image(path, objects)
                stamp = index+1
                detection = detection_message(objects, stamp, 'optical', width, height)
                depth = {'stamp_ns': stamp, 'frame_id': 'optical', 'encoding': '16UC1', 'image': np.full((height, width), 800, np.uint16)}
                camera = {'stamp_ns': stamp, 'frame_id': 'optical', 'width': width, 'height': height,
                          'K': [width, 0, width/2, 0, width, height/2, 0, 0, 1]}
                result = project_frame(detection, depth, camera)
                assert result['valid'] and len(result['objects']) == len(objects)
                for obj in result['objects']:
                    np.testing.assert_allclose(obj['position_optical_m'],
                        [(obj['u']/width-.5)*.8, (obj['v']-height/2)/width*.8, .8], atol=1e-7)
            assert matched == 18
        finally:
            client.close()
    Path(args.output).write_text(json.dumps({'passed': True, 'unit_tests': outcome.testsRun,
        'model_matched_objects': matched, 'depth_source': 'synthetic_test', 'ros_tested': False}, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
