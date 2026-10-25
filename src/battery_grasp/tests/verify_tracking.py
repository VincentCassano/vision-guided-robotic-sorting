#!/usr/bin/env python3
import copy
import sys
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from battery_grasp.observation_tracking import ObservationTracker, CATEGORIES

ROTATION = [[0, -1, 0], [-1, 0, 0], [0, 0, -1]]
TRANSLATION = [.65, 0, 1.1]


def fixture(stamp=1000000000, points=None, classes=None):
    points = points if points is not None else [[0, 0, .8], [.15, 0, .8]]
    classes = classes if classes is not None else [0] * len(points)
    return {'schema_version': 1, 'stamp_ns': stamp, 'frame_id': 'battery_replay_optical_frame',
            'simulation_only': True, 'depth_source': 'synthetic_test', 'execution_ready': False,
            'valid': True, 'rejected': [],
            'objects': [dict(class_id=c, category=CATEGORIES[c], position_optical_m=p)
                        for p, c in zip(points, classes)]}


class TrackingTests(unittest.TestCase):
    def setUp(self):
        self.tracker = ObservationTracker(ROTATION, TRANSLATION)

    def run_frame(self, stamp=1000000000, points=None, classes=None):
        return self.tracker.process(fixture(stamp, points, classes), stamp)

    def test_known_transform_and_flags(self):
        result = self.run_frame(points=[[.1, .2, .8]], classes=[3])
        np.testing.assert_allclose(result['objects'][0]['position_base_m'], [.45, -.1, .3])
        self.assertEqual(result['frame_id'], 'base_link')
        self.assertFalse(result['execution_ready'])
        self.assertFalse(result['objects'][0]['grasp_supported'])

    def test_reordered_and_moving(self):
        first = self.run_frame()['objects']
        second = self.run_frame(1100000000, [[.151, 0, .8], [.001, 0, .8]])['objects']
        self.assertEqual([o['track_id'] for o in second], [first[1]['track_id'], first[0]['track_id']])
        self.assertTrue(all(o['association'] == 'matched' for o in second))

    def test_class_change_starts_new_track(self):
        first = self.run_frame(points=[[0, 0, .8]])['objects'][0]
        second = self.run_frame(1100000000, [[0, 0, .8]], [1])['objects'][0]
        self.assertNotEqual(first['track_id'], second['track_id'])

    def test_gap_starts_new_tracks(self):
        first = self.run_frame()['objects'][0]['track_id']
        self.assertNotEqual(first, self.run_frame(6000000000)['objects'][0]['track_id'])

    def test_empty_frame_drops_tracks(self):
        first = self.run_frame()['objects'][0]['track_id']
        self.assertEqual(self.run_frame(1100000000, [])['objects'], [])
        self.assertNotEqual(first, self.run_frame(1200000000)['objects'][0]['track_id'])

    def test_missing_target_not_resurrected(self):
        first = self.run_frame()['objects'][1]['track_id']
        self.run_frame(1100000000, [[0, 0, .8]])
        self.assertNotEqual(first, self.run_frame(1200000000)['objects'][1]['track_id'])

    def test_duplicate_positions_are_ambiguous(self):
        result = self.run_frame(points=[[0, 0, .8], [.001, 0, .8]])
        self.assertTrue(all(o['track_id'] is None for o in result['objects']))

    def test_two_old_candidates_not_guessed(self):
        self.run_frame(points=[[-.015, 0, .8], [.015, 0, .8]])
        result = self.run_frame(1100000000, [[0, 0, .8]])
        self.assertEqual(result['objects'][0]['association'], 'ambiguous')

    def test_one_old_two_new_not_assigned_twice(self):
        self.run_frame(points=[[0, 0, .8]])
        result = self.run_frame(1100000000, [[-.015, 0, .8], [.015, 0, .8]])
        self.assertTrue(all(o['track_id'] is None for o in result['objects']))

    def test_large_motion_starts_new_track(self):
        first = self.run_frame(points=[[0, 0, .8]])['objects'][0]['track_id']
        self.assertNotEqual(first, self.run_frame(1100000000, [[.1, 0, .8]])['objects'][0]['track_id'])

    def test_bad_frames_clear_association(self):
        for field, value in [('schema_version', 2), ('valid', False), ('execution_ready', True),
                             ('simulation_only', False), ('frame_id', 'wrong'),
                             ('depth_source', 'real'), ('rejected', [{}])]:
            with self.subTest(field=field):
                self.setUp()
                self.run_frame()
                frame = fixture(1100000000)
                frame[field] = value
                with self.assertRaises(ValueError):
                    self.tracker.process(frame, 1100000000)
                self.assertEqual(self.tracker.previous, [])

    def test_bad_points_and_classes(self):
        for update in [dict(position_optical_m=[float('nan'), 0, .8]),
                       dict(position_optical_m=[0, .8]), dict(class_id=99),
                       dict(category='DEFECT_RED')]:
            frame = fixture()
            frame['objects'][0].update(update)
            with self.assertRaises(ValueError):
                self.tracker.process(frame, 1000000000)

    def test_stale_future_duplicate_and_old(self):
        for stamp, now in [(1000000000, 12000000000), (2000000000, 1000000000)]:
            with self.assertRaises(ValueError):
                self.tracker.process(fixture(stamp), now)
        self.run_frame()
        for stamp in [1000000000, 999999999]:
            with self.assertRaises(ValueError):
                self.tracker.process(fixture(stamp), 1100000000)

    def test_invalid_transforms(self):
        for rotation, translation in [(np.zeros((3, 3)), TRANSLATION),
                                      (np.diag([1, 1, -1]), TRANSLATION),
                                      (ROTATION, [0, float('inf'), 0])]:
            with self.assertRaises(ValueError):
                ObservationTracker(rotation, translation)

    def test_input_not_mutated(self):
        frame = fixture()
        reference = copy.deepcopy(frame)
        self.tracker.process(frame, frame['stamp_ns'])
        self.assertEqual(frame, reference)


if __name__ == '__main__':
    unittest.main()
