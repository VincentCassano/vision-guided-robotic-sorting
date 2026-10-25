#!/usr/bin/env python3
import copy
import json
import unittest
from pathlib import Path
from verify_observation_sorting import ObservationSortingTests


class ObservationFaultTests(unittest.TestCase):
    def setUp(self):
        self.h = ObservationSortingTests()
        self.h.setUp()
        self.h.ready()
        self.h.control.command('START',self.h.now)
        self.h.control.tick(self.h.now)

    def assert_locked(self):
        control = self.h.control
        self.assertEqual(control.state,'ERROR',control.reason)
        self.assertTrue(control.reset_required)
        self.assertEqual(control.planner.completed,set())
        before = copy.deepcopy(control.executor.poses)
        for _ in range(3):
            self.h.frame()
            self.assertTrue(all(v is None for v in control.tick(self.h.now).values()))
            self.assertFalse(control.command('START',self.h.now)[0]['accepted'])
        self.assertEqual(control.executor.poses,before)

    def test_transient_invalid_frame_cannot_be_overwritten(self):
        self.h.control.observe(self.h.adapter.process(None,int(self.h.now*1e9)))
        self.h.frame()
        self.h.frame()
        self.assert_locked()

    def test_transient_inventory_change_cannot_be_overwritten(self):
        self.h.completed.add('r2c2')
        self.h.frame()
        self.h.completed.clear()
        self.h.frame()
        self.h.frame()
        self.assert_locked()

    def test_dropout_times_out_before_next_step(self):
        self.h.now += self.h.cfg['max_age']+.01
        output = self.h.control.tick(self.h.now)
        self.assertTrue(all(v is None for v in output.values()))
        self.assert_locked()

    def test_displacement_rejected_and_latched(self):
        obs = self.h.observation()
        obs['objects'][0]['position_base_m'][0] += .025
        self.h.control.observe(self.h.adapter.process(obs,obs['stamp_ns']))
        self.assert_locked()

    def test_cumulative_small_motion_compared_to_task_snapshot(self):
        for distance in (.009,.018,.027):
            obs = self.h.observation()
            obs['objects'][0]['position_base_m'][0] += distance
            scene = self.h.adapter.process(obs,obs['stamp_ns'])
            self.assertTrue(scene['valid'])
            self.h.control.observe(scene)
        self.assert_locked()

    def test_new_target_does_not_join_batch(self):
        self.h.setUp()
        self.h.completed.add('r2c2')
        self.h.ready()
        self.h.control.command('START',self.h.now)
        self.h.control.tick(self.h.now)
        self.h.completed.clear()
        self.h.frame()
        self.assertEqual(len(self.h.control.planner.inventory),8)
        self.assert_locked()

    def test_category_change_latches_error(self):
        self.h.cfg['scene_categories'][0][0]='DEFECT_RED'
        self.h.frame()
        self.assert_locked()

    def test_error_requires_reset_ack_and_two_new_frames(self):
        self.h.control.observe(None)
        self.assert_locked()
        control = self.h.control
        old_session = control.planner.session
        response,event = control.command('RESET',self.h.now)
        self.assertTrue(response['accepted'])
        self.assertNotEqual(event['session_id'],old_session)
        self.h.adapter.reset(int(round(event['stamp']*1e9)))
        control.confirm_scene_reset(old_session,self.h.now)
        self.assertIsNone(control.scene_ack)
        control.confirm_scene_reset(event['session_id'],self.h.now)
        self.assertFalse(self.h.frame()['valid'])
        self.assertFalse(control.command('START',self.h.now)[0]['accepted'])
        self.assertTrue(self.h.frame()['valid'])
        self.assertTrue(control.command('START',self.h.now)[0]['accepted'])


if __name__=='__main__':
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(ObservationFaultTests))
    Path(__file__).resolve().parents[3].joinpath('docs/observation_faults_verification.json').write_text(
        json.dumps(dict(passed=result.wasSuccessful(),tests=result.testsRun,simulation_only=True,ros_tested=False),indent=2),encoding='utf-8')
    raise SystemExit(0 if result.wasSuccessful() else 1)
