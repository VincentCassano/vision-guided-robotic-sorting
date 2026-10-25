#!/usr/bin/env python3
"""Run the real image/perception/planning core without mocking its algorithms."""
import copy
import json
from pathlib import Path
import sys
import unittest

import numpy as np
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PACKAGE/'src'))
from battery_grasp.perception import render_scene, detect_batteries, localize
from battery_grasp.sorting_core import Planner, SimExecutor, UnsafePlan, build_sequence, validate_sequence, validate_transition, validate_config

CFG = yaml.safe_load((PACKAGE/'config/sorting_simulation.yaml').read_text(encoding='utf-8'))['battery_grasp_sim']


def scene(cfg=CFG, completed=()):
    rgb,depth = render_scene(cfg,completed)
    objects,rejected = localize(detect_batteries(rgb),depth,cfg['camera']['intrinsics'],cfg)
    assert not rejected
    return objects


def result(task,stamp):
    return {'schema_version':1,'task_id':task['task_id'],'status':'DONE','target_id':task['target']['id'],
            'arm':task['arm'],'bin':task['bin'],'final_pose':list(CFG['bins'][task['bin']]),'stamp':stamp}


def run_cycle(cfg=CFG):
    planner = Planner(cfg,'offline-test')
    executor = SimExecutor(cfg)
    executor.set_running(True)
    trace = []
    for index in range(10):
        stamp = float(index*10+1)
        objects = scene(cfg,planner.completed)
        task = planner.observe(objects,stamp,stamp+0.01)
        if task is None:
            return planner,trace
        sequence = build_sequence(task,cfg)
        validate_sequence(sequence,cfg,planner.poses[task['arm']])
        executor.observe(snapshot(objects,stamp))
        assert executor.accept(sequence,stamp)
        events = []
        for step in range(9):
            now = stamp+(step+1)*cfg['step_delay']
            executor.observe(snapshot(scene(cfg,planner.completed),now))
            event,completed = executor.tick(now)
            assert event is not None
            events.append(event)
        assert completed is not None and completed['status']=='DONE'
        trace.append(sequence)
        assert planner.finish(completed)
    raise AssertionError('Cycle did not terminate')


def snapshot(objects,stamp=1):
    return {'schema_version':1,'frame_id':'base_link','valid':True,'objects':objects,'stamp':stamp}


class Stage4Tests(unittest.TestCase):
    def test_nine_image_detections_and_categories(self):
        objects = scene()
        self.assertEqual(len(objects),9)
        self.assertEqual(sum(o['category']=='NORMAL_BLACK' for o in objects),7)
        self.assertEqual(sum(o['category']=='DEFECT_RED' for o in objects),1)
        self.assertEqual(sum(o['category']=='DEFECT_WHITE' for o in objects),1)

    def test_full_closed_loop_order_bins_and_paths(self):
        planner,trace = run_cycle()
        self.assertEqual(planner.status,'DONE')
        self.assertEqual([s['target']['id'] for s in trace],['r0c0','r1c0','r2c0','r0c1','r1c1','r2c1','r0c2','r1c2','r2c2'])
        self.assertEqual({s['arm'] for s in trace},{'LEFT'})
        self.assertEqual(sum(s['bin']=='LEFT_UPPER_BIN' for s in trace),7)
        self.assertEqual(sum(s['bin']=='LEFT_LOWER_BIN' for s in trace),2)
        for sequence in trace:
            steps = sequence['steps']
            self.assertEqual(steps[1]['position'][:2],steps[2]['position'][:2])
            self.assertEqual(steps[4]['position'][:2],steps[5]['position'][:2])
            self.assertEqual(steps[6]['position'][2],CFG['motion']['safe_z'])
            self.assertEqual(steps[7]['state'],'OPEN_HAND')

    def test_grid_identity_survives_removal(self):
        all_ids = {o['id'] for o in scene()}
        self.assertEqual({o['id'] for o in scene(completed=['r0c0','r2c1'])},all_ids-{'r0c0','r2c1'})

    def test_incomplete_initial_inventory_blocks(self):
        p = Planner(CFG)
        self.assertIsNone(p.observe(scene()[:-1],1,1))
        self.assertEqual(p.status,'BLOCKED')
        self.assertIsNotNone(p.observe(scene(),2,2))

    def test_missing_uncompleted_is_not_cleared(self):
        p = Planner(CFG)
        task = p.observe(scene(),1,1)
        p.finish(result(task,2))
        self.assertIsNone(p.observe(scene(completed=['r0c0','r1c0']),3,3))
        self.assertEqual(p.completed,{'r0c0'})

    def test_old_snapshot_waits_for_removal(self):
        p = Planner(CFG)
        task = p.observe(scene(),1,1)
        p.finish(result(task,2))
        self.assertIsNone(p.observe(scene(),3,3))
        self.assertIsNone(p.observe(scene(completed=['r0c0']),2,2))
        self.assertIsNotNone(p.observe(scene(completed=['r0c0']),3,3))

    def test_duplicate_or_unrelated_completion_ignored(self):
        p = Planner(CFG)
        task = p.observe(scene(),1,1)
        bad = result(task,2)
        bad['task_id'] = 'unrelated'
        self.assertFalse(p.finish(bad))
        self.assertTrue(p.finish(result(task,2)))
        self.assertFalse(p.finish(result(task,2)))

    def test_wrong_bin_completion_rejected(self):
        p = Planner(CFG)
        task = p.observe(scene(),1,1)
        bad = result(task,2)
        bad['bin'] = 'LEFT_LOWER_BIN'
        with self.assertRaises(UnsafePlan):
            p.finish(bad)
        self.assertFalse(p.completed)

    def test_execution_failure_does_not_remove_target(self):
        p = Planner(CFG)
        task = p.observe(scene(),1,1)
        p.finish({'task_id':task['task_id'],'status':'ERROR','reason':'test failure'})
        self.assertEqual(p.status,'ERROR')
        self.assertFalse(p.completed)
        self.assertIsNone(p.observe(scene(),2,2))

    def test_stale_and_future_perception(self):
        p = Planner(CFG)
        self.assertIsNone(p.observe(scene(),1,5))
        self.assertIsNone(p.observe(scene(),10,5))

    def test_duplicate_slot_blocks(self):
        objects = scene()
        objects.append(copy.deepcopy(objects[0]))
        self.assertIsNone(Planner(CFG).observe(objects,1,1))

    def test_unsupported_square_category_blocks(self):
        objects = scene()
        objects[0]['category'] = 'square_battery'
        self.assertIsNone(Planner(CFG).observe(objects,1,1))

    def test_changed_category_blocks(self):
        p = Planner(CFG)
        task = p.observe(scene(),1,1)
        p.finish(result(task,2))
        objects = scene(completed=p.completed)
        objects[0]['category'] = 'DEFECT_WHITE'
        self.assertIsNone(p.observe(objects,3,3))

    def test_invalid_depth_rejected(self):
        rgb,depth = render_scene(CFG)
        depth[:,:] = np.nan
        targets,rejected = localize(detect_batteries(rgb),depth,CFG['camera']['intrinsics'],CFG)
        self.assertEqual(targets,[])
        self.assertEqual(len(rejected),9)

    def test_nonfinite_target_blocks(self):
        objects = scene()
        objects[0]['position'][0] = float('nan')
        self.assertIsNone(Planner(CFG).observe(objects,1,1))

    def test_low_sideways_or_diagonal_motion_rejected(self):
        with self.assertRaises(UnsafePlan):
            validate_transition([0.5,0,0.3],[0.6,0,0.3],CFG)
        with self.assertRaises(UnsafePlan):
            validate_transition([0.5,0,0.3],[0.6,0,0.65],CFG)

    def test_tampered_sequence_rejected(self):
        p = Planner(CFG)
        task = p.observe(scene(),1,1)
        sequence = build_sequence(task,CFG)
        sequence['steps'][2]['position'][1] += 0.05
        with self.assertRaises(UnsafePlan):
            validate_sequence(sequence,CFG,p.poses['LEFT'])

    def test_unsafe_height_config_rejected(self):
        cfg = copy.deepcopy(CFG)
        cfg['motion']['safe_z'] = 0.35
        with self.assertRaises(UnsafePlan):
            validate_config(cfg)

    def test_blocked_vertical_corridor(self):
        cfg = copy.deepcopy(CFG)
        cfg['obstacles'].append({'name':'blocking cover','bounds':{'x_min':0.49,'x_max':0.82,'y_min':-0.20,'y_max':0.20,'z_min':0.29,'z_max':0.40}})
        p = Planner(cfg)
        self.assertIsNone(p.observe(scene(cfg),1,1))
        self.assertEqual(p.status,'BLOCKED')

    def test_middle_requires_same_side_clearance(self):
        p = Planner(CFG)
        task = p.observe(scene(),1,1)
        task['target'] = next(o for o in task['objects'] if o['id']=='r0c1')
        task['bin'] = 'LEFT_LOWER_BIN'
        with self.assertRaises(UnsafePlan):
            build_sequence(task,CFG)
        task['objects'] = [o for o in task['objects'] if o['column']!=0]
        build_sequence(task,CFG)

    def test_right_disabled_unreachable_targets_block(self):
        cfg = copy.deepcopy(CFG)
        cfg['arms']['LEFT']['reach']['y_min'] = -0.05
        p,trace = run_cycle(cfg)
        self.assertEqual(len(trace),6)
        self.assertEqual(p.status,'BLOCKED')

    def test_right_requires_explicit_verification(self):
        cfg = copy.deepcopy(CFG)
        cfg['arms']['LEFT']['reach']['y_min'] = -0.05
        cfg['arms']['right_enabled'] = True
        p,trace = run_cycle(cfg)
        self.assertEqual(len(trace),6)
        cfg['arms']['right_paths_verified'] = True
        p,trace = run_cycle(cfg)
        self.assertEqual(p.status,'DONE')
        self.assertEqual([s['arm'] for s in trace],['LEFT']*6+['RIGHT']*3)

    def test_executor_waits_for_startup_scene(self):
        sequence = build_sequence(Planner(CFG).observe(scene(),1,1),CFG)
        executor = SimExecutor(CFG)
        executor.set_running(True)
        self.assertFalse(executor.accept(sequence,1))
        executor.observe(snapshot(scene()))
        self.assertTrue(executor.accept(sequence,1))
        self.assertFalse(executor.accept(sequence,1))

    def test_executor_timeout_stops_before_grasp(self):
        sequence = build_sequence(Planner(CFG).observe(scene(),1,1),CFG)
        executor = SimExecutor(CFG)
        executor.set_running(True)
        executor.observe(snapshot(scene()))
        executor.accept(sequence,1)
        event,completion = executor.tick(1.25)
        self.assertEqual(event['state'],'SAFE_APPROACH')
        event,completion = executor.tick(3)
        self.assertIsNone(event)
        self.assertEqual(completion['status'],'ERROR')
        self.assertFalse(executor.accept(sequence,3))
        self.assertEqual(executor.tick(4),(None,None))

    def test_executor_detects_midflight_scene_change(self):
        sequence = build_sequence(Planner(CFG).observe(scene(),1,1),CFG)
        executor = SimExecutor(CFG)
        executor.set_running(True)
        executor.observe(snapshot(scene()))
        executor.accept(sequence,1)
        executor.observe(snapshot(scene()[:-1],1.25))
        event,completion = executor.tick(1.25)
        self.assertIsNone(event)
        self.assertEqual(completion['status'],'ERROR')

    def test_executor_rejects_wrong_schema_and_frame(self):
        sequence = build_sequence(Planner(CFG).observe(scene(),1,1),CFG)
        executor = SimExecutor(CFG)
        executor.set_running(True)
        executor.observe(snapshot(scene()))
        for key,value in [('schema_version',2),('frame_id','camera_test_frame')]:
            altered = copy.deepcopy(sequence)
            altered[key] = value
            with self.assertRaises(UnsafePlan):
                executor.accept(altered,1)

    def test_executor_stale_start_pose_rejected(self):
        sequence = build_sequence(Planner(CFG).observe(scene(),1,1),CFG)
        executor = SimExecutor(CFG)
        executor.set_running(True)
        executor.observe(snapshot(scene()))
        executor.poses['LEFT'][0] += 0.05
        with self.assertRaises(UnsafePlan):
            executor.accept(sequence,1)

    def test_unknown_grid_target_is_not_silently_dropped(self):
        rgb,depth = render_scene(CFG)
        detections = detect_batteries(rgb)
        detections[0]['u'] += 40
        objects,rejected = localize(detections,depth,CFG['camera']['intrinsics'],CFG)
        self.assertTrue(rejected)

    def test_middle_right_requires_right_side_clear(self):
        cfg = copy.deepcopy(CFG)
        cfg['arms']['right_enabled'] = cfg['arms']['right_paths_verified'] = True
        task = Planner(cfg).observe(scene(),1,1)
        task['arm'],task['start_pose'] = 'RIGHT',cfg['arms']['RIGHT']['home']
        task['target'] = next(o for o in task['objects'] if o['id']=='r0c1')
        task['bin'] = 'LEFT_LOWER_BIN'
        task['objects'] = [o for o in task['objects'] if o['column']!=2]
        build_sequence(task,cfg)

    def test_invalid_rigid_transform_rejected(self):
        cfg = copy.deepcopy(CFG)
        cfg['camera']['rotation'][0][0] = 1
        with self.assertRaises(UnsafePlan):
            validate_config(cfg)


if __name__ == '__main__':
    outcome = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(Stage4Tests))
    planner,trace = run_cycle()
    output = PACKAGE.parents[1]/'docs'/'stage4_verification.json'
    output.write_text(json.dumps({'tests_run':outcome.testsRun,'passed':outcome.wasSuccessful(),
                                  'scope':'Offline shared image, localization, planning, trajectory and timer executor; ROS transport not tested',
                                  'state':planner.status,'completed':sorted(planner.completed),'trace':trace},indent=2),encoding='utf-8')
    sys.exit(0 if outcome.wasSuccessful() else 1)

