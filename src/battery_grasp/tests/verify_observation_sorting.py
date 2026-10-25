#!/usr/bin/env python3
import copy
import json
import unittest
from pathlib import Path
import numpy as np
from verify_stage4 import CFG
from battery_grasp.perception import render_scene, detect_batteries
from battery_grasp.rgbd import project_frame
from battery_grasp.observation_tracking import ObservationTracker
from battery_grasp.observation_adapter import ObservationAdapter
from battery_grasp.sorting_core import CLASSES
from battery_grasp.task_control import TaskController


class ObservationSortingTests(unittest.TestCase):
    def setUp(self):
        self.cfg = copy.deepcopy(CFG)
        camera = self.cfg['camera']
        self.tracker = ObservationTracker(camera['rotation'],camera['translation'],camera['frame_id'])
        self.adapter = ObservationAdapter(self.cfg)
        self.control = TaskController(self.cfg,0)
        self.control.confirm_scene_reset(self.control.planner.session,0)
        self.now = 0.
        self.completed = set()

    def observation(self):
        self.now = round(self.now + .1, 6)
        stamp = int(round(self.now*1e9))
        rgb, depth = render_scene(self.cfg,self.completed)
        detections = detect_batteries(rgb)
        for item in detections:
            item.update(class_id=CLASSES.index(item['category']),grasp_supported=True)
        cam = self.cfg['camera']
        fx,fy,cx,cy = cam['intrinsics']
        detection = dict(schema_version=1, stamp_ns=stamp, frame_id=cam['frame_id'],
                         image_width=640,image_height=480,detections=detections)
        info = dict(stamp_ns=stamp,frame_id=cam['frame_id'],width=640,height=480,
                    K=[fx,0,cx,0,fy,cy,0,0,1])
        dep = dict(stamp_ns=stamp,frame_id=cam['frame_id'],image=depth,encoding='32FC1')
        return self.tracker.process(project_frame(detection,dep,info),stamp)

    def frame(self):
        observation = self.observation()
        scene = self.adapter.process(observation,observation['stamp_ns'])
        self.control.observe(scene)
        return scene

    def ready(self):
        self.assertFalse(self.frame()['valid'])
        self.assertTrue(self.frame()['valid'])
        self.assertTrue(self.control.status(self.now)['ready'])

    def run_until(self, expected, limit=400):
        results = []
        for _ in range(limit):
            self.frame()
            output = self.control.tick(self.now)
            if output['result']:
                self.assertEqual(output['result']['status'],'DONE')
                results.append(output['result'])
                self.completed.add(output['result']['target_id'])
            self.assertNotEqual(self.control.state,'ERROR',self.control.reason)
            if self.control.state == expected:
                return results
        self.fail('State timeout: '+self.control.state)

    def test_manual_then_auto_full_image_closed_loop(self):
        self.ready()
        self.assertTrue(all(v is None for v in self.control.tick(self.now).values()))
        self.control.command('MODE_MANUAL',self.now)
        results = []
        for count in (1,2):
            self.assertTrue(self.control.command('START',self.now)[0]['accepted'])
            results.extend(self.run_until('IDLE'))
            self.assertEqual(len(self.completed),count)
            for _ in range(4):
                self.frame()
                self.assertTrue(all(v is None for v in self.control.tick(self.now).values()))
        self.control.command('MODE_AUTO',self.now)
        self.assertTrue(self.control.command('START',self.now)[0]['accepted'])
        results.extend(self.run_until('DONE'))
        self.assertEqual(len(results),9)
        self.assertEqual(len({r['target_id'] for r in results}),9)
        self.assertEqual([r['target_id'] for r in results],['r{}c{}'.format(r,c) for c in range(3) for r in range(3)])
        self.assertEqual(sum(r['bin']=='LEFT_UPPER_BIN' for r in results),7)
        self.assertEqual(self.control.latest['objects'],[])

    def test_stop_and_reset_require_new_stable_frames(self):
        self.ready()
        self.control.command('START',self.now)
        self.control.tick(self.now)
        self.control.command('STOP',self.now)
        self.control.tick(self.now)
        self.assertFalse(self.control.command('START',self.now)[0]['accepted'])
        _,event = self.control.command('RESET',self.now)
        self.adapter.reset(int(round(event['stamp']*1e9)))
        self.control.confirm_scene_reset(event['session_id'],self.now)
        self.assertFalse(self.frame()['valid'])
        self.assertFalse(self.control.command('START',self.now)[0]['accepted'])
        self.assertTrue(self.frame()['valid'])
        self.assertTrue(self.control.command('START',self.now)[0]['accepted'])

    def test_reject_workspace_grid_duplicate_square_and_ambiguity(self):
        for mutation in ('workspace','grid','duplicate','square','ambiguous','track'):
            with self.subTest(mutation=mutation):
                self.setUp()
                self.ready()
                obs = self.observation()
                item = obs['objects'][0]
                if mutation=='workspace': item['position_base_m']=[9,9,9]
                if mutation=='grid': item['position_base_m']=[.3,.3,.3]
                if mutation=='duplicate': obs['objects'][1]['position_base_m']=item['position_base_m'][:]
                if mutation=='square': item.update(class_id=3,category='UNSUPPORTED_SQUARE',grasp_supported=False)
                if mutation=='ambiguous': item['association']='ambiguous'
                if mutation=='track': item['track_id']=None
                rejected = self.adapter.process(obs,obs['stamp_ns'])
                self.assertFalse(rejected['valid'])
                self.assertEqual(rejected['objects'],[])
                self.control.observe(rejected)
                self.assertFalse(self.control.command('START',self.now)[0]['accepted'])

    def test_stale_future_duplicate_and_reset_barrier(self):
        self.ready()
        obs = self.observation()
        for now in (obs['stamp_ns']-1,obs['stamp_ns']+2000000000):
            self.assertFalse(self.adapter.process(obs,now)['valid'])
        self.adapter.reset(obs['stamp_ns'])
        self.assertFalse(self.adapter.process(obs,obs['stamp_ns'])['valid'])
        self.assertFalse(self.frame()['valid'])
        self.assertTrue(self.frame()['valid'])
        self.assertFalse(self.adapter.process(obs,obs['stamp_ns'])['valid'])

    def test_category_change_restarts_stability(self):
        self.ready()
        self.cfg['scene_categories'][0][0]='DEFECT_RED'
        self.assertFalse(self.frame()['valid'])
        self.assertTrue(self.frame()['valid'])

    def test_track_jump_restarts_stability(self):
        self.ready()
        obs = self.observation()
        obs['objects'][0]['track_id'] += 100
        self.assertFalse(self.adapter.process(obs,obs['stamp_ns'])['valid'])

    def test_invalid_observation_stops_running_controller(self):
        self.ready()
        self.control.command('START',self.now)
        self.control.tick(self.now)
        self.control.observe(self.adapter.process(None,int(self.now*1e9)))
        output = self.control.tick(self.now)
        self.assertEqual(self.control.state,'ERROR')
        self.assertIsNone(output['event'])
        self.assertEqual(self.completed,set())

    def test_missing_uncompleted_target_stops(self):
        self.ready()
        self.control.command('START',self.now)
        self.control.tick(self.now)
        self.completed.add('r2c2')
        for _ in range(5):
            self.frame()
            self.control.tick(self.now)
            if self.control.state=='ERROR': break
        self.assertEqual(self.control.state,'ERROR')
        self.assertEqual(self.control.planner.completed,set())

    def test_small_batch_can_complete(self):
        self.completed = {'r{}c{}'.format(r,c) for r in range(3) for c in range(3)} - {'r0c0','r1c0'}
        self.ready()
        self.control.command('START',self.now)
        self.assertEqual(len(self.run_until('DONE')),2)

    def test_empty_scene_does_not_start(self):
        self.completed = {'r{}c{}'.format(r,c) for r in range(3) for c in range(3)}
        self.assertTrue(self.frame()['valid'])
        self.assertFalse(self.control.command('START',self.now)[0]['accepted'])

    def test_message_contract_rejections(self):
        for key,value in [('schema_version',2),('frame_id','wrong'),('source_frame_id','wrong'),
                          ('execution_ready',True),('simulation_only',False),('depth_source','real'),
                          ('transform_source','real'),('valid',False),('rejected',['bad'])]:
            obs = self.observation()
            obs[key]=value
            self.assertFalse(self.adapter.process(obs,int(self.now*1e9))['valid'])

    def test_nonfinite_point_rejected(self):
        obs = self.observation()
        obs['objects'][0]['position_base_m'][0]=float('nan')
        self.assertFalse(self.adapter.process(obs,obs['stamp_ns'])['valid'])


if __name__=='__main__':
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(ObservationSortingTests))
    report = dict(passed=result.wasSuccessful(),tests=result.testsRun,simulation_only=True,ros_tested=False,
                  input='Rendered RGB-D images and deterministic color detector; no YOLO claims',
                  chain='RGB-D -> optical -> base observations -> stable grid adapter -> TaskController/SimExecutor')
    Path(__file__).resolve().parents[3].joinpath('docs/observation_sorting_verification.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    raise SystemExit(0 if result.wasSuccessful() else 1)
