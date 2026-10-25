#!/usr/bin/env python3
import copy
import json
import sys
import threading
import unittest
from pathlib import Path
from urllib.request import Request,urlopen
from urllib.error import HTTPError

sys.path.insert(0,str(Path(__file__).resolve().parent))
from verify_stage4 import CFG,scene,snapshot
from battery_grasp.task_control import TaskController
from battery_grasp.sorting_core import SimExecutor,Planner,build_sequence
from battery_grasp.http_api import create_server


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.control = TaskController(CFG,0)
        self.control.confirm_scene_reset(self.control.planner.session,1)
        self.control.observe(snapshot(scene(),1))

    def start(self):
        response,_ = self.control.command('START',1)
        self.assertEqual(response['http_status'],200)
        return self.control.tick(1)

    def test_idle_never_moves(self):
        self.assertTrue(self.control.status(1)['ready'])
        for now in (1,2,3):
            self.assertTrue(all(v is None for v in self.control.tick(now).values()))
        self.assertIsNone(self.control.executor.active)

    def test_executor_independently_locked_by_default(self):
        e = SimExecutor(CFG)
        e.observe(snapshot(scene(),1))
        sequence = build_sequence(Planner(CFG).observe(scene(),1,1),CFG)
        self.assertFalse(e.accept(sequence,1))
        self.assertEqual(e.tick(1),(None,None))

    def test_start_requires_fresh_nonempty_scene(self):
        for s in (None,snapshot([],1),snapshot(scene(),0)):
            self.control.observe(s)
            self.assertEqual(self.control.command('START',1)[0]['http_status'],409)
            self.assertEqual(self.control.state,'IDLE')

    def test_unknown_command(self):
        self.assertEqual(self.control.command('LAUNCH',1)[0]['http_status'],400)

    def test_repeated_start_is_idempotent(self):
        self.start()
        active = self.control.executor.active['task_id']
        self.control.command('START',1.1)
        self.assertEqual(self.control.executor.active['task_id'],active)

    def test_stop_at_every_step_prevents_next_step(self):
        for count in range(9):
            c = TaskController(CFG,0)
            c.confirm_scene_reset(c.planner.session,1)
            c.observe(snapshot(scene(),1))
            c.command('START',1)
            c.tick(1)
            now = 1
            for _ in range(count):
                now += 0.3
                c.observe(snapshot(scene(),now))
                c.tick(now)
            response,_ = c.command('STOP',now+0.01)
            self.assertEqual(response['state'],'STOPPING')
            before = copy.deepcopy(c.executor.poses)
            self.assertTrue(all(v is None for v in c.tick(now+0.3).values()))
            self.assertEqual(c.executor.poses,before)
            self.assertEqual(c.state,'IDLE')
            self.assertTrue(c.reset_required)
            self.assertFalse(c.planner.completed)
            self.assertEqual(c.command('START',now+0.4)[0]['http_status'],409)

    def test_reset_while_running_or_stopping_rejected(self):
        self.start()
        session = self.control.planner.session
        self.assertEqual(self.control.command('RESET',1)[0]['http_status'],409)
        self.control.command('STOP',1)
        self.assertEqual(self.control.command('RESET',1)[0]['http_status'],409)
        self.assertEqual(self.control.planner.session,session)

    def test_reset_rotates_session_and_waits_for_new_frame(self):
        old = self.control.planner.session
        response,event = self.control.command('RESET',2)
        self.assertNotEqual(old,event['session_id'])
        self.assertFalse(response['ready'])
        self.control.observe(snapshot(scene(),1.99))
        self.assertFalse(self.control.status(2)['ready'])
        self.control.observe(snapshot(scene(),2.01))
        self.assertFalse(self.control.status(2.02)['ready'])
        self.control.confirm_scene_reset(old,2.01)
        self.assertFalse(self.control.status(2.02)['ready'])
        self.control.confirm_scene_reset(self.control.planner.session,2.01)
        self.assertTrue(self.control.status(2.02)['ready'])

    def test_running_perception_timeout_is_error(self):
        self.start()
        self.control.tick(5)
        self.assertEqual(self.control.state,'ERROR')
        self.assertFalse(self.control.executor.running)
        self.assertEqual(self.control.command('START',5)[0]['http_status'],409)
        self.assertEqual(self.control.command('RESET',5)[0]['state'],'IDLE')

    def test_full_controlled_cycle(self):
        self.control.command('START',1)
        completed = set()
        events = []
        for i in range(200):
            now = 1+i*0.3
            self.control.observe(snapshot(scene(completed=completed),now))
            out = self.control.tick(now)
            if out['event']:
                events.append(out['event'])
            if out['result']:
                self.assertEqual(out['result']['status'],'DONE')
                completed.add(out['result']['target_id'])
            if self.control.state=='DONE':
                break
        self.assertEqual(self.control.state,'DONE')
        self.assertEqual(len(events),81)
        self.assertEqual(len(completed),9)
        self.assertFalse(self.control.executor.running)
        self.assertEqual(self.control.command('START',now)[0]['http_status'],409)

    def run_until_idle_or_done(self,available,now):
        results = []
        for _ in range(150):
            now += 0.3
            objects = [o for o in available if o['id'] not in self.control.planner.completed]
            self.control.observe(snapshot(objects,now))
            out = self.control.tick(now)
            if out['result']:
                results.append(out['result'])
            if self.control.state in ('IDLE','DONE','ERROR'):
                break
        self.assertNotEqual(self.control.state,'ERROR',self.control.reason)
        now += 0.3
        self.control.observe(snapshot([o for o in available if o['id'] not in self.control.planner.completed],now))
        return now,results

    def test_manual_one_command_one_target_then_continue(self):
        self.control.command('MODE_MANUAL',1)
        session = self.control.planner.session
        now = 1
        for count in range(1,10):
            self.assertEqual(self.control.command('START',now)[0]['http_status'],200)
            now,results = self.run_until_idle_or_done(scene(),now)
            self.assertEqual(len(results),1)
            self.assertEqual(self.control.state,'IDLE')
            self.assertEqual(len(self.control.planner.completed),count)
            self.assertEqual(self.control.planner.session,session)
            self.assertFalse(self.control.reset_required)
            self.assertTrue(all(v is None for v in self.control.tick(now).values()))
        self.assertEqual(self.control.status(now)['remaining'],0)
        self.assertEqual(self.control.command('START',now)[0]['http_status'],409)

    def test_busy_manual_command_not_queued(self):
        self.control.command('MODE_MANUAL',1)
        self.start()
        self.assertEqual(self.control.command('START',1.1)[0]['http_status'],409)

    def test_mode_change_rejected_during_execution(self):
        self.start()
        self.assertEqual(self.control.command('MODE_MANUAL',1.1)[0]['http_status'],409)
        self.assertEqual(self.control.mode,'AUTO')

    def test_manual_then_auto_preserves_inventory(self):
        self.control.command('MODE_MANUAL',1)
        self.control.command('START',1)
        now,results = self.run_until_idle_or_done(scene(),1)
        self.assertEqual(len(results),1)
        self.control.command('MODE_AUTO',now)
        self.control.command('START',now)
        now,results = self.run_until_idle_or_done(scene(),now)
        self.assertEqual(len(results),8)
        self.assertEqual(self.control.state,'DONE')

    def test_auto_accepts_three_visible_targets(self):
        available = [o for o in scene() if o['column']==0]
        self.control.observe(snapshot(available,1))
        self.assertEqual(self.control.command('START',1)[0]['total'],3)
        now,results = self.run_until_idle_or_done(available,1)
        self.assertEqual(len(results),3)
        self.assertEqual(self.control.state,'DONE')

    def test_missing_remaining_target_blocks_next_manual_start(self):
        self.control.command('MODE_MANUAL',1)
        self.control.command('START',1)
        now,_ = self.run_until_idle_or_done(scene(),1)
        objects = [o for o in scene() if o['id'] not in self.control.planner.completed]
        self.control.observe(snapshot(objects[:-1],now))
        self.assertEqual(self.control.command('START',now)[0]['http_status'],409)
        self.assertEqual(len(self.control.planner.completed),1)

    def test_reset_preserves_selected_mode(self):
        self.control.command('MODE_MANUAL',1)
        self.control.command('RESET',2)
        self.assertEqual(self.control.mode,'MANUAL')


class HttpTests(unittest.TestCase):
    def setUp(self):
        self.control = TaskController(CFG,0)
        self.control.confirm_scene_reset(self.control.planner.session,1)
        self.control.observe(snapshot(scene(),1))
        self.lock = threading.RLock()
        self.server = create_server('127.0.0.1',0,self.backend)
        self.thread = threading.Thread(target=self.server.serve_forever,daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(2)

    def backend(self,command):
        with self.lock:
            return self.control.status(1) if command=='STATUS' else self.control.command(command,1)[0]

    def request(self,path,method='GET',body=None):
        req = Request('http://127.0.0.1:{}/{}'.format(self.server.server_port,path),data=body,method=method)
        try:
            response = urlopen(req,timeout=3)
        except HTTPError as exc:
            response = exc
        with response:
            return response.status,json.load(response)

    def test_get_status_and_post_start_stop(self):
        code,body = self.request('status')
        self.assertEqual((code,body['state']),(200,'IDLE'))
        self.assertEqual(self.request('start','POST',b'{}')[1]['state'],'RUNNING')
        self.assertEqual(self.request('stop','POST')[0],202)

    def test_get_start_stop_compatibility(self):
        self.assertEqual(self.request('start')[0],200)
        self.assertEqual(self.request('stop')[0],202)

    def test_reset_is_post_only(self):
        self.assertEqual(self.request('reset')[0],405)
        self.assertEqual(self.request('reset','POST')[0],200)

    def test_bad_route_method_and_body(self):
        self.assertEqual(self.request('unknown')[0],404)
        self.assertEqual(self.request('start?x=1')[0],404)
        self.assertEqual(self.request('status','POST')[0],405)
        self.assertEqual(self.request('start','POST',b'{bad')[0],400)
        self.assertEqual(self.request('start','POST',b'{"speed":1}')[0],400)
        self.assertEqual(self.request('start','POST',b'x'*1025)[0],413)
        self.assertEqual(self.control.state,'IDLE')

    def test_conflict_propagates(self):
        self.request('start','POST')
        self.assertEqual(self.request('reset','POST')[0],409)

    def test_unavailable_backend(self):
        self.control = None
        self.assertEqual(self.request('status')[0],503)

    def test_mode_routes_and_status(self):
        self.assertEqual(self.request('mode/manual')[0],405)
        self.assertEqual(self.request('mode/manual','POST')[1]['mode'],'MANUAL')
        self.assertEqual(self.request('status')[1]['mode'],'MANUAL')
        self.assertEqual(self.request('mode/auto','POST')[1]['mode'],'AUTO')


if __name__=='__main__':
    suite = unittest.TestSuite([unittest.defaultTestLoader.loadTestsFromTestCase(cls) for cls in (ControlTests,HttpTests)])
    outcome = unittest.TextTestRunner(verbosity=2).run(suite)
    output = Path(__file__).resolve().parents[3]/'docs/stage5_verification.json'
    output.write_text(json.dumps({'tests_run':outcome.testsRun,'passed':outcome.wasSuccessful(),
                                  'scope':'Shared controller/executor and real local TCP HTTP; ROS transport not exercised'},indent=2),encoding='utf-8')
    sys.exit(0 if outcome.wasSuccessful() else 1)
