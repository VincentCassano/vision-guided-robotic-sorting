#!/usr/bin/env python3
import json
import threading
import time
import unittest
from pathlib import Path
from urllib.request import Request,urlopen
from urllib.error import HTTPError,URLError

import rospy
import rostest
from std_msgs.msg import String


class RosStage5Test(unittest.TestCase):
    def request(self,path,method='GET'):
        request = Request('http://127.0.0.1:{}/'.format(rospy.get_param('~api_port',18082))+path,method=method)
        try:
            response = urlopen(request,timeout=4)
        except HTTPError as exc:
            response = exc
        with response:
            return response.status,json.load(response)

    def wait_status(self,predicate,timeout=15):
        timeout = max(timeout,rospy.get_param('~minimum_wait',0))
        deadline = time.monotonic()+timeout
        last = None
        while time.monotonic()<deadline and not rospy.is_shutdown():
            try:
                code,last = self.request('status')
                if code==200 and predicate(last):
                    return last
            except (URLError,TimeoutError):
                pass
            time.sleep(0.1)
        self.fail('Status timeout: '+str(last))

    def test_http_state_and_stop_reset(self):
        events,results,scenes = [],[],[]
        lock = threading.RLock()
        def collect(storage,msg):
            with lock:
                storage.append(json.loads(msg.data))
        subscribers = [rospy.Subscriber(topic,String,lambda m,s=storage:collect(s,m),queue_size=200)
                       for topic,storage in [('/grasp_state',events),('/sorting_result',results)]]
        if rospy.get_param('~require_observation_bridge',False):
            subscribers.append(rospy.Subscriber('/battery_vision/sorting_scene',String,lambda m:collect(scenes,m),queue_size=20))
        initial = self.wait_status(lambda s:s['state']=='IDLE' and s['ready'])
        time.sleep(0.8)
        with lock:
            self.assertEqual(events,[],'Startup unexpectedly executed movement')
        self.assertEqual(self.request('start')[0],200)
        deadline = time.monotonic()+8
        while time.monotonic()<deadline:
            with lock:
                if len(events)>=3:
                    break
            time.sleep(0.03)
        with lock:
            self.assertGreaterEqual(len(events),3)
        self.assertEqual(self.request('reset','POST')[0],409)
        code,stopped = self.request('stop','POST')
        self.assertEqual(code,202)
        self.wait_status(lambda s:s['state']=='IDLE' and s['reset_required'])
        time.sleep(0.7)
        with lock:
            self.assertTrue(all(e['stamp']<=stopped['stamp'] for e in events),'Step emitted after STOP acknowledgement')
            self.assertEqual(results,[],'Interrupted target marked completed')
        self.assertEqual(self.request('start','POST')[0],409)
        self.assertEqual(self.request('reset','POST')[0],200)
        fresh = self.wait_status(lambda s:s['ready'])
        self.assertNotEqual(initial['session_id'],fresh['session_id'])
        with lock:
            events.clear()
            results.clear()
        self.assertEqual(self.request('mode/manual','POST')[1]['mode'],'MANUAL')
        for count in (1,2):
            self.assertEqual(self.request('start','POST')[0],200)
            self.wait_status(lambda s:s['state']=='IDLE' and len(s['completed'])==count and s['ready'])
            time.sleep(0.5)
            with lock:
                self.assertEqual(len(results),count)
                self.assertEqual(len(events),count*9)
        self.assertEqual(self.request('mode/auto','POST')[1]['mode'],'AUTO')
        self.assertEqual(self.request('start','POST')[0],200)
        final = self.wait_status(lambda s:s['state'] in ('DONE','ERROR'),timeout=45)
        self.assertEqual(final['state'],'DONE',final)
        time.sleep(0.3)
        with lock:
            self.assertEqual(len(events),81)
            self.assertEqual(len(results),9)
            self.assertEqual(len({r['target_id'] for r in results}),9)
            self.assertTrue(all(r['status']=='DONE' for r in results))
            report = {'initial':initial,'stop':stopped,'final':final,'events':list(events),'results':list(results)}
        self.assertEqual(self.request('reset','POST')[0],200)
        ready = self.wait_status(lambda s:s['ready'])
        self.assertEqual(ready['completed'],[])
        if rospy.get_param('~require_observation_bridge',False):
            with lock:
                valid_scenes = [s for s in scenes if s.get('valid')]
                self.assertTrue(valid_scenes,'No valid adapted observation scene')
                self.assertTrue(all(s.get('adapter')=='stable_observation_grid' and s.get('execution_ready') is False for s in valid_scenes))
                self.assertTrue(any(len(s['objects'])==9 for s in valid_scenes))
                self.assertTrue(any(len(s['objects'])==0 for s in valid_scenes))
                report['observation_adapter_verified'] = True
        Path(rospy.get_param('~report','/tmp/battery_grasp_stage5_ros.json')).write_text(json.dumps(report,indent=2),encoding='utf-8')
        for subscriber in subscribers:
            subscriber.unregister()


if __name__=='__main__':
    rospy.init_node('verify_ros_stage5')
    rostest.rosrun('battery_grasp','stage5_http_control',RosStage5Test)
