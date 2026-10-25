#!/usr/bin/env python3
import json
import threading
import time
import unittest
from pathlib import Path
import rospy
import rostest
from std_msgs.msg import String
from std_srvs.srv import Trigger


class ObservationFaultRosTests(unittest.TestCase):
    def test_fault_latching_and_reset(self):
        self.lock = threading.RLock()
        self.session = None
        self.events, self.results = [], []
        self.pub = rospy.Publisher('/battery_vision/observations_base',String,queue_size=10)
        self.ack = rospy.Publisher('/battery_grasp/scene_ready',String,queue_size=1)

        def reset(msg):
            with self.lock:
                self.session = json.loads(msg.data)['session_id']

        def collect(store,msg):
            with self.lock:
                store.append(json.loads(msg.data))

        subscribers = [rospy.Subscriber('/battery_grasp/scene_reset',String,reset),
                       rospy.Subscriber('/grasp_state',String,lambda m:collect(self.events,m)),
                       rospy.Subscriber('/sorting_result',String,lambda m:collect(self.results,m))]
        try:
            for name in ('start','reset','status'):
                rospy.wait_for_service('/battery_grasp/'+name,timeout=10)
            self.wait(lambda s:s['ready'])
            reports = []
            for fault in ('invalid','dropout','moved','new_target','category'):
                before = len(self.events)
                self.assertTrue(self.command('start')['accepted'])
                self.wait(lambda s:len(self.events)>before)
                if fault!='dropout':
                    self.publish(fault)
                failed = self.wait(lambda s:s['state']=='ERROR',pump=False,timeout=4)
                time.sleep(.1)
                stopped_count = len(self.events)
                for _ in range(8):
                    self.publish()
                    time.sleep(.03)
                self.assertEqual(len(self.events),stopped_count,'Movement resumed after error')
                self.assertEqual(self.results,[],'Faulted target marked completed')
                self.assertFalse(self.command('start')['accepted'])
                old_session = failed['session_id']
                self.assertTrue(self.command('reset')['accepted'])
                ready = self.wait(lambda s:s['ready'] and s['session_id']!=old_session)
                self.assertEqual(ready['completed'],[])
                reports.append(dict(fault=fault,reason=failed['reason'],reset_recovered=True))
            Path('/tmp/battery_grasp_observation_faults_ros.json').write_text(json.dumps(
                dict(passed=True,simulation_only=True,cases=reports,
                     input='Controlled base observations injected into real ROS adapter and controller'),indent=2))
        finally:
            for sub in subscribers: sub.unregister()
            self.pub.unregister()
            self.ack.unregister()

    def command(self,name):
        return json.loads(rospy.ServiceProxy('/battery_grasp/'+name,Trigger)().message)

    def publish(self,fault='normal'):
        with self.lock:
            session = self.session
        if session is None: return
        stamp = rospy.Time.now()
        objects = [dict(class_id=0,category='NORMAL_BLACK',grasp_supported=True,track_id=i+1,
                        association='matched',position_base_m=[x,.16,.3])
                   for i,x in enumerate((.516666667,.65))]
        if fault=='moved': objects[0]['position_base_m'][0] += .025
        if fault=='new_target':
            objects.append(dict(class_id=0,category='NORMAL_BLACK',grasp_supported=True,track_id=3,
                                association='matched',position_base_m=[.783333333,.16,.3]))
        if fault=='category': objects[0].update(class_id=1,category='DEFECT_RED')
        frame = dict(schema_version=1,stamp_ns=stamp.to_nsec(),frame_id='base_link',
                     source_frame_id='camera_test_frame',simulation_only=True,execution_ready=False,
                     depth_source='synthetic_test',transform_source='configured_simulation',
                     valid=fault!='invalid',rejected=[] if fault!='invalid' else ['injected'],objects=objects)
        self.ack.publish(String(data=json.dumps(dict(schema_version=1,session_id=session,stamp=stamp.to_sec()))))
        self.pub.publish(String(data=json.dumps(frame)))

    def wait(self,predicate,pump=True,timeout=10):
        deadline = time.monotonic()+timeout
        last = None
        while time.monotonic()<deadline and not rospy.is_shutdown():
            if pump: self.publish()
            time.sleep(.03)
            last = self.command('status')
            if predicate(last): return last
        self.fail('Timeout: '+str(last))


if __name__=='__main__':
    rospy.init_node('verify_ros_observation_faults')
    rostest.rosrun('battery_grasp','observation_faults',ObservationFaultRosTests)
