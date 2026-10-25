#!/usr/bin/env python3
"""Live ROS acceptance: must be run with rostest, not an offline substitute."""
import json
import threading
import time
import unittest
from pathlib import Path

import rospy
import rostest
from std_msgs.msg import String
from std_srvs.srv import Trigger
from battery_grasp.ros_support import config, decode
from battery_grasp.sorting_core import STEPS, validate_transition


class RosStage4Test(unittest.TestCase):
    def test_full_pipeline(self):
        cfg = config()
        events,results,states,errors = [],[],[],[]
        lock = threading.RLock()

        def collect(storage,msg):
            with lock:
                try:
                    storage.append(decode(msg))
                except ValueError as exc:
                    errors.append(str(exc))

        subscriptions = [rospy.Subscriber(topic,String,lambda msg,s=storage:collect(s,msg),queue_size=200)
                         for topic,storage in [('/grasp_state',events),('/sorting_result',results),('/sorting_state',states)]]
        rospy.wait_for_service('/battery_grasp/status',timeout=15)
        status = rospy.ServiceProxy('/battery_grasp/status',Trigger)
        ready_deadline = time.monotonic()+15
        while time.monotonic()<ready_deadline:
            if json.loads(status().message)['ready']:
                break
            time.sleep(0.1)
        self.assertTrue(json.loads(status().message)['ready'],'Initial scene not ready')
        self.assertTrue(rospy.ServiceProxy('/battery_grasp/start',Trigger)().success)
        deadline = time.monotonic()+60
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with lock:
                if any(s.get('state') in ('DONE','ERROR') for s in states):
                    break
            time.sleep(0.05)
        time.sleep(0.2)
        with lock:
            report = {'events':events,'results':results,'states':states,'decode_errors':errors}
            path = Path(rospy.get_param('~report_path','/tmp/battery_grasp_stage4_ros.json'))
            path.write_text(json.dumps(report,indent=2),encoding='utf-8')
            self.assertFalse(errors)
            self.assertTrue(states and states[-1]['state']=='DONE',str(states[-1:] or 'No sorting state'))
            self.assertEqual(len(results),9)
            self.assertTrue(all(r['status']=='DONE' for r in results))
            expected = ['r{}c{}'.format(row,col) for col in range(3) for row in range(3)]
            self.assertEqual([r['target_id'] for r in results],expected)
            self.assertEqual(sum(r['bin']=='LEFT_UPPER_BIN' for r in results),7)
            self.assertEqual(sum(r['bin']=='LEFT_LOWER_BIN' for r in results),2)
            self.assertTrue(all(r['arm']=='LEFT' for r in results))
            self.assertEqual(len(events),81)
            current = {arm:list(cfg['arms'][arm]['home']) for arm in ('LEFT','RIGHT')}
            for result in results:
                steps = [event for event in events if event['task_id']==result['task_id']]
                self.assertEqual([e['state'] for e in steps],list(STEPS))
                for event in steps:
                    validate_transition(current[event['arm']],event['position'],cfg)
                    current[event['arm']] = event['position']
            self.assertEqual(len(states[-1]['completed']),9)
        for subscription in subscriptions:
            subscription.unregister()


if __name__ == '__main__':
    rospy.init_node('verify_ros_stage4')
    rostest.rosrun('battery_grasp','stage4_live_pipeline',RosStage4Test)
