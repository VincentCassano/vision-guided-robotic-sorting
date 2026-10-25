#!/usr/bin/env python3
import json
import threading
import time
import unittest
import rospy
import rostest
from std_msgs.msg import String


class TrackingRosTest(unittest.TestCase):
    def test_transform_association_and_rejection(self):
        received = []
        event = threading.Event()

        def receive(msg):
            received.append(json.loads(msg.data))
            event.set()

        sub = rospy.Subscriber('/battery_vision/observations_base', String, receive)
        pub = rospy.Publisher('/battery_vision/objects_3d', String, queue_size=1)
        deadline = time.monotonic() + 10
        while (not pub.get_num_connections() or not sub.get_num_connections()) and time.monotonic() < deadline:
            time.sleep(.05)
        self.assertGreater(pub.get_num_connections(), 0)
        self.assertGreater(sub.get_num_connections(), 0)

        def send(points, classes=None, **overrides):
            time.sleep(.02)
            stamp = rospy.Time.now().to_nsec()
            classes = classes if classes is not None else [0] * len(points)
            categories = ['NORMAL_BLACK', 'DEFECT_RED', 'DEFECT_WHITE', 'UNSUPPORTED_SQUARE']
            frame = dict(schema_version=1, stamp_ns=stamp, frame_id='battery_replay_optical_frame',
                         simulation_only=True, depth_source='synthetic_test', execution_ready=False,
                         valid=True, rejected=[], objects=[
                             dict(class_id=c, category=categories[c], position_optical_m=p)
                             for p, c in zip(points, classes)])
            frame.update(overrides)
            event.clear()
            pub.publish(String(data=json.dumps(frame)))
            self.assertTrue(event.wait(5), 'No observation response')
            return received[-1]

        first = send([[.1, .2, .8], [.25, .2, .8]], [0, 3])
        self.assertTrue(first['valid'])
        for actual, expected in zip(first['objects'][0]['position_base_m'], [.45, -.1, .3]):
            self.assertAlmostEqual(actual, expected)
        second = send([[.251, .2, .8], [.101, .2, .8]], [3, 0])
        self.assertEqual([o['track_id'] for o in second['objects']],
                         [first['objects'][1]['track_id'], first['objects'][0]['track_id']])
        self.assertFalse(second['execution_ready'])
        self.assertFalse(second['objects'][0]['grasp_supported'])
        for overrides in [dict(frame_id='wrong'), dict(valid=False), dict(execution_ready=True),
                          dict(stamp_ns=rospy.Time.now().to_nsec() - 20000000000)]:
            bad = send([[0, 0, .8]], **overrides)
            self.assertFalse(bad['valid'])
            self.assertEqual(bad['objects'], [])
        ambiguous = send([[0, 0, .8], [.001, 0, .8]])
        self.assertTrue(all(o['track_id'] is None for o in ambiguous['objects']))
        self.assertEqual(send([])['objects'], [])
        with open('/tmp/battery_grasp_tracking_ros.json', 'w') as stream:
            json.dump(dict(passed=True, simulation_only=True, execution_ready=False,
                           scope='ROS transform and association with controlled synthetic observations'), stream)
        sub.unregister()
        pub.unregister()


if __name__ == '__main__':
    rospy.init_node('verify_ros_tracking')
    rostest.rosrun('battery_grasp', 'observation_tracking', TrackingRosTest)
