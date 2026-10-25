#!/usr/bin/env python3
import copy
import json
import sys
import threading
import time
import unittest
from pathlib import Path
import cv2
import numpy as np
import rospy
import rostest
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
sys.path.insert(0, str(Path(__file__).resolve().parent))
from verify_yolo import check_image


class RgbdRosTest(unittest.TestCase):
    def test_projection_and_invalid_frames(self):
        received, observed, condition = {}, {}, threading.Condition()

        def callback(msg, destination):
            value = json.loads(msg.data)
            with condition:
                destination[value['stamp_ns']] = value
                condition.notify_all()

        subscriptions = [rospy.Subscriber('/battery_vision/objects_3d', String, lambda m: callback(m, received)),
                         rospy.Subscriber('/battery_vision/detections', String, lambda m: callback(m, observed))]
        rgb_pub = rospy.Publisher('/battery_vision/image', Image, queue_size=1, latch=True)
        depth_pub = rospy.Publisher('/battery_vision/depth', Image, queue_size=1, latch=True)
        info_pub = rospy.Publisher('/battery_vision/camera_info', CameraInfo, queue_size=1, latch=True)
        detection_pub = rospy.Publisher('/battery_vision/detections', String, queue_size=1)
        bridge = CvBridge()

        def wait(stamp):
            deadline = time.monotonic()+180
            with condition:
                while (stamp not in received or stamp not in observed) and time.monotonic() < deadline:
                    condition.wait(1)
                self.assertIn(stamp, received, 'No aligned 3D result')
                self.assertIn(stamp, observed, 'No detection message')
                return received[stamp]

        paths = sorted(Path(rospy.get_param('~images')).glob('*.JPG'))
        self.assertEqual(len(paths), 5)
        count = 0
        for index, path in enumerate(paths):
            pixels = cv2.imread(str(path))
            height, width = pixels.shape[:2]
            rgb = bridge.cv2_to_imgmsg(pixels, 'bgr8')
            rgb.header.stamp = rospy.Time.now()
            rgb.header.frame_id = 'battery_replay_optical_frame'
            encoding = '16UC1' if index % 2 == 0 else '32FC1'
            values = np.full((height, width), 800, np.uint16) if encoding == '16UC1' else np.full((height, width), .8, np.float32)
            depth = bridge.cv2_to_imgmsg(values, encoding)
            depth.header = copy.deepcopy(rgb.header)
            info = CameraInfo()
            info.header = copy.deepcopy(rgb.header)
            info.width, info.height = width, height
            info.K = [float(width), 0., width/2., 0., float(width), height/2., 0., 0., 1.]
            info.D = [0.]*5
            rgb_pub.publish(rgb)
            depth_pub.publish(depth)
            info_pub.publish(info)
            result = wait(rgb.header.stamp.to_nsec())
            self.assertTrue(result['valid'])
            self.assertTrue(result['simulation_only'])
            self.assertEqual(result['depth_source'], 'synthetic_test')
            self.assertFalse(result['execution_ready'])
            self.assertEqual(result['frame_id'], rgb.header.frame_id)
            count += check_image(path, result['objects'])
            for obj in result['objects']:
                np.testing.assert_allclose(obj['position_optical_m'],
                    [(obj['u']/width-.5)*.8, (obj['v']-height/2)/width*.8, .8], atol=1e-6)
        self.assertEqual(count, 18)
        template = observed[rgb.header.stamp.to_nsec()]
        cases = []
        for fault in ['zero_depth', 'frame_mismatch', 'invalid_intrinsics', 'rgb_size_mismatch']:
            stamp = rospy.Time.now()
            detections = copy.deepcopy(template)
            detections.update(stamp_ns=stamp.to_nsec(), stamp=stamp.to_sec())
            sample = bridge.cv2_to_imgmsg(np.full((height, width), 0 if fault == 'zero_depth' else 800, np.uint16), '16UC1')
            sample.header.stamp = stamp
            sample.header.frame_id = rgb.header.frame_id
            camera = copy.deepcopy(info)
            camera.header.stamp = stamp
            if fault == 'frame_mismatch':
                sample.header.frame_id = 'wrong_frame'
            elif fault == 'invalid_intrinsics':
                camera.K = [0.]*9
            elif fault == 'rgb_size_mismatch':
                detections['image_width'] = 1
            detection_pub.publish(String(data=json.dumps(detections)))
            depth_pub.publish(sample)
            info_pub.publish(camera)
            result = wait(stamp.to_nsec())
            self.assertFalse(result['valid'], fault)
            self.assertEqual(result['objects'], [], fault)
            self.assertFalse(result['execution_ready'])
            cases.append(fault)
        stamp = rospy.Time.now()
        detections = copy.deepcopy(template)
        detections.update(stamp_ns=stamp.to_nsec(), stamp=stamp.to_sec())
        sample = bridge.cv2_to_imgmsg(np.full((height, width), 800, np.uint16), '16UC1')
        sample.header.frame_id = rgb.header.frame_id
        sample.header.stamp = rospy.Time(stamp.secs, stamp.nsecs) + rospy.Duration(0, 1)
        camera = copy.deepcopy(info)
        camera.header.stamp = stamp
        detection_pub.publish(String(data=json.dumps(detections)))
        depth_pub.publish(sample)
        info_pub.publish(camera)
        time.sleep(.5)
        with condition:
            self.assertNotIn(stamp.to_nsec(), received)
        sample.header.stamp = stamp
        depth_pub.publish(sample)
        self.assertTrue(wait(stamp.to_nsec())['valid'])
        cases.append('timestamp_mismatch_waits_for_matching_depth')
        Path('/tmp/battery_grasp_rgbd_ros.json').write_text(json.dumps({'passed': True,
            'matched_objects': count, 'invalid_cases': cases, 'depth_source': 'synthetic_test'}))
        for sub in subscriptions:
            sub.unregister()


if __name__ == '__main__':
    rospy.init_node('verify_ros_rgbd')
    rostest.rosrun('battery_grasp', 'rgbd_projection', RgbdRosTest)
