#!/usr/bin/env python3
import json
import threading
import time
import unittest
import sys
from pathlib import Path
import cv2
import numpy as np
import rospy
import rostest
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
sys.path.insert(0, str(Path(__file__).resolve().parent))
from verify_yolo import check_image


class YoloRosTest(unittest.TestCase):
    def test_actual_model_image_transport(self):
        received = {}
        condition = threading.Condition()

        def callback(msg):
            value = json.loads(msg.data)
            with condition:
                received[value['stamp_ns']] = value
                condition.notify_all()

        subscriber = rospy.Subscriber('/battery_vision/detections', String, callback)
        publisher = rospy.Publisher('/battery_vision/image', Image, queue_size=1, latch=True)
        directory = Path(rospy.get_param('~images'))
        paths = sorted(directory.glob('*.JPG'))
        self.assertEqual(len(paths), 5)
        def detect(pixels):
            message = CvBridge().cv2_to_imgmsg(pixels, 'bgr8')
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = 'battery_replay_optical_frame'
            stamp = message.header.stamp.to_nsec()
            publisher.publish(message)
            deadline = time.monotonic() + 180
            with condition:
                while stamp not in received and time.monotonic() < deadline:
                    condition.wait(timeout=1)
                self.assertIn(stamp, received, 'No matching detection frame before timeout')
                value = received[stamp]
            self.assertEqual(value['schema_version'], 1)
            self.assertEqual(value['frame_id'], message.header.frame_id)
            return value['detections']

        count = 0
        for path in paths:
            count += check_image(path, detect(cv2.imread(str(path))))
        self.assertEqual(count, 18)
        dark_frames = []
        for level in [0, 8, 16]:
            dark_frames.append({'gray_level': level, 'detections': detect(np.full((480, 640, 3), level, np.uint8))})
        square_confidence = rospy.get_param('~square_confidence', 0.25)
        if square_confidence >= 0.60:
            self.assertTrue(all(not frame['detections'] for frame in dark_frames), dark_frames)
        Path('/tmp/battery_grasp_yolo_ros.json').write_text(json.dumps(
            {'passed': True, 'images': 5, 'matched_objects': count, 'square_confidence': square_confidence,
             'dark_frames': dark_frames, 'scope': 'VM ROS image replay, no depth or robot'}))
        subscriber.unregister()


if __name__ == '__main__':
    rospy.init_node('verify_ros_yolo')
    rostest.rosrun('battery_grasp', 'yolo_image_replay', YoloRosTest)
