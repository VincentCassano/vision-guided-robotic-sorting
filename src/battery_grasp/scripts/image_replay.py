#!/usr/bin/env python3
from pathlib import Path
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo


def main():
    rospy.init_node('battery_image_replay')
    directory = Path(rospy.get_param('~directory'))
    images = sorted(p for p in directory.iterdir() if p.suffix.lower() in ('.jpg', '.jpeg', '.png'))
    if not images:
        raise ValueError('Replay directory contains no images')
    publisher = rospy.Publisher('~image', Image, queue_size=1, latch=True)
    synthetic_depth = rospy.get_param('~synthetic_depth', False)
    if synthetic_depth:
        depth_pub = rospy.Publisher('~depth', Image, queue_size=1, latch=True)
        info_pub = rospy.Publisher('~camera_info', CameraInfo, queue_size=1, latch=True)
        rospy.logwarn('Diagnostic replay: constant synthetic depth and synthetic intrinsics; no real 3D measurements')
    bridge, rate = CvBridge(), rospy.Rate(rospy.get_param('~rate', 0.2))
    sequence = 0
    while not rospy.is_shutdown():
        path = images[sequence % len(images)]
        bgr = cv2.imread(str(path))
        if bgr is None:
            raise ValueError('Cannot read ' + str(path))
        message = bridge.cv2_to_imgmsg(bgr, 'bgr8')
        message.header.seq = sequence
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = 'battery_replay_optical_frame'
        publisher.publish(message)
        if synthetic_depth:
            height, width = bgr.shape[:2]
            depth = bridge.cv2_to_imgmsg(np.full((height, width), 800, np.uint16), '16UC1')
            depth.header = message.header
            info = CameraInfo()
            info.header = message.header
            info.width, info.height = width, height
            info.K = [float(width), 0., width/2., 0., float(width), height/2., 0., 0., 1.]
            info.D = [0.] * 5
            depth_pub.publish(depth)
            info_pub.publish(info)
        rospy.loginfo('Replay image: %s', path.name)
        sequence += 1
        rate.sleep()


if __name__ == '__main__':
    main()
