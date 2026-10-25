#!/usr/bin/env python3
import threading
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from battery_grasp.rgbd import FrameMatcher, project_frame
from battery_grasp.ros_support import decode, encode


def main():
    rospy.init_node('vision_depth_test')
    bridge, matcher, lock = CvBridge(), FrameMatcher(), threading.RLock()
    publisher = rospy.Publisher('~objects', String, queue_size=1)

    def receive(kind, msg):
        stamp = 0
        try:
            value = decode(msg) if kind == 0 else msg
            stamp = value['stamp_ns'] if kind == 0 else msg.header.stamp.to_nsec()
            with lock:
                matched = matcher.add(kind, stamp, value)
                if matched is None:
                    return
                detection, depth, info = matched
                depth_data = {'stamp_ns': depth.header.stamp.to_nsec(), 'frame_id': depth.header.frame_id,
                              'encoding': depth.encoding, 'image': bridge.imgmsg_to_cv2(depth, 'passthrough')}
                camera = {'stamp_ns': info.header.stamp.to_nsec(), 'frame_id': info.header.frame_id,
                          'width': info.width, 'height': info.height, 'K': list(info.K), 'D': list(info.D)}
                publisher.publish(encode(project_frame(detection, depth_data, camera)))
        except (ValueError, KeyError, TypeError, CvBridgeError) as exc:
            publisher.publish(encode({'schema_version': 1, 'stamp_ns': stamp, 'simulation_only': True,
                                      'depth_source': 'synthetic_test', 'execution_ready': False,
                                      'valid': False, 'objects': [], 'rejected': [{'reason': str(exc)}]}))
            rospy.logwarn('RGB-D frame rejected: %s', exc)

    rospy.Subscriber('~detections', String, lambda m: receive(0, m), queue_size=2)
    rospy.Subscriber('~depth', Image, lambda m: receive(1, m), queue_size=2, buff_size=2**27)
    rospy.Subscriber('~camera_info', CameraInfo, lambda m: receive(2, m), queue_size=2)
    rospy.spin()


if __name__ == '__main__':
    main()
