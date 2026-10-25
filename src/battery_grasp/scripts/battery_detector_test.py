#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from battery_grasp.perception import detect_batteries
from battery_grasp.ros_support import config, encode
from battery_grasp.sorting_core import CLASSES


def main():
    rospy.init_node('battery_detector_test')
    config()
    bridge = CvBridge()
    publisher = rospy.Publisher('/battery_pixels',String,queue_size=1)

    def callback(msg):
        detections = detect_batteries(bridge.imgmsg_to_cv2(msg,'bgr8'))
        for item in detections:
            item.update(class_id=CLASSES.index(item['category']), grasp_supported=True)
        publisher.publish(encode({'schema_version':1,'stamp':msg.header.stamp.to_sec(),
                                  'stamp_ns':msg.header.stamp.to_nsec(),'frame_id':msg.header.frame_id,
                                  'image_width':msg.width,'image_height':msg.height,'detections':detections}))

    rospy.Subscriber('/camera/color/image_raw',Image,callback,queue_size=1,buff_size=2**24)
    rospy.spin()


if __name__ == '__main__':
    main()
