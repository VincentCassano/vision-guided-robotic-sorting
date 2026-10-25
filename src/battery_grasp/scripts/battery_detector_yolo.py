#!/usr/bin/env python3
import json
import threading
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from battery_grasp.inference_client import InferenceClient
from battery_grasp.yolo_inference import detection_message


def main():
    rospy.init_node('battery_detector_yolo')
    class_confidences = rospy.get_param('~class_confidences', None)
    square_confidence = rospy.get_param('~square_confidence', 0.25)
    # A supplied square override affects only class 3, even with a four-class profile.
    if class_confidences is not None:
        if not isinstance(class_confidences, list) or len(class_confidences) != 4:
            raise ValueError('class_confidences must be a YAML list of four values')
        class_confidences = list(map(float, class_confidences))
        if rospy.has_param('~square_confidence'):
            class_confidences[3] = float(square_confidence)
    effective_confidences = class_confidences if class_confidences is not None else [0.25, 0.25, 0.25, float(square_confidence)]
    if any(not 0.25 <= value < 1.0 for value in effective_confidences):
        raise ValueError('Class thresholds must be at least 0.25 and below one')
    rospy.loginfo('YOLO thresholds (black/red/white/square): %s', effective_confidences)
    client = InferenceClient(rospy.get_param('~inference_python'), rospy.get_param('~worker'),
                             rospy.get_param('~repository'), rospy.get_param('~weights'),
                             rospy.get_param('~device', 'cpu'), rospy.get_param('~timeout', 120),
                             square_confidence=square_confidence,
                             class_confidences=effective_confidences)
    rospy.on_shutdown(client.close)
    bridge, lock = CvBridge(), threading.Lock()
    output = rospy.Publisher('~detections', String, queue_size=1)

    def callback(msg):
        if rospy.is_shutdown():
            return
        if not lock.acquire(False):
            return
        try:
            detections = client.detect(bridge.imgmsg_to_cv2(msg, 'bgr8'))
            value = detection_message(detections, msg.header.stamp.to_nsec(), msg.header.frame_id, msg.width, msg.height)
            output.publish(String(data=json.dumps(value, allow_nan=False)))
        except Exception as exc:
            if not rospy.is_shutdown():
                rospy.logerr('YOLO inference failed: %s', exc)
                rospy.signal_shutdown('Inference failure')
        finally:
            lock.release()

    rospy.Subscriber('~image', Image, callback, queue_size=1, buff_size=2**26)
    rospy.spin()


if __name__ == '__main__':
    main()
