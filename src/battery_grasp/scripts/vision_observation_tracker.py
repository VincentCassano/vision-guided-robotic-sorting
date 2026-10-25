#!/usr/bin/env python3
import threading
import rospy
from std_msgs.msg import String
from battery_grasp.observation_tracking import ObservationTracker
from battery_grasp.ros_support import decode, encode


def main():
    rospy.init_node('vision_observation_tracker')
    camera = rospy.get_param('/battery_grasp_sim/camera')
    tracker = ObservationTracker(camera['rotation'], camera['translation'],
                                 source_frame=rospy.get_param('~source_frame', 'battery_replay_optical_frame'),
                                 gate=rospy.get_param('~association_gate', 0.02),
                                 max_gap=rospy.get_param('~max_gap', 1.0),
                                 max_age=rospy.get_param('~max_age', 10.0))
    lock = threading.Lock()
    publisher = rospy.Publisher('~observations', String, queue_size=1)

    def receive(msg):
        with lock:
            try:
                result = tracker.process(decode(msg), rospy.Time.now().to_nsec())
            except (ValueError, KeyError, TypeError) as exc:
                tracker.invalidate()
                result = {'schema_version': 1, 'stamp_ns': 0, 'frame_id': 'base_link',
                          'simulation_only': True, 'depth_source': 'synthetic_test',
                          'execution_ready': False, 'valid': False, 'objects': [],
                          'rejected': [{'reason': str(exc)}]}
                rospy.logwarn('Observation rejected: %s', exc)
            publisher.publish(encode(result))

    rospy.Subscriber('~objects', String, receive, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    main()
