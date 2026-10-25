#!/usr/bin/env python3
import threading
import rospy
from std_msgs.msg import String
from battery_grasp.observation_adapter import ObservationAdapter
from battery_grasp.ros_support import config, decode, encode


def main():
    rospy.init_node('observation_to_sim_scene')
    adapter = ObservationAdapter(config())
    lock = threading.RLock()
    session = [None]
    publisher = rospy.Publisher('~scene', String, queue_size=1)

    def reset(msg):
        with lock:
            data = decode(msg)
            if data['session_id'] != session[0]:
                adapter.reset(int(round(data['stamp']*1e9)))
                session[0] = data['session_id']

    def receive(msg):
        with lock:
            try:
                frame = decode(msg) if session[0] is not None else None
            except (ValueError, KeyError, TypeError):
                frame = None
            output = adapter.process(frame, rospy.Time.now().to_nsec())
            output['session_id'] = session[0]
            publisher.publish(encode(output))

    rospy.Subscriber('/battery_grasp/scene_reset', String, reset, queue_size=10)
    rospy.Subscriber('~observations', String, receive, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    main()
