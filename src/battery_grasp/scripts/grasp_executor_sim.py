#!/usr/bin/env python3
"""Simulated poses only. No hardware, MoveIt, or SDK output."""
import threading
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from battery_grasp.ros_support import config, decode, encode, pose
from battery_grasp.sorting_core import SimExecutor


def main():
    rospy.init_node('grasp_executor_sim')
    cfg = config()
    executor,lock = SimExecutor(cfg),threading.RLock()
    result_pub = rospy.Publisher('/sorting_result',String,queue_size=10,latch=True)
    pose_pub = rospy.Publisher('/current_grasp_pose',PoseStamped,queue_size=10)
    event_pub = rospy.Publisher('/grasp_state',String,queue_size=10,latch=True)

    def observe(msg):
        with lock:
            try:
                executor.observe(decode(msg))
            except (ValueError,KeyError,TypeError):
                executor.observe(None)

    def receive(msg):
        with lock:
            sequence = None
            try:
                sequence = decode(msg)
                executor.accept(sequence,rospy.Time.now().to_sec())
            except (ValueError,KeyError,TypeError,IndexError) as exc:
                if sequence is not None and 'task_id' in sequence:
                    result_pub.publish(encode(executor.fail(sequence,exc)))
                rospy.logerr('Simulated execution rejected: %s',exc)

    def tick(_):
        with lock:
            event,result = executor.tick(rospy.Time.now().to_sec())
            if event is not None:
                message = PoseStamped()
                message.header.frame_id,message.header.stamp = cfg['frame_id'],rospy.Time.now()
                message.pose = pose(event['position'])
                pose_pub.publish(message)
                event_pub.publish(encode(event))
            if result is not None:
                result_pub.publish(encode(result))

    rospy.Subscriber('/battery_objects',String,observe,queue_size=1)
    rospy.Subscriber('/sorting_sequence',String,receive,queue_size=1)
    rospy.Timer(rospy.Duration(cfg['step_delay']),tick)
    rospy.spin()


if __name__ == '__main__':
    main()
