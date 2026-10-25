#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
from battery_grasp.ros_support import config, decode, encode, pose_array
from battery_grasp.sorting_core import build_sequence


def main():
    rospy.init_node('grasp_sequence_generator')
    cfg = config()
    pub = rospy.Publisher('/sorting_sequence',String,queue_size=1,latch=True)
    visual = rospy.Publisher('/grasp_sequence',PoseArray,queue_size=1)
    result_pub = rospy.Publisher('/sorting_result',String,queue_size=10)

    def callback(msg):
        task = None
        try:
            task = decode(msg)
            sequence = build_sequence(task,cfg)
            pub.publish(encode(sequence))
            visual.publish(pose_array([s['position'] for s in sequence['steps']]))
        except (ValueError,KeyError,TypeError,IndexError) as exc:
            rospy.logerr('Sequence rejected: %s',exc)
            if task and 'task_id' in task:
                result_pub.publish(encode({'schema_version':1,'task_id':task['task_id'],'status':'ERROR','reason':str(exc)}))

    rospy.Subscriber('/sorting_task',String,callback,queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    main()
