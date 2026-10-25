#!/usr/bin/env python3
import threading
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from battery_grasp.ros_support import config, decode, encode, pose
from battery_grasp.sorting_core import Planner, require


def main():
    rospy.init_node('sorting_planner')
    cfg = config()
    planner,lock = Planner(cfg),threading.RLock()
    task_pub = rospy.Publisher('/sorting_task',String,queue_size=1,latch=True)
    state_pub = rospy.Publisher('/sorting_state',String,queue_size=1,latch=True)
    completion_pub = rospy.Publisher('/completed_battery_target',String,queue_size=10,latch=True)
    selected_pub = rospy.Publisher('/selected_battery_target',PoseStamped,queue_size=1)
    latest = [None]

    def receive(msg):
        with lock:
            try:
                data = decode(msg)
                require(data['frame_id'] == cfg['frame_id'] and data['valid'] is True,'Invalid perception frame')
                latest[0] = data
            except (ValueError,KeyError,TypeError) as exc:
                latest[0] = None
                planner.status,planner.reason = 'BLOCKED',str(exc)

    def finish(msg):
        with lock:
            try:
                result = decode(msg)
                if planner.finish(result):
                    completion_pub.publish(encode(result))
            except (ValueError,KeyError,TypeError) as exc:
                planner.failed = True
                planner.status,planner.reason = 'ERROR',str(exc)

    def tick(_):
        with lock:
            if latest[0] is not None and not planner.failed:
                data = latest[0]
                task = planner.observe(data['objects'],data['stamp'],rospy.Time.now().to_sec())
                if task:
                    selected = PoseStamped()
                    selected.header.frame_id,selected.header.stamp = cfg['frame_id'],rospy.Time.now()
                    selected.pose = pose(task['target']['position'])
                    selected_pub.publish(selected)
            if planner.current is not None and not planner.failed:
                task_pub.publish(encode(planner.current))
            state_pub.publish(encode({'schema_version':1,'state':planner.status,'reason':planner.reason,
                                      'completed':sorted(planner.completed),'total':9}))

    rospy.Subscriber('/battery_objects',String,receive,queue_size=1)
    rospy.Subscriber('/sorting_result',String,finish,queue_size=10)
    rospy.Timer(rospy.Duration(0.2),tick)
    rospy.spin()


if __name__ == '__main__':
    main()
