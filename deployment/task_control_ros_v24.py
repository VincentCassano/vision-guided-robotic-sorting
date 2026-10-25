#!/usr/bin/env python3
"""ROS services/topics proxy the same real web controller, never SimExecutor."""
import json
from urllib.request import Request, urlopen
import rospy
import rosservice
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import PoseStamped


def main():
    rospy.init_node('battery_real_task_control')
    endpoint=rospy.get_param('~endpoint','http://127.0.0.1:18081').rstrip('/')
    prefix=rospy.get_param('~prefix','/battery_grasp').rstrip('/')
    commands=('START','STOP','RESET','MODE_MANUAL','MODE_AUTO')
    existing=set(rosservice.get_service_list())
    names=[prefix+'/'+name.lower() for name in commands+('STATUS',)]
    if existing.intersection(names):
        raise RuntimeError('同名任务服务已运行；先退出仿真控制，不覆盖现有服务：'+str(existing.intersection(names)))
    state_pub=rospy.Publisher(prefix+'/state',String,queue_size=1,latch=True)
    result_pub=rospy.Publisher('/sorting_result',String,queue_size=10)
    sorting_pub=rospy.Publisher('/sorting_state',String,queue_size=1,latch=True)
    selected_pub=rospy.Publisher('/selected_battery_target',PoseStamped,queue_size=1)
    last=[None,0]

    def request(command=None):
        req=endpoint+'/api/status' if command is None else Request(endpoint+'/api/command',
            data=json.dumps(dict(command=command)).encode('utf8'),
            headers={'Content-Type':'application/json'},method='POST')
        # Commands are sent once. A timeout is not permission to resend motion.
        from urllib.error import HTTPError
        try:
            with urlopen(req,timeout=3) as response:
                return json.loads(response.read())
        except HTTPError as exc:
            return json.loads(exc.read())

    def service(command):
        try:
            value=request(None if command=='STATUS' else command)
            return TriggerResponse(success=(value.get('schema_version')==24 if command=='STATUS' else value.get('accepted') is True),
                message=json.dumps(value,ensure_ascii=False,allow_nan=False))
        except (OSError,ValueError) as exc:
            return TriggerResponse(success=False,message=json.dumps(dict(error=str(exc),
                command_not_retried=True,physical_stop_verified=False),ensure_ascii=False))

    services=[rospy.Service(prefix+'/'+name.lower(),Trigger,
        lambda req,c=name:service(c)) for name in commands+('STATUS',)]

    def publish(_):
        try:
            value=request()
            payload=json.dumps(value,ensure_ascii=False,allow_nan=False)
            state_pub.publish(payload)
            sorting_pub.publish(payload)
            session=value.get('runner_result')
            count=value.get('completed',0)
            if session!=last[0] or count<last[1]:
                last[:]=[session,0]
            if count>last[1]:
                result_pub.publish(json.dumps(dict(status='COMMAND_CYCLE_COMPLETED',
                    count=count,simulation_only=False,actual_pickup_and_correct_bin_verified=False)))
                last[1]=count
            point=(value.get('locked_target') or {}).get('top_center_world_m')
            if point and value.get('busy'):
                pose=PoseStamped()
                pose.header.frame_id='World'
                pose.header.stamp=rospy.Time.now()
                pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=point
                pose.pose.orientation.w=1.
                selected_pub.publish(pose)
        except (OSError,ValueError) as exc:
            rospy.logwarn_throttle(10,'Real task controller unavailable: %s',exc)

    def topic(msg):
        command=msg.data.strip().upper()
        if command not in commands:
            rospy.logwarn('Unknown real task command: %s',command)
            return
        value=service(command)
        if not value.success:
            rospy.logwarn('Real task command not accepted: %s',value.message)

    rospy.Subscriber(prefix+'/command',String,topic,queue_size=1)
    rospy.Timer(rospy.Duration(1),publish)
    rospy.loginfo('ROS real task control -> %s; no simulated executor',endpoint)
    rospy.spin()


if __name__=='__main__':
    main()
