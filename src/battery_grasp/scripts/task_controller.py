#!/usr/bin/env python3
import json
import threading
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import PoseStamped, PoseArray
from battery_grasp.ros_support import config, decode, encode, pose, pose_array
from battery_grasp.task_control import TaskController


def main():
    rospy.init_node('task_controller')
    cfg = config()
    control = TaskController(cfg,rospy.Time.now().to_sec())
    lock = threading.RLock()
    publishers = {name:rospy.Publisher(topic,String,queue_size=20,latch=name in ('state','reset')) for name,topic in
                  [('state','/battery_grasp/state'),('reset','/battery_grasp/scene_reset'),
                   ('task','/sorting_task'),('sequence','/sorting_sequence'),('event','/grasp_state'),
                   ('result','/sorting_result'),('completion','/completed_battery_target')]}
    position_pub = rospy.Publisher('/current_grasp_pose',PoseStamped,queue_size=10)
    selected_pub = rospy.Publisher('/selected_battery_target',PoseStamped,queue_size=10)
    sequence_pub = rospy.Publisher('/grasp_sequence',PoseArray,queue_size=10)
    sorting_pub = rospy.Publisher('/sorting_state',String,queue_size=1,latch=True)

    def publish_state():
        value = encode(control.status(rospy.Time.now().to_sec()))
        publishers['state'].publish(value)
        sorting_pub.publish(value)

    def command(value):
        with lock:
            response,reset = control.command(value,rospy.Time.now().to_sec())
            if reset:
                publishers['reset'].publish(encode(reset))
            publish_state()
            return TriggerResponse(success=response['accepted'],message=json.dumps(response,allow_nan=False))

    def status(_):
        with lock:
            return TriggerResponse(success=True,message=json.dumps(control.status(rospy.Time.now().to_sec()),allow_nan=False))

    def observe(msg):
        with lock:
            try:
                control.observe(decode(msg))
            except (ValueError,KeyError,TypeError):
                control.observe(None)

    def scene_ready(msg):
        with lock:
            try:
                data = decode(msg)
                control.confirm_scene_reset(data['session_id'],data['stamp'])
            except (ValueError,KeyError,TypeError):
                pass

    def tick(_):
        with lock:
            output = control.tick(rospy.Time.now().to_sec())
            for name in ('task','sequence','event','result'):
                if output[name] is not None:
                    publishers[name].publish(encode(output[name]))
            if output['task']:
                msg = PoseStamped()
                msg.header.frame_id,msg.header.stamp = cfg['frame_id'],rospy.Time.now()
                msg.pose = pose(output['task']['target']['position'])
                selected_pub.publish(msg)
            if output['sequence']:
                sequence_pub.publish(pose_array([s['position'] for s in output['sequence']['steps']]))
            if output['event']:
                msg = PoseStamped()
                msg.header.frame_id,msg.header.stamp = cfg['frame_id'],rospy.Time.now()
                msg.pose = pose(output['event']['position'])
                position_pub.publish(msg)
            if output['result'] and output['result']['status']=='DONE':
                publishers['completion'].publish(encode(output['result']))
            publish_state()

    services = [rospy.Service('/battery_grasp/'+name.lower(),Trigger,lambda req,c=name:command(c)) for name in ('START','STOP','RESET','MODE_MANUAL','MODE_AUTO')]
    services.append(rospy.Service('/battery_grasp/status',Trigger,status))
    rospy.Subscriber('/battery_grasp/command',String,lambda msg:command(msg.data.strip().upper()),queue_size=10)
    rospy.Subscriber('/battery_objects',String,observe,queue_size=1)
    rospy.Subscriber('/battery_grasp/scene_ready',String,scene_ready,queue_size=1)
    publishers['reset'].publish(encode({'schema_version':1,'session_id':control.planner.session,'stamp':rospy.Time.now().to_sec()}))
    publish_state()
    rospy.Timer(rospy.Duration(0.05),tick)
    rospy.spin()


if __name__=='__main__':
    main()
