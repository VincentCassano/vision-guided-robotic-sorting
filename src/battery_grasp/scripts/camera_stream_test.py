#!/usr/bin/env python3
import threading
import math
import cv2
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from battery_grasp.perception import render_scene
from battery_grasp.ros_support import config, decode, encode


def main():
    rospy.init_node('camera_stream_test')
    cfg,bridge = config(),CvBridge()
    photo_directory = rospy.get_param('~photo_directory','')
    if photo_directory:
        from battery_grasp.photo_scene import PhotoScene
        renderer = PhotoScene(photo_directory).render
        rospy.logwarn('Photo-cutout simulation: synthetic scene, depth and removal feedback')
    else:
        renderer = render_scene
    rotation,_ = cv2.Rodrigues(np.array(cfg['camera']['rotation'],dtype=float))
    angle = float(np.linalg.norm(rotation))
    q = rotation.flatten()*math.sin(angle/2)/angle if angle else np.zeros(3)
    transform = TransformStamped()
    transform.header.frame_id,transform.child_frame_id = cfg['frame_id'],cfg['camera']['frame_id']
    transform.header.stamp = rospy.Time.now()
    transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z = cfg['camera']['translation']
    transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z = q
    transform.transform.rotation.w = math.cos(angle/2)
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(transform)
    completed,lock = set(),threading.RLock()
    session = [None]
    rgb_pub = rospy.Publisher('/camera/color/image_raw',Image,queue_size=1)
    depth_pub = rospy.Publisher('/camera/depth/image_raw',Image,queue_size=1)
    info_pub = rospy.Publisher('/camera/color/camera_info',CameraInfo,queue_size=1)
    ready_pub = rospy.Publisher('/battery_grasp/scene_ready',String,queue_size=1,latch=True)

    def finish(msg):
        try:
            event = decode(msg)
            if event['status'] == 'DONE':
                with lock:
                    if session[0] is None or event['task_id'].split(':')[0]==session[0]:
                        completed.add(event['target_id'])
        except (ValueError,KeyError,TypeError) as exc:
            rospy.logwarn('Rejected camera completion: %s',exc)

    def reset(msg):
        try:
            data = decode(msg)
            with lock:
                if data['session_id']!=session[0]:
                    session[0] = data['session_id']
                    completed.clear()
        except (ValueError,KeyError,TypeError) as exc:
            rospy.logwarn('Rejected scene reset: %s',exc)

    rospy.Subscriber('/completed_battery_target',String,finish,queue_size=10)
    rospy.Subscriber('/battery_grasp/scene_reset',String,reset,queue_size=10)
    rate = rospy.Rate(rospy.get_param('~rate',10))
    while not rospy.is_shutdown():
        with lock:
            rgb,depth = renderer(cfg,completed)
            frame_session = session[0]
        stamp = rospy.Time.now()
        rgb_msg,depth_msg = bridge.cv2_to_imgmsg(rgb,'bgr8'),bridge.cv2_to_imgmsg(depth,'32FC1')
        info = CameraInfo()
        info.width,info.height = 640,480
        fx,fy,cx,cy = cfg['camera']['intrinsics']
        info.K = [fx,0,cx,0,fy,cy,0,0,1]
        info.P = [fx,0,cx,0,0,fy,cy,0,0,0,1,0]
        for msg in (rgb_msg,depth_msg,info):
            msg.header.stamp,msg.header.frame_id = stamp,cfg['camera']['frame_id']
        info_pub.publish(info)
        depth_pub.publish(depth_msg)
        rgb_pub.publish(rgb_msg)
        if frame_session is not None:
            ready_pub.publish(encode({'schema_version':1,'session_id':frame_session,'stamp':stamp.to_sec()}))
        rate.sleep()


if __name__ == '__main__':
    main()
