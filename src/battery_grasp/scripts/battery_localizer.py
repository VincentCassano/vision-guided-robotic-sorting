#!/usr/bin/env python3
import threading
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import MarkerArray
from battery_grasp.perception import localize
from battery_grasp.ros_support import config, decode, encode, markers, pose_array
from battery_grasp.sorting_core import require


def main():
    rospy.init_node('battery_localizer')
    cfg,bridge = config(),CvBridge()
    buffers = ({},{},{})
    lock = threading.RLock()
    last_stamp = [-1]
    pub = rospy.Publisher('/battery_objects',String,queue_size=1)
    poses = rospy.Publisher('/battery_targets',PoseArray,queue_size=1)
    visuals = rospy.Publisher('/battery_localized_markers',MarkerArray,queue_size=1)

    def receive(kind,msg):
        try:
            value = decode(msg) if kind == 0 else msg
            stamp = value['stamp_ns'] if kind == 0 else msg.header.stamp.to_nsec()
            with lock:
                buffers[kind][stamp] = value
                for buffer in buffers:
                    for old in sorted(buffer)[:-20]:
                        del buffer[old]
                if stamp <= last_stamp[0] or not all(stamp in b for b in buffers):
                    return
                detection,depth,info = [b.pop(stamp) for b in buffers]
                last_stamp[0] = stamp
                require(detection['frame_id'] == depth.header.frame_id == info.header.frame_id == cfg['camera']['frame_id'],'Frame mismatch')
                require(depth.encoding == '32FC1','Simulation requires metre-valued 32FC1 depth')
                depth_image = bridge.imgmsg_to_cv2(depth,'32FC1')
                require(depth_image.shape == (info.height,info.width),'Depth/CameraInfo size mismatch')
                intrinsics = [info.K[0],info.K[4],info.K[2],info.K[5]]
                objects,rejected = localize(detection['detections'],depth_image,intrinsics,cfg)
                pub.publish(encode({'schema_version':1,'frame_id':cfg['frame_id'],'stamp':detection['stamp'],
                                    'objects':objects,'valid':not rejected,'rejected':rejected}))
                poses.publish(pose_array([o['position'] for o in objects]))
                visuals.publish(markers(objects))
        except (ValueError,KeyError,TypeError) as exc:
            rospy.logwarn_throttle(1.0,'Localization rejected: %s',exc)

    rospy.Subscriber('/battery_pixels',String,lambda m:receive(0,m),queue_size=2)
    rospy.Subscriber('/camera/depth/image_raw',Image,lambda m:receive(1,m),queue_size=2,buff_size=2**24)
    rospy.Subscriber('/camera/color/camera_info',CameraInfo,lambda m:receive(2,m),queue_size=2)
    rospy.spin()


if __name__ == '__main__':
    main()
