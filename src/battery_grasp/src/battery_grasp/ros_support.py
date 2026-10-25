"""Versioned JSON transport and RViz helpers; no robot control interfaces."""
import json
import rospy
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from .sorting_core import VERSION, UnsafePlan, require, validate_config


def config():
    cfg = rospy.get_param('/battery_grasp_sim')
    validate_config(cfg)
    return cfg


def encode(value):
    return String(data=json.dumps(value,allow_nan=False))


def decode(msg):
    value = json.loads(msg.data)
    require(isinstance(value,dict) and type(value.get('schema_version')) is int and value['schema_version'] == VERSION,'Unsupported message schema')
    return value


def pose(position):
    p = Pose()
    p.position.x,p.position.y,p.position.z = position
    p.orientation.w = 1.0
    return p


def markers(objects, frame='base_link'):
    result = MarkerArray()
    clear = Marker()
    clear.action = Marker.DELETEALL
    result.markers.append(clear)
    colors = {'NORMAL_BLACK':(0.15,0.15,0.15),'DEFECT_RED':(1.0,0.1,0.1),'DEFECT_WHITE':(0.95,0.95,0.95)}
    for i,obj in enumerate(objects):
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = rospy.Time.now()
        m.ns,m.id = 'batteries',i
        m.type,m.action = Marker.CYLINDER,Marker.ADD
        m.pose = pose(obj['position'])
        m.scale.x,m.scale.y,m.scale.z = 0.04,0.04,0.08
        m.color.r,m.color.g,m.color.b = colors[obj['category']]
        m.color.a = 1.0
        result.markers.append(m)
    return result


def pose_array(positions,frame='base_link'):
    result = PoseArray()
    result.header.frame_id = frame
    result.header.stamp = rospy.Time.now()
    result.poses = [pose(p) for p in positions]
    return result
