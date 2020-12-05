#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import yaml
import numpy as np
import quaternion
import geometry_msgs.msg
import turtlesim.srv
from hrpsys_ros_bridge.msg import BoxPoses
from hrpsys_ros_bridge.msg import BoxPose
from ar_track_alvar_msgs.msg import AlvarMarkers
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

box_info_fname = rospy.get_param("/boxpose_pub/info_yaml", "../config/box_info.yaml")
marker_size = rospy.get_param("/ar_track_alvar/marker_size", 5.0) * 0.01 #cm -> m
marker_frame_id = rospy.get_param("/ar_track_alvar/output_frame", "/camera")
top_box_id = rospy.get_param("/boxpose_pub/top_box_id", 7)
base_box_id = rospy.get_param("/boxpose_pub/base_box_id", 8)
hold_box_id = rospy.get_param("/boxpose_pub/hold_box_id", 9)
look_box_mode = "box-balancer"

with open(box_info_fname) as yml:
    box_info = yaml.load(yml)
marker_to_box_dict = {}
for b in box_info:
    for m in box_info[b]['markers']:
        marker_to_box_dict[m] = b

rospy.init_node('boxpose_pub')
box_pose_pub = rospy.Publisher("box_pose", BoxPoses, queue_size = 1)
markers_pub = rospy.Publisher("markers", MarkerArray, queue_size = 1)
look_at_point_pub = rospy.Publisher("look_at_point", PointStamped, queue_size = 1)
rhand_pose_pub = rospy.Publisher("rhand_pose", PoseStamped, queue_size = 1)
lhand_pose_pub = rospy.Publisher("lhand_pose", PoseStamped, queue_size = 1)

box_dict = {}

class MarkerData:
    def __init__(self, m, pos, rot):
        self.marker_data = m
        self.box_pos_from_marker = pos
        self.box_rot_from_marker = rot
        self.probability = 1.0

class BoxData:
    def __init__(self):
        self.box_pose_data = BoxPose()
        self.box_marker_data = Marker()
        self.markers_data = {}
        self.probability = 1.0

rate = rospy.Rate(30.0)
def callback(msg):
    for m in msg.markers:
        if m.id in marker_to_box_dict.keys():
            marker = Marker()
            #marker.header.frame_id = marker_frame_id.lstrip()
            #marker.header.stamp = rospy.Time.now()
            marker.header = m.header
            marker.ns = "marker"
            marker.id = m.id
            marker.pose = m.pose.pose
            m_pos = np.array([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z])
            m_quat = np.quaternion(marker.pose.orientation.w, marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z)
            m_new_pos = m_pos + np.dot(quaternion.as_rotation_matrix(m_quat), np.array([0, 0, marker_size * 0.01]))
            marker.pose.position.x = m_new_pos[0]
            marker.pose.position.y = m_new_pos[1]
            marker.pose.position.z = m_new_pos[2]
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.5
            marker.scale.x = marker_size
            marker.scale.y = marker_size
            marker.scale.z = marker_size * 0.02;
            marker.lifetime = rospy.Duration()
            marker.type = 1

            delay = rospy.Time.now() - m.header.stamp
            rospy.loginfo("marker_id: " + str(m.id) + " delay: " + str(delay.secs * 1000 + delay.nsecs / 1000000) + "ms")

            b_pos = np.array([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z])
            b_rot = np.dot(quaternion.as_rotation_matrix(np.quaternion(m.pose.pose.orientation.w, m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z)), np.array(box_info[marker_to_box_dict[m.id]]['markers'][m.id]['rot']).T)
            b_pos = m_pos + np.dot(b_rot, -np.array(box_info[marker_to_box_dict[m.id]]['markers'][m.id]['pos']))

            if not marker_to_box_dict[m.id] in box_dict:
                box_dict[marker_to_box_dict[m.id]] = BoxData()
            box_dict[marker_to_box_dict[m.id]].box_pose_data.header = m.header
            box_dict[marker_to_box_dict[m.id]].box_marker_data.header = m.header
            box_dict[marker_to_box_dict[m.id]].markers_data[m.id] = MarkerData(marker, b_pos, b_rot)
            box_dict[marker_to_box_dict[m.id]].probability = 1.0



    for b in box_dict:
        if len(box_dict[b].markers_data) != 0:
            pos = np.array([0, 0, 0])
            rot = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
            probability_sum = 0
            for m in box_dict[b].markers_data:
                pos = pos + box_dict[b].markers_data[m].probability * box_dict[b].markers_data[m].box_pos_from_marker
                rot = rot + box_dict[b].markers_data[m].probability * box_dict[b].markers_data[m].box_rot_from_marker
                probability_sum = probability_sum + box_dict[b].markers_data[m].probability
            pos = pos / probability_sum
            rot = rot / probability_sum
            quat = quaternion.from_rotation_matrix(rot, nonorthogonal=True)

            box_dict[b].box_pose_data.px = pos[0]
            box_dict[b].box_pose_data.py = pos[1]
            box_dict[b].box_pose_data.pz = pos[2]
            box_dict[b].box_pose_data.rx = quat.x
            box_dict[b].box_pose_data.ry = quat.y
            box_dict[b].box_pose_data.rz = quat.z
            box_dict[b].box_pose_data.rw = quat.w
            box_dict[b].box_pose_data.id = b


            box_dict[b].box_marker_data.ns = "box"
            box_dict[b].box_marker_data.id = b
            box_dict[b].box_marker_data.pose.position.x = pos[0]
            box_dict[b].box_marker_data.pose.position.y = pos[1]
            box_dict[b].box_marker_data.pose.position.z = pos[2]
            box_dict[b].box_marker_data.pose.orientation.x = quat.x
            box_dict[b].box_marker_data.pose.orientation.y = quat.y
            box_dict[b].box_marker_data.pose.orientation.z = quat.z
            box_dict[b].box_marker_data.pose.orientation.w = quat.w
            box_dict[b].box_marker_data.color.r = 1.0
            box_dict[b].box_marker_data.color.g = 1.0
            box_dict[b].box_marker_data.color.b = 0.0
            box_dict[b].box_marker_data.color.a = 0.5
            box_dict[b].box_marker_data.scale.x = box_info[b]['size'][0]
            box_dict[b].box_marker_data.scale.y = box_info[b]['size'][1]
            box_dict[b].box_marker_data.scale.z = box_info[b]['size'][2]
            box_dict[b].box_marker_data.lifetime = rospy.Duration()
            box_dict[b].box_marker_data.type = 1


    box_poses_data = BoxPoses()
    box_poses_data.existence = False
    box_poses_data.header.stamp = rospy.Time.now()
    markers_data = MarkerArray()
    box_del_list = []
    for b in box_dict:
        marker_del_list = []
        for m in box_dict[b].markers_data:
            if box_dict[b].markers_data[m].probability > 0:
                box_dict[b].markers_data[m].marker_data.action = Marker.ADD
                markers_data.markers.append(box_dict[b].markers_data[m].marker_data)
                box_dict[b].markers_data[m].probability -= 0.7
            else:
                box_dict[b].markers_data[m].marker_data.action = Marker.DELETE
                markers_data.markers.append(box_dict[b].markers_data[m].marker_data)
                marker_del_list.append(m)
        for m in marker_del_list:
            box_dict[b].markers_data.pop(m)
        if box_dict[b].probability > 0:
            box_dict[b].box_marker_data.action = Marker.ADD
            box_poses_data.existence = True
            box_poses_data.header.stamp = box_dict[b].box_pose_data.header.stamp
            box_poses_data.poses.append(box_dict[b].box_pose_data)
            br = tf.TransformBroadcaster()
            br.sendTransform(
                (box_dict[b].box_marker_data.pose.position.x,
                 box_dict[b].box_marker_data.pose.position.y,
                 box_dict[b].box_marker_data.pose.position.z),
                (box_dict[b].box_marker_data.pose.orientation.x,
                 box_dict[b].box_marker_data.pose.orientation.y,
                 box_dict[b].box_marker_data.pose.orientation.z,
                 box_dict[b].box_marker_data.pose.orientation.w),
                rospy.Time.now(), "box"+str(b), marker_frame_id.lstrip())
            markers_data.markers.append(box_dict[b].box_marker_data)
            box_dict[b].probability -= 0.7
        else:
            box_dict[b].box_marker_data.action = Marker.DELETE
            markers_data.markers.append(box_dict[b].box_marker_data)
            box_del_list.append(b)
    for b in box_del_list:
        box_dict.pop(b)
    markers_pub.publish(markers_data)
    box_pose_pub.publish(box_poses_data)

    point_data = PointStamped()
    point_data.header.stamp = rospy.Time.now()
    if look_box_mode == "lift-box":
        if hold_box_id in box_dict:
            point_data.point.x = box_dict[hold_box_id].markers_data[list(box_dict[hold_box_id].markers_data)[0]].marker_data.pose.position.x
            point_data.point.y = box_dict[hold_box_id].markers_data[list(box_dict[hold_box_id].markers_data)[0]].marker_data.pose.position.y
            point_data.point.z = box_dict[hold_box_id].markers_data[list(box_dict[hold_box_id].markers_data)[0]].marker_data.pose.position.z
        elif base_box_id in box_dict:
            pos = np.array([box_dict[base_box_id].box_pose_data.px, box_dict[base_box_id].box_pose_data.py, box_dict[base_box_id].box_pose_data.pz])
            rot = quaternion.as_rotation_matrix(np.quaternion(box_dict[base_box_id].box_pose_data.rw, box_dict[base_box_id].box_pose_data.rx, box_dict[base_box_id].box_pose_data.ry, box_dict[base_box_id].box_pose_data.rz))
            pos = pos + np.dot(rot, np.array([-box_info[base_box_id]['size'][0]/2.0, 0, -box_info[base_box_id]['size'][2]/2.0]))
            point_data.point.x = pos[0]
            point_data.point.y = pos[1]
            point_data.point.z = pos[2]
        elif top_box_id in box_dict:
            pos = np.array([box_dict[top_box_id].box_pose_data.px, box_dict[top_box_id].box_pose_data.py, box_dict[top_box_id].box_pose_data.pz])
            rot = quaternion.as_rotation_matrix(np.quaternion(box_dict[top_box_id].box_pose_data.rw, box_dict[top_box_id].box_pose_data.rx, box_dict[top_box_id].box_pose_data.ry, box_dict[top_box_id].box_pose_data.rz))
            pos = pos + np.dot(rot, np.array([-box_info[top_box_id]['size'][0]/2.0, 0, -box_info[top_box_id]['size'][2]/2.0]))
            point_data.point.x = pos[0]
            point_data.point.y = pos[1]
            point_data.point.z = pos[2]
        else:
            point_data.point.x = 0.0
            point_data.point.y = 0.0
            point_data.point.z = 0.0
    elif look_box_mode == "box-balancer":
        #if top_box_id in box_dict and base_box_id in box_dict:
        #    point_data.point.x = (box_dict[top_box_id].box_pose_data.px + box_dict[base_box_id].box_pose_data.px) / 2.0
        #    point_data.point.y = (box_dict[top_box_id].box_pose_data.py + box_dict[base_box_id].box_pose_data.py) / 2.0
        #    point_data.point.z = (box_dict[top_box_id].box_pose_data.pz + box_dict[base_box_id].box_pose_data.pz) / 2.0
        #el
        if top_box_id in box_dict:
            pos = np.array([box_dict[top_box_id].box_pose_data.px, box_dict[top_box_id].box_pose_data.py, box_dict[top_box_id].box_pose_data.pz])
            rot = quaternion.as_rotation_matrix(np.quaternion(box_dict[top_box_id].box_pose_data.rw, box_dict[top_box_id].box_pose_data.rx, box_dict[top_box_id].box_pose_data.ry, box_dict[top_box_id].box_pose_data.rz))
            pos = pos + np.dot(rot, np.array([-box_info[top_box_id]['size'][0]/2.0, 0, -box_info[top_box_id]['size'][2]/2.0]))
            point_data.point.x = pos[0]
            point_data.point.y = pos[1]
            point_data.point.z = pos[2]
        elif base_box_id in box_dict:
            pos = np.array([box_dict[base_box_id].box_pose_data.px, box_dict[base_box_id].box_pose_data.py, box_dict[base_box_id].box_pose_data.pz])
            rot = quaternion.as_rotation_matrix(np.quaternion(box_dict[base_box_id].box_pose_data.rw, box_dict[base_box_id].box_pose_data.rx, box_dict[base_box_id].box_pose_data.ry, box_dict[base_box_id].box_pose_data.rz))
            pos = pos + np.dot(rot, np.array([-box_info[base_box_id]['size'][0]/2.0, 0, box_info[base_box_id]['size'][2]/2.0]))
            point_data.point.x = pos[0]
            point_data.point.y = pos[1]
            point_data.point.z = pos[2]
        elif hold_box_id in box_dict:
            pos = np.array([box_dict[hold_box_id].box_pose_data.px, box_dict[hold_box_id].box_pose_data.py, box_dict[hold_box_id].box_pose_data.pz])
            rot = quaternion.as_rotation_matrix(np.quaternion(box_dict[hold_box_id].box_pose_data.rw, box_dict[hold_box_id].box_pose_data.rx, box_dict[hold_box_id].box_pose_data.ry, box_dict[hold_box_id].box_pose_data.rz))
            pos = pos + np.dot(rot, np.array([-box_info[hold_box_id]['size'][0]/2.0, 0, box_info[hold_box_id]['size'][2]/2.0]))
            point_data.point.x = pos[0]
            point_data.point.y = pos[1]
            point_data.point.z = pos[2]
        else:
            point_data.point.x = 0.0
            point_data.point.y = 0.0
            point_data.point.z = 0.0
    look_at_point_pub.publish(point_data)

    if hold_box_id in box_dict:
        rpose_data = PoseStamped()
        lpose_data = PoseStamped()
        pos = np.array([box_dict[hold_box_id].box_pose_data.px, box_dict[hold_box_id].box_pose_data.py, box_dict[hold_box_id].box_pose_data.pz])
        rot = quaternion.as_rotation_matrix(np.quaternion(box_dict[hold_box_id].box_pose_data.rw, box_dict[hold_box_id].box_pose_data.rx, box_dict[hold_box_id].box_pose_data.ry, box_dict[hold_box_id].box_pose_data.rz))
        rpos = pos + np.dot(rot, np.array(box_info[hold_box_id]['rhand_pose']['pos']))
        lpos = pos + np.dot(rot, np.array(box_info[hold_box_id]['lhand_pose']['pos']))
        rrot = np.dot(rot, np.array(box_info[hold_box_id]['rhand_pose']['rot']))
        lrot = np.dot(rot, np.array(box_info[hold_box_id]['lhand_pose']['rot']))
        rquat = quaternion.from_rotation_matrix(rrot, nonorthogonal=True)
        lquat = quaternion.from_rotation_matrix(lrot, nonorthogonal=True)
        rpose_data.pose.position.x = rpos[0]
        rpose_data.pose.position.y = rpos[1]
        rpose_data.pose.position.z = rpos[2]
        rpose_data.pose.orientation.x = rquat.x
        rpose_data.pose.orientation.y = rquat.y
        rpose_data.pose.orientation.z = rquat.z
        rpose_data.pose.orientation.w = rquat.w
        lpose_data.pose.position.x = lpos[0]
        lpose_data.pose.position.y = lpos[1]
        lpose_data.pose.position.z = lpos[2]
        lpose_data.pose.orientation.x = lquat.x
        lpose_data.pose.orientation.y = lquat.y
        lpose_data.pose.orientation.z = lquat.z
        lpose_data.pose.orientation.w = lquat.w
        rhand_pose_pub.publish(rpose_data)
        lhand_pose_pub.publish(lpose_data)
        br = tf.TransformBroadcaster()
        br.sendTransform(
            (rpos[0], rpos[1], rpos[2]),
            (rquat.x, rquat.y, rquat.z, rquat.w),
            rospy.Time.now(), "rhand_pose", marker_frame_id.lstrip())
        br.sendTransform(
            (lpos[0], lpos[1], lpos[2]),
            (lquat.x, lquat.y, lquat.z, lquat.w),
            rospy.Time.now(), "lhand_pose", marker_frame_id.lstrip())

    rate.sleep()

def mode_cb(msg):
    global look_box_mode
    look_box_mode = msg.data
    print(look_box_mode)
    
rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)
rospy.Subscriber("look_box_mode", String, mode_cb)
rospy.spin()
