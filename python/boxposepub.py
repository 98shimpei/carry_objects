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

box_info_fname = rospy.get_param("/boxpose_pub/info_yaml", "../config/box_info.yaml")
marker_size = rospy.get_param("/ar_track_alvar/marker_size", 5.0) * 0.01 #cm -> m
marker_id = rospy.get_param("/ar_track_alvar/output_frame", "/camera")

with open(box_info_fname) as yml:
    box_info = yaml.load(yml)

rospy.init_node('boxpose_pub')
box_pose_pub = rospy.Publisher("box_pose", BoxPoses, queue_size = 1)
markers_pub = rospy.Publisher("markers", MarkerArray, queue_size = 1)
listener = tf.TransformListener()


marker_dict = {}

class MarkerData:
    def __init__(self, b, m):
        self.box_pose_data = b
        self.marker_data = m
        self.probability = 1.0

rate = rospy.Rate(20.0)
def callback(msg):
    for m in msg.markers:
        if m.id in box_info:
            box_pose = BoxPose()
            box_pose.header = m.header
            m_pos = np.array([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z])
            m_quat = np.quaternion(m.pose.pose.orientation.w, m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z)
            m_new_pos = m_pos + np.dot(quaternion.as_rotation_matrix(m_quat), np.dot(box_info[m.id]['markers'][m.id]['rot'], -np.array(box_info[m.id]['markers'][m.id]['pos'])))
            box_pose.px = m_new_pos[0]
            box_pose.py = m_new_pos[1]
            box_pose.pz = m_new_pos[2]
            box_pose.rx = m.pose.pose.orientation.x
            box_pose.ry = m.pose.pose.orientation.y
            box_pose.rz = m.pose.pose.orientation.z
            box_pose.rw = m.pose.pose.orientation.w
            box_pose.id = m.id
            delay = rospy.Time.now() - m.header.stamp
            rospy.loginfo("id: " + str(m.id) + " delay: " + str(delay.secs * 1000 + delay.nsecs / 1000000) + "ms")

            marker = Marker()
            marker.header.frame_id = marker_id.lstrip()
            marker.header.stamp = rospy.Time.now()
            marker.ns = "basic_shapes"
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
            marker.color.a = 1.0
            marker.scale.x = marker_size
            marker.scale.y = marker_size
            marker.scale.z = marker_size * 0.02;
            marker.lifetime = rospy.Duration()
            marker.type = 1
            marker_dict[m.id] = MarkerData(box_pose, marker)

    box_poses_data = BoxPoses()
    box_poses_data.existence = False
    box_poses_data.header.stamp = rospy.Time.now()
    markers_data = MarkerArray()
    for m in marker_dict:
        if marker_dict[m].probability > 0.5:
            marker_dict[m].marker_data.action = Marker.ADD
            box_poses_data.existence = True
            box_poses_data.header.stamp = marker_dict[m].box_pose_data.header.stamp
            box_poses_data.poses.append(marker_dict[m].box_pose_data)
            markers_data.markers.append(marker_dict[m].marker_data)
            marker_dict[m].probability -= 0.2
        elif marker_dict[m].probability > 0:
            marker_dict[m].marker_data.action = Marker.DELETE
            markers_data.markers.append(marker_dict[m].marker_data)
            marker_dict[m].probability = -1
    markers_pub.publish(markers_data)
    box_pose_pub.publish(box_poses_data)
    rate.sleep()

rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)
rospy.spin()
