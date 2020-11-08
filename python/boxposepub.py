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

box_info_fname = rospy.get_param("/boxpose_pub/info_yaml", "../config/box_info.yaml")
with open(box_info_fname) as yml:
    box_info = yaml.load(yml)

rospy.init_node('boxpose_pub')
pub = rospy.Publisher("box_pose", BoxPoses, queue_size = 1)

listener = tf.TransformListener()


rate = rospy.Rate(20.0)
def callback(msg):
    pubdata = BoxPoses()
    pubdata.existence = False
    pubdata.header.stamp = rospy.Time.now()
    for m in msg.markers:
        if m.id in box_info:
            box_pose = BoxPose()
            box_pose.header = m.header
            m_pos = np.array([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z])
            m_quat = np.quaternion(m.pose.pose.orientation.w, m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z)
            m_new_pos = m_pos + np.dot(quaternion.as_rotation_matrix(m_quat), np.dot(box_info[m.id]['marker']['rot'], -np.array(box_info[m.id]['marker']['pos'])))
            box_pose.px = m_new_pos[0]
            box_pose.py = m_new_pos[1]
            box_pose.pz = m_new_pos[2]
            box_pose.rx = m.pose.pose.orientation.x
            box_pose.ry = m.pose.pose.orientation.y
            box_pose.rz = m.pose.pose.orientation.z
            box_pose.rw = m.pose.pose.orientation.w
            box_pose.id = m.id
            pubdata.existence = True
            pubdata.header.stamp = m.header.stamp
            pubdata.poses.append(box_pose)
            delay = rospy.Time.now() - m.header.stamp
            rospy.loginfo("id: " + str(m.id) + " delay: " + str(delay.secs * 1000 + delay.nsecs / 1000000) + "ms")
    pub.publish(pubdata)
    rate.sleep()

rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)
rospy.spin()
