#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
from hrpsys_ros_bridge.msg import BoxPoses
from hrpsys_ros_bridge.msg import BoxPose

rospy.init_node('boxpose_pub')
pub = rospy.Publisher("box_pose", BoxPoses, queue_size = 1)

listener = tf.TransformListener()

print("hogefuga")
rate = rospy.Rate(20.0)
print("fuga")
while not rospy.is_shutdown():
    print("hehe")
    try:
        (trans,rot) = listener.lookupTransform('/map', '/ar_marker_7', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
    print("hoge")
    pubdata = BoxPoses()
    box_pose = BoxPose()
    box_pose.px = trans[0]
    box_pose.py = trans[1]
    box_pose.pz = trans[2]
    box_pose.rx = rot[0]
    box_pose.ry = rot[1]
    box_pose.rz = rot[2]
    box_pose.rw = rot[3]
    box_pose.id = 7
    pubdata.existence = True
    pubdata.poses.append(box_pose)
    pub.publish(pubdata)
    rate.sleep()
print("piyo")
