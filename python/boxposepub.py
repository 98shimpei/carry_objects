#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
from hrpsys_ros_bridge.msg import BoxPoses
from hrpsys_ros_bridge.msg import BoxPose
from ar_track_alvar_msgs.msg import AlvarMarkers

rospy.init_node('boxpose_pub')
pub = rospy.Publisher("box_pose", BoxPoses, queue_size = 1)

listener = tf.TransformListener()


rate = rospy.Rate(20.0)
def callback(msg):
    pubdata = BoxPoses()
    pubdata.existence = False
    pubdata.header.stamp = rospy.Time.now()
    for m in msg.markers:
        box_pose = BoxPose()
        box_pose.header = m.header
        box_pose.px = m.pose.pose.position.x
        box_pose.py = m.pose.pose.position.y
        box_pose.pz = m.pose.pose.position.z
        box_pose.rx = m.pose.pose.orientation.x
        box_pose.ry = m.pose.pose.orientation.y
        box_pose.rz = m.pose.pose.orientation.z
        box_pose.rw = m.pose.pose.orientation.w
        box_pose.id = m.id
        pubdata.existence = True
        pubdata.poses.append(box_pose)
        delay = rospy.Time.now() - m.header.stamp
        rospy.loginfo("id: " + str(m.id) + " delay: " + str(delay.secs * 1000 + delay.nsecs / 1000000) + "ms")
    pub.publish(pubdata)
    rate.sleep()

rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)
rospy.spin()
