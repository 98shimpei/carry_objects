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
        now = m.header.stamp
        target_marker_frame = '/ar_marker_' + str(m.id)
        flag = 3
        while(flag > 0):
            try:
                (trans,rot) = listener.lookupTransform('/map', target_marker_frame, now)
                flag = 0
                box_pose = BoxPose()
                box_pose.header = m.header
                box_pose.px = trans[0]
                box_pose.py = trans[1]
                box_pose.pz = trans[2]
                box_pose.rx = rot[0]
                box_pose.ry = rot[1]
                box_pose.rz = rot[2]
                box_pose.rw = rot[3]
                box_pose.id = m.id
                pubdata.existence = True
                pubdata.poses.append(box_pose)
                delay = rospy.Time.now() - m.header.stamp
                rospy.loginfo("id: " + str(m.id) + " delay: " + str(delay.secs * 1000 + delay.nsecs / 1000000) + "ms")
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                flag -= 1
                rospy.sleep(0.0001)
                continue
    pub.publish(pubdata)
    rate.sleep()

rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)
rospy.spin()
