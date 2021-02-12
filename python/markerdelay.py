#!/usr/bin/env python  
# -*- coding: utf-8 -*-
import roslib
import rospy
import math
import tf
import yaml
import numpy as np
import quaternion
import geometry_msgs.msg
import turtlesim.srv
from time import sleep
from hrpsys_ros_bridge.msg import BoxPoses
from hrpsys_ros_bridge.msg import BoxPose
from ar_track_alvar_msgs.msg import AlvarMarkers
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_msgs.msg import Bool
from carry_objects.srv import *
from carry_objects.msg import *
import copy

rospy.init_node('markerdelay_pub')
marker_stock = []
markers_pub = rospy.Publisher("dmarkers", MarkerArray, queue_size = 1)
enable_marker = [9, 8, 7]

with open("../config/box_info.yaml") as yml:
    box_info = yaml.load(yml)

def cb(msg):
    global marker_stock
    marker_stock.append(msg)
    #sleep(0.005)
    #if len(marker_stock) > 7:
    if True:
        current_markers = marker_stock.pop(0)
        markers_data = MarkerArray()
        for marker in current_markers.markers:
            if marker.id in enable_marker:
                marker.header.stamp = rospy.Time.now()
                markers_data.markers.append(marker)
                br = tf.TransformBroadcaster()
                br.sendTransform(
                    (marker.pose.position.x,
                     marker.pose.position.y,
                     marker.pose.position.z),
                    (marker.pose.orientation.x,
                     marker.pose.orientation.y,
                     marker.pose.orientation.z,
                     marker.pose.orientation.w),
                    rospy.Time.now(), "box"+str(marker.id), marker.header.frame_id)
            if marker.id == enable_marker[0]:
                for mid in range(1,len(enable_marker)):
                    marker_data = Marker()
                    marker_data.header = marker.header
                    marker_data.header.stamp = rospy.Time.now()
                    marker_data.ns = "st_box"
                    marker_data.id = enable_marker[mid]
                    marker_data.pose = copy.deepcopy(marker.pose)
                    z = 0
                    for mmid in range(1, mid+1):
                        z += box_info[enable_marker[mmid]]['size'][2]
                    z += box_info[enable_marker[0]]['size'][2] * 0.5
                    z -= box_info[enable_marker[mid]]['size'][2] * 0.5
                    pos = np.array([marker_data.pose.position.x, marker_data.pose.position.y, marker_data.pose.position.z])
                    rot = quaternion.as_rotation_matrix(np.quaternion(marker_data.pose.orientation.w, marker_data.pose.orientation.x, marker_data.pose.orientation.y, marker_data.pose.orientation.z))
                    pos = pos + np.dot(rot, np.array([0, 0, z]))
                    marker_data.pose.position.x = pos[0]
                    marker_data.pose.position.y = pos[1]
                    marker_data.pose.position.z = pos[2]
                    marker_data.color.r = 0.0
                    marker_data.color.g = 1.0
                    marker_data.color.b = 0.0
                    marker_data.color.a = 0.3
                    marker_data.scale.x = box_info[enable_marker[mid]]['size'][0]
                    marker_data.scale.y = box_info[enable_marker[mid]]['size'][1]
                    marker_data.scale.z = box_info[enable_marker[mid]]['size'][2]
                    marker_data.lifetime = rospy.Duration()
                    marker_data.type = 1
                    marker_data.action = Marker.ADD
                    markers_data.markers.append(marker_data)

        markers_pub.publish(markers_data)

rospy.Subscriber("markers", MarkerArray, cb)
rospy.spin()
