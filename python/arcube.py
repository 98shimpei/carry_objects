#!/usr/bin/env python
import rospy
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from ar_track_alvar_msgs.msg import AlvarMarkers
import numpy as np
import quaternion

rospy.init_node("marker_pub")

pub = rospy.Publisher("markers", MarkerArray, queue_size = 1)
rate = rospy.Rate(25)

marker_size = rospy.get_param("/ar_track_alvar/marker_size", 5.0) * 0.01 #cm -> m
marker_id = rospy.get_param("/ar_track_alvar/output_frame", "/camera")

def callback(msg):
    markers_data = MarkerArray()

    for m in msg.markers:
        marker_data = Marker()
        marker_data.header.frame_id = marker_id.lstrip()
        marker_data.header.stamp = rospy.Time.now()

        marker_data.ns = "basic_shapes"
        marker_data.id = m.id

        marker_data.action = Marker.ADD

        marker_data.pose = m.pose.pose
        m_pos = np.array([marker_data.pose.position.x, marker_data.pose.position.y, marker_data.pose.position.z])
        m_quat = np.quaternion(marker_data.pose.orientation.w, marker_data.pose.orientation.x, marker_data.pose.orientation.y, marker_data.pose.orientation.z)
        m_new_pos = m_pos + np.dot(quaternion.as_rotation_matrix(m_quat), np.array([0, 0, marker_size * 0.5]))
        marker_data.pose.position.x = m_new_pos[0]
        marker_data.pose.position.y = m_new_pos[1]
        marker_data.pose.position.z = m_new_pos[2]

        marker_data.color.r = 1.0
        marker_data.color.g = 0.0
        marker_data.color.b = 0.0
        marker_data.color.a = 1.0

        marker_data.scale.x = marker_size
        marker_data.scale.y = marker_size
        marker_data.scale.z = marker_size

        marker_data.lifetime = rospy.Duration()

        marker_data.type = 1

        markers_data.markers.append(marker_data)

    pub.publish(markers_data)

rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)
rospy.spin()
