import roslib
import rospy
import math
import tf
import numpy as np
import quaternion
import geometry_msgs.msg
import time
from geometry_msgs.msg import WrenchStamped
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
marker_pub = rospy.Publisher("marker8", Marker, queue_size = 1)
rospy.init_node('boxpublisher8')
listener = tf.TransformListener()

def callback(msg):
    markers_data = MarkerArray()
    marker = Marker()
    marker.header = msg.header
    marker.header.frame_id = "box8"
    marker.ns = "marker"
    marker.id = 8
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1
    marker.scale.x = 0.310
    marker.scale.y = 0.310
    marker.scale.z = 0.230
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.5
    marker.color.a = 0.5
    marker.type = 1
    marker.lifetime = rospy.Duration()
    marker.action = Marker.ADD
    marker_pub.publish(marker)
    print("pub")

rospy.Subscriber("lhsensor", WrenchStamped, callback)
rospy.spin()
