#!/usr/bin/env python
import rospy
import geometry_msgs.msg
from std_msgs.msg import String
from ar_track_alvar_msgs.msg import AlvarMarkers
import time
rospy.init_node('test_pub')

pub = rospy.Publisher("test", String, queue_size=1)
pub2 = rospy.Publisher("test2", String, queue_size=1)
r = rospy.Rate(10)
time.sleep(1)
#while not rospy.is_shutdown():
print("pub")
pub.publish(String("pub1"))
pub2.publish(String("pub2"))
    #r.sleep()
