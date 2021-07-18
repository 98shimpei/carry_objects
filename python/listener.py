#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped

pub = rospy.Publisher('return', PointStamped, queue_size=10)
def callback(msg):
    print("I heard")
    difftime = rospy.Time.now() - msg.header.stamp
    print(difftime.secs * 1000 + float(difftime.nsecs) / 1000000)
    pos = PointStamped()
    pos.header.stamp = msg.header.stamp
    pub.publish(pos)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", PointStamped, callback)
    rospy.spin()

if __name__ == "__main__":
    listener()
