#!/usr/bin/env python
import rospy
import threading
import time
from geometry_msgs.msg import PointStamped
rospy.init_node('talker', anonymous=True)
pub = rospy.Publisher('chatter', PointStamped, queue_size=10)
pubtime = rospy.Time.now()
tlock = threading.Lock()

def callback(msg):
    global pubtime
    print('back')
    tlock.acquire()
    difftime = rospy.Time.now() - pubtime
    difftime_header = rospy.Time.now() - msg.header.stamp
    print(difftime.secs * 1000 + float(difftime.nsecs) / 1000000)
    print(difftime_header.secs * 1000 + float(difftime_header.nsecs) / 1000000)
    tlock.release()

def talker():
    global pubtime
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        pos = PointStamped()
        tlock.acquire()
        print("talk")
        pubtime = rospy.Time.now()
        pos.header.stamp = pubtime
        tlock.release()
        pub.publish(pos)
        r.sleep()

def return_listener():
    rospy.Subscriber("return", PointStamped, callback)
    rospy.spin()

if __name__=='__main__':
    try:
        thread1 = threading.Thread(target=talker)
        thread2 = threading.Thread(target=return_listener)
        thread1.start()
        time.sleep(3)
        thread2.start()
    except rospy.ROSInterruptException: pass
