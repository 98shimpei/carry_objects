import rospy
import geometry_msgs.msg
from ar_track_alvar_msgs.msg import AlvarMarkers
from hrpsys_ros_bridge.msg import BoxPose
rospy.init_node('test_pub')

pub = rospy.Publisher("test", AlvarMarkers, queue_size=1)
r = rospy.Rate(10)
while not rospy.is_shutdown():
    print("pub")
    pub.publish(AlvarMarkers())
    r.sleep()
