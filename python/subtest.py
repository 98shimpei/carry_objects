import rospy
import geometry_msgs.msg
from ar_track_alvar_msgs.msg import AlvarMarkers
from hrpsys_ros_bridge.msg import BoxPose
rospy.init_node('test_sub')
def cb(msg):
    print("sub")

rospy.Subscriber("test", AlvarMarkers, cb)
rospy.spin()
