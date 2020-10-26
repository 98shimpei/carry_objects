import rospy
import geometry_msgs.msg
from ar_track_alvar_msgs.msg import AlvarMarkers
from hrpsys_ros_bridge.msg import BoxPose
rospy.init_node('test_pub')

pub = rospy.Publisher("test", AlvarMarkers, queue_size=1)
while(True):
    pub.publish(AlvarMarkers())
    rospy.sleep(0.1)
