import rospy
import geometry_msgs.msg
from ar_track_alvar_msgs.msg import AlvarMarkers
rospy.init_node('test')
def cb(msg):
    print("hoge")

rospy.Subscriber("ar_pose_marker", AlvarMarkers, cb)
rospy.spin()
