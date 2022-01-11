import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage

rospy.init_node("camera_info_publisher", anonymous=True)
publisher = rospy.Publisher("rs_l515/color/camera_info", CameraInfo, queue_size=1)

def callback(msg):
    camera_info_msg = CameraInfo()
    camera_info_msg.header = msg.header
    camera_info_msg.header.frame_id = "rs_l515_color_optical_frame"
    camera_info_msg.width = 960
    camera_info_msg.height = 540
    camera_info_msg.K = [680.8599243164062, 0.0, 490.65777587890625, 0.0, 680.97412109375, 279.8470458984375, 0.0, 0.0, 1.0]
    camera_info_msg.D = [0.14332719147205353, -0.4750896096229553, -0.0009839553385972977, -5.964898082311265e-05, 0.39867401123046875]
    camera_info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    camera_info_msg.P = [680.8599243164062, 0.0, 490.65777587890625, 0.0, 0.0, 680.97412109375, 279.8470458984375, 0.0, 0.0, 0.0, 1.0, 0.0]
    camera_info_msg.distortion_model = "plumb_bob"
    publisher.publish(camera_info_msg)

rospy.Subscriber("rs_l515/color/image_raw/compressed", CompressedImage, callback)
rospy.spin()

