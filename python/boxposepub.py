#!/usr/bin/env python  
# -*- coding: utf-8 -*-
import roslib
import rospy
import math
import tf
import yaml
import numpy as np
import quaternion
import geometry_msgs.msg
import turtlesim.srv
from hrpsys_ros_bridge.msg import BoxPoses
from hrpsys_ros_bridge.msg import BoxPose
from ar_track_alvar_msgs.msg import AlvarMarkers
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

box_info_fname = rospy.get_param("/boxpose_pub/info_yaml", "../config/box_info.yaml")
marker_size = rospy.get_param("/ar_track_alvar/marker_size", 5.0) * 0.01 #cm -> m
marker_frame_id = rospy.get_param("/ar_track_alvar/output_frame", "/camera")
world_tf = rospy.get_param("/boxpose_pub/world_tf", "/odom_ground")
top_box_id = rospy.get_param("/boxpose_pub/top_box_id", 7)
base_box_id = rospy.get_param("/boxpose_pub/base_box_id", 8)
hold_box_id = rospy.get_param("/boxpose_pub/hold_box_id", 9)
put_box_id = rospy.get_param("/boxpose_pub/put_box_id", 8)
look_box_mode = "box-balancer"

with open(box_info_fname) as yml:
    box_info = yaml.load(yml)
marker_to_box_dict = {}
for b in box_info:
    for m in box_info[b]['markers']:
        marker_to_box_dict[m] = b

rospy.init_node('boxpose_pub')
box_pose_pub = rospy.Publisher("box_pose", BoxPoses, queue_size = 1)
markers_pub = rospy.Publisher("markers", MarkerArray, queue_size = 1)
look_at_point_pub = rospy.Publisher("look_at_point", PointStamped, queue_size = 1)

box_dict = {}

listener = tf.TransformListener()
world_to_camera_pos = np.array([0, 0, 0])
world_to_camera_rot = np.identity(3)

class LookAtData:
    def __init__(self, bid, blocal):
        self.id = bid
        self.local_pos = blocal
        self.transition = 0.0
        self.old_local_pos = blocal
        self.new_local_pos = blocal
        self.look_at_pos = np.array([0, 0, 0])
        self.transition_rate = 0.1
    def new_target(self, bid, blocal):
        if bid < 0:
            self.id = bid
        if self.id < 0:
            self.id = bid
            self.old_local_pos = blocal
            self.new_local_pos = blocal
            self.local_pos = blocal
            self.transiton = 0.0
        if not (self.id == bid and np.linalg.norm(self.new_local_pos - blocal) < 0.001):
            self.old_local_pos, dummyrot = box_dict[bid].camera_to_local(self.look_at_pos)
            self.new_local_pos = blocal
            self.id = bid
            self.transition = 1.0
    def publish(self):
        if self.transition > 0:
            self.transition -= self.transition_rate
            if self.transition < 0:
                self.transition = 0
            self.local_pos = self.transition * self.old_local_pos + (1 - self.transition) * self.new_local_pos
        point_data = PointStamped()
        point_data.header.stamp = rospy.Time.now()
        if self.id in box_dict:
            self.look_at_pos, dummy_rot = box_dict[self.id].local_to_camera(self.local_pos)
            point_data.point.x = self.look_at_pos[0]
            point_data.point.y = self.look_at_pos[1]
            point_data.point.z = self.look_at_pos[2]
            br = tf.TransformBroadcaster()
            br.sendTransform(
                (self.look_at_pos[0], self.look_at_pos[1], self.look_at_pos[2]),
                (0, 0, 0, 1),
                rospy.Time.now(), "look_at_point", marker_frame_id.lstrip())
        else:
            point_data.point.x = 0.0
            point_data.point.y = 0.0
            point_data.point.z = 0.0
        look_at_point_pub.publish(point_data)

look_at_data = LookAtData(base_box_id, np.array([-box_info[base_box_id]['size'][0]/2.0, 0, box_info[base_box_id]['size'][2]/2.0]))

class MarkerData:
    def __init__(self, m, pos, rot):
        self.marker_data = m
        self.box_pos_from_marker = pos
        self.box_rot_from_marker = rot
        self.probability = 1.0

class BoxData:
    def __init__(self):
        self.box_pose_data = BoxPose()
        self.box_marker_data = Marker()
        self.markers_data = {}
        self.pos = np.array([0, 0, 0])
        self.rot = np.identity(3)
        self.quat = np.quaternion(1, 0, 0, 0)
        self.probability = 1.0
        self.initflag = True
        self.fixed_pos = np.array([0, 0, 0])
        self.fixed_rot = np.identity(3)
        
    def local_to_camera(self, local_pos, local_rot = np.identity(3)):
        tmppos = self.pos + np.dot(self.rot, local_pos)
        tmprot = np.dot(self.rot, local_rot)
        return tmppos, tmprot

    def camera_to_local(self, camera_pos, camera_rot = np.identity(3)):
        tmppos = np.dot(self.rot.T, (camera_pos - self.pos))
        tmprot = np.dot(self.rot.T, camera_rot)
        return tmppos, tmprot

    def pose_update(self, pos, rot, quat):
        self.pos = pos
        self.rot = rot
        self.quat = quat
        tmp_pos = world_to_camera_pos + np.dot(world_to_camera_rot, self.pos)
        tmp_rot = np.dot(world_to_camera_rot, self.rot)
        ft = 0.05
        if self.initflag:
            ft = 1.0
            self.initflag = False
        self.fixed_pos = (1.0-ft) * self.fixed_pos + ft * tmp_pos
        self.fixed_rot = (1.0-ft) * self.fixed_rot + ft * tmp_rot
    
    def disappear(self):
        self.box_marker_data.color.r = 0.0
        self.box_marker_data.color.g = 0.0
        self.box_marker_data.color.b = 1.0

    def marker_pose_update(self):
        self.box_marker_data.pose.position.x = self.pos[0]
        self.box_marker_data.pose.position.y = self.pos[1]
        self.box_marker_data.pose.position.z = self.pos[2]
        self.box_marker_data.pose.orientation.x = self.quat.x
        self.box_marker_data.pose.orientation.y = self.quat.y
        self.box_marker_data.pose.orientation.z = self.quat.z
        self.box_marker_data.pose.orientation.w = self.quat.w
        self.box_marker_data.header.frame_id = marker_frame_id.lstrip()
        self.box_marker_data.header.stamp = rospy.Time.now()

    def pose_estimate(self):
        self.pos = np.dot(world_to_camera_rot.T, (self.fixed_pos - world_to_camera_pos))
        self.rot = np.dot(world_to_camera_rot.T, self.fixed_rot)
        self.quat = quaternion.from_rotation_matrix(self.rot)


goal_box = BoxData()
goal_box.probability = 0.0

def callback(msg):
    global world_to_camera_pos, world_to_camera_rot
    start_time = rospy.Time.now()
    try:
        p, q = listener.lookupTransform(world_tf, marker_frame_id, rospy.Time(0))
        world_to_camera_pos = np.array(p)
        world_to_camera_rot = quaternion.as_rotation_matrix(np.quaternion(q[3], q[0], q[1], q[2])) #w,x,y,z
    except:
        print("tf listen error")
    top_box_id = rospy.get_param("/boxpose_pub/top_box_id", 7)
    base_box_id = rospy.get_param("/boxpose_pub/base_box_id", 8)
    hold_box_id = rospy.get_param("/boxpose_pub/hold_box_id", 9)
    put_box_id = rospy.get_param("/boxpose_pub/put_box_id", 8)
    a_time = rospy.Time.now()
    #マーカーについて
    for m in msg.markers:
        if m.id in marker_to_box_dict.keys():
            marker = Marker()
            marker.header = m.header
            marker.ns = "marker"
            marker.id = m.id
            marker.pose = m.pose.pose
            m_pos = np.array([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z])
            m_quat = np.quaternion(marker.pose.orientation.w, marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z)
            m_new_pos = m_pos + np.dot(quaternion.as_rotation_matrix(m_quat), np.array([0, 0, marker_size * 0.01]))
            marker.pose.position.x = m_new_pos[0]
            marker.pose.position.y = m_new_pos[1]
            marker.pose.position.z = m_new_pos[2]
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.5
            marker.scale.x = marker_size
            marker.scale.y = marker_size
            marker.scale.z = marker_size * 0.02
            marker.lifetime = rospy.Duration()
            marker.type = 1

            delay = rospy.Time.now() - m.header.stamp
            #rospy.loginfo("marker_id: " + str(m.id) + " delay: " + str(delay.secs * 1000 + delay.nsecs / 1000000) + "ms")

            b_pos = np.array([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z])
            b_rot = np.dot(quaternion.as_rotation_matrix(np.quaternion(m.pose.pose.orientation.w, m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z)), np.array(box_info[marker_to_box_dict[m.id]]['markers'][m.id]['rot']).T)
            b_pos = m_pos + np.dot(b_rot, -np.array(box_info[marker_to_box_dict[m.id]]['markers'][m.id]['pos']))

            if (not marker_to_box_dict[m.id] in box_dict) or box_dict[marker_to_box_dict[m.id]].probability < 0:
                box_dict[marker_to_box_dict[m.id]] = BoxData()
            box_dict[marker_to_box_dict[m.id]].box_pose_data.header = m.header
            box_dict[marker_to_box_dict[m.id]].box_marker_data.header = m.header
            box_dict[marker_to_box_dict[m.id]].markers_data[m.id] = MarkerData(marker, b_pos, b_rot)
            box_dict[marker_to_box_dict[m.id]].probability = 1.0


    #箱について
    for b in box_dict:
        if len(box_dict[b].markers_data) != 0:
            pos = np.array([0, 0, 0])
            rot = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
            probability_sum = 0
            for m in box_dict[b].markers_data:
                pos = pos + box_dict[b].markers_data[m].probability * box_dict[b].markers_data[m].box_pos_from_marker
                rot = rot + box_dict[b].markers_data[m].probability * box_dict[b].markers_data[m].box_rot_from_marker
                probability_sum = probability_sum + box_dict[b].markers_data[m].probability
            pos = pos / probability_sum
            rot = rot / probability_sum
            quat = quaternion.from_rotation_matrix(rot, nonorthogonal=True)

            box_dict[b].pose_update(pos, rot, quat)

            box_dict[b].box_pose_data.px = pos[0]
            box_dict[b].box_pose_data.py = pos[1]
            box_dict[b].box_pose_data.pz = pos[2]
            box_dict[b].box_pose_data.rx = quat.x
            box_dict[b].box_pose_data.ry = quat.y
            box_dict[b].box_pose_data.rz = quat.z
            box_dict[b].box_pose_data.rw = quat.w
            box_dict[b].box_pose_data.id = b


            box_dict[b].box_marker_data.ns = "box"
            box_dict[b].box_marker_data.id = b
            box_dict[b].box_marker_data.color.r = 1.0
            box_dict[b].box_marker_data.color.g = 1.0
            box_dict[b].box_marker_data.color.b = 0.0
            box_dict[b].box_marker_data.color.a = 0.5
            box_dict[b].box_marker_data.scale.x = box_info[b]['size'][0]
            box_dict[b].box_marker_data.scale.y = box_info[b]['size'][1]
            box_dict[b].box_marker_data.scale.z = box_info[b]['size'][2]
            box_dict[b].box_marker_data.lifetime = rospy.Duration()
            box_dict[b].box_marker_data.type = 1
            box_dict[b].box_marker_data.action = Marker.ADD

    #目標の箱について
    if put_box_id in box_dict:
        goal_box.pos, goal_box.rot = box_dict[put_box_id].local_to_camera(np.array([0, 0, 0.5 * (box_info[hold_box_id]['size'][2] + box_info[put_box_id]['size'][2])]))
        goal_box.quat = quaternion.from_rotation_matrix(goal_box.rot, nonorthogonal=True)
        goal_box.probability = 1.0

        goal_box.box_marker_data.header = box_dict[put_box_id].box_marker_data.header
        goal_box.box_marker_data.ns = "goal_box"
        goal_box.box_marker_data.id = 100
        goal_box.box_marker_data.pose.position.x = goal_box.pos[0]
        goal_box.box_marker_data.pose.position.y = goal_box.pos[1]
        goal_box.box_marker_data.pose.position.z = goal_box.pos[2]
        goal_box.box_marker_data.pose.orientation.x = goal_box.quat.x
        goal_box.box_marker_data.pose.orientation.y = goal_box.quat.y
        goal_box.box_marker_data.pose.orientation.z = goal_box.quat.z
        goal_box.box_marker_data.pose.orientation.w = goal_box.quat.w
        goal_box.box_marker_data.color.r = 0.0
        goal_box.box_marker_data.color.g = 1.0
        goal_box.box_marker_data.color.b = 0.0
        goal_box.box_marker_data.color.a = 0.2
        goal_box.box_marker_data.scale.x = box_info[hold_box_id]['size'][0]
        goal_box.box_marker_data.scale.y = box_info[hold_box_id]['size'][1]
        goal_box.box_marker_data.scale.z = box_info[hold_box_id]['size'][2]
        goal_box.box_marker_data.lifetime = rospy.Duration()
        goal_box.box_marker_data.type = 1

    b_time = rospy.Time.now()

    #publish
    box_poses_data = BoxPoses()
    box_poses_data.existence = False
    box_poses_data.header.stamp = rospy.Time.now()
    markers_data = MarkerArray()
    box_del_list = []
    for b in box_dict:
        #マーカー
        marker_del_list = []
        for m in box_dict[b].markers_data:
            if box_dict[b].markers_data[m].probability > 0:
                box_dict[b].markers_data[m].marker_data.action = Marker.ADD
                markers_data.markers.append(box_dict[b].markers_data[m].marker_data)
                box_dict[b].markers_data[m].probability -= 0.8
            else:
                box_dict[b].markers_data[m].marker_data.action = Marker.DELETE
                markers_data.markers.append(box_dict[b].markers_data[m].marker_data)
                marker_del_list.append(m)
        for m in marker_del_list:
            box_dict[b].markers_data.pop(m)
        #箱
        if box_dict[b].probability > 0:
            box_poses_data.existence = True
            box_poses_data.header.stamp = box_dict[b].box_pose_data.header.stamp
            box_poses_data.poses.append(box_dict[b].box_pose_data)
            box_dict[b].probability -= 0.8
        elif box_dict[b].probability > -5:
            box_dict[b].disappear()
            box_dict[b].pose_estimate()
            box_dict[b].probability = -10
        else:
            box_dict[b].pose_estimate()
        br = tf.TransformBroadcaster()
        br.sendTransform(
            (box_dict[b].pos[0],
             box_dict[b].pos[1],
             box_dict[b].pos[2]),
            (box_dict[b].quat.x,
             box_dict[b].quat.y,
             box_dict[b].quat.z,
             box_dict[b].quat.w),
            rospy.Time.now(), "box"+str(b), marker_frame_id.lstrip())
        box_dict[b].marker_pose_update()
        markers_data.markers.append(box_dict[b].box_marker_data)

    if goal_box.probability > 0:
        goal_box.box_marker_data.action = Marker.ADD
        br = tf.TransformBroadcaster()
        br.sendTransform(
            (goal_box.pos[0],
             goal_box.pos[1],
             goal_box.pos[2]),
            (goal_box.quat.x,
             goal_box.quat.y,
             goal_box.quat.z,
             goal_box.quat.w),
            rospy.Time.now(), "goal_box", marker_frame_id.lstrip())
        markers_data.markers.append(goal_box.box_marker_data)
        goal_box.probability -= 0.3
    elif goal_box.probability > -5:
        goal_box.box_marker_data.action = Marker.DELETE
        goal_box.probability = -10
        markers_data.markers.append(goal_box.box_marker_data)

    markers_pub.publish(markers_data)
    box_pose_pub.publish(box_poses_data)

    c_time = rospy.Time.now()

    #look_at_pointを出力
    bid = -1
    blocal = np.array([0, 0, 0])
    if look_box_mode == "lift-box":
        if hold_box_id in box_dict:
            bid = hold_box_id
            blocal = np.array([-box_info[hold_box_id]['size'][0]/2.0, 0, -box_info[hold_box_id]['size'][2]/2.0])
        elif base_box_id in box_dict:
            bid = base_box_id
            blocal = np.array([-box_info[base_box_id]['size'][0]/2.0, 0, -box_info[base_box_id]['size'][2]/2.0])
        elif top_box_id in box_dict:
            bid = top_box_id
            blocal = np.array([-box_info[top_box_id]['size'][0]/2.0, 0, -box_info[top_box_id]['size'][2]/2.0])
    elif look_box_mode == "box-balancer":
        if base_box_id in box_dict:
            bid = base_box_id
            blocal = np.array([-box_info[base_box_id]['size'][0]/2.0, 0, box_info[base_box_id]['size'][2]/2.0])
        elif top_box_id in box_dict:
            bid = top_box_id
            blocal = np.array([-box_info[top_box_id]['size'][0]/2.0, 0, -box_info[top_box_id]['size'][2]/2.0])
        elif hold_box_id in box_dict:
            bid = hold_box_id
            blocal = np.array([-box_info[hold_box_id]['size'][0]/2.0, 0, box_info[hold_box_id]['size'][2]/2.0])
    elif look_box_mode == "put-box":
        if put_box_id in box_dict:
            bid = put_box_id
            blocal = np.array([-box_info[hold_box_id]['size'][0]/2.0, 0, box_info[hold_box_id]['size'][2]/2.0])
        elif hold_box_id in box_dict:
            bid = hold_box_id
            blocal = np.array([-box_info[hold_box_id]['size'][0]/2.0, 0, -box_info[hold_box_id]['size'][2]/2.0])
    
    if bid > 0:
        look_at_data.new_target(bid, blocal)
    look_at_data.publish()

    
    d_time = rospy.Time.now()

    #持つ箱について手の位置・体の位置の目標TFを出力
    if hold_box_id in box_dict:
        br = tf.TransformBroadcaster()
        for tag in ['rhand_pose', 'lhand_pose', 'body_pose']:
            tfpos, tfrot = box_dict[hold_box_id].local_to_camera(np.array(box_info[hold_box_id][tag]['pos']), np.array(box_info[hold_box_id][tag]['rot']))
            tfquat = quaternion.from_rotation_matrix(tfrot, nonorthogonal=True)
            br.sendTransform(
                (tfpos[0], tfpos[1], tfpos[2]),
                (tfquat.x, tfquat.y, tfquat.z, tfquat.w),
                rospy.Time.now(), tag, marker_frame_id.lstrip())
    if goal_box.probability > 0:
        for tag in ['rhand_pose', 'lhand_pose', 'body_pose']:
            tfpos, tfrot = goal_box.local_to_camera(np.array(box_info[hold_box_id][tag]['pos']), np.array(box_info[hold_box_id][tag]['rot']))
            tfquat = quaternion.from_rotation_matrix(tfrot, nonorthogonal=True)
            br.sendTransform(
                (tfpos[0], tfpos[1], tfpos[2]),
                (tfquat.x, tfquat.y, tfquat.z, tfquat.w),
                rospy.Time.now(), 'goal_'+tag, marker_frame_id.lstrip())
    e_time = rospy.Time.now()
    a_t = (a_time - start_time).secs + float((a_time - start_time).nsecs) / 1000000000
    b_t = (b_time - a_time).secs + float((b_time - a_time).nsecs) / 1000000000
    c_t = (c_time - b_time).secs + float((c_time - b_time).nsecs) / 1000000000
    d_t = (d_time - c_time).secs + float((d_time - c_time).nsecs) / 1000000000
    e_t = (e_time - d_time).secs + float((e_time - d_time).nsecs) / 1000000000
    all_t = (e_time - start_time).secs + float((e_time - start_time).nsecs) / 1000000000
    if all_t > 0 :
        rospy.loginfo(
            " " + "{:.3f}".format(all_t) + " "
            " " + "{:.3f}".format(a_t) + " "
            " " + "{:.3f}".format(b_t) + " "
            " " + "{:.3f}".format(c_t) + " "
            " " + "{:.3f}".format(d_t) + " "
            " " + "{:.3f}".format(e_t) + " "
            " " + "{:.2f}".format(a_t / all_t) + " "
            " " + "{:.2f}".format(b_t / all_t) + " "
            " " + "{:.2f}".format(c_t / all_t) + " "
            " " + "{:.2f}".format(d_t / all_t) + " "
            " " + "{:.2f}".format(e_t / all_t) + " "
            )

def mode_cb(msg):
    global look_box_mode
    look_box_mode = msg.data
    print(look_box_mode)
    
rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)
rospy.Subscriber("look_box_mode", String, mode_cb)
rospy.spin()
