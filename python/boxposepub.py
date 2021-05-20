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
from hrpsys_ros_bridge.msg import BoxPoses
from hrpsys_ros_bridge.msg import BoxPose
from ar_track_alvar_msgs.msg import AlvarMarkers
from stag_ros.msg import STagMarkerArray
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_msgs.msg import Bool
from carry_objects.srv import *
from carry_objects.msg import *

box_info_fname = rospy.get_param("/boxpose_pub/info_yaml", "../config/box_info.yaml")
marker_size = rospy.get_param("/ar_track_alvar/marker_size", 15.7) * 0.01 #cm -> m
world_tf = rospy.get_param("/boxpose_pub/world_tf", "/odom_ground")
top_box_id = rospy.get_param("/boxpose_pub/top_box_id", 7)
base_box_id = rospy.get_param("/boxpose_pub/base_box_id", 8)
hold_box_id = rospy.get_param("/boxpose_pub/hold_box_id", 9)
put_box_id = rospy.get_param("/boxpose_pub/put_box_id", 8)
marker_type = rospy.get_param("/boxpose_pub/marker_type", "ar")
marker_frame_id = rospy.get_param("/boxpose_pub/output_frame", "/camera")

look_box_mode = "box-balancer"
okinaoshi_counter = 3

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
box_states_pub = rospy.Publisher("box_states", BoxStates, queue_size = 1)
emergency_command_pub = rospy.Publisher("emergency_command", EmergencyCommand, queue_size = 1)

box_dict = {}
box_states = BoxStates()

ignore_marker_dict = {}

listener = tf.TransformListener()
world_to_camera_pos = np.array([0, 0, 0])
world_to_camera_rot = np.identity(3)
lhand_pos = np.array([0, 0, 0])
rhand_pos = np.array([0, 0, 0])
lhand_rot = np.array([0, 0, 0])
rhand_rot = np.array([0, 0, 0])
lift_now = False
check_cooltime = 10
box_look_flag = False
look_timer = 200
looked_time = rospy.Time.now()

class LookAtData:
    def __init__(self, bid, blocal):
        self.id = bid
        self.local_pos = blocal
        self.transition = 0.0
        self.old_local_pos = blocal
        self.new_local_pos = blocal
        self.look_at_pos = np.array([0, 0, 0])
        self.transition_rate = 0.022
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
        #look_at_point_pub.publish(point_data)

look_at_data = LookAtData(base_box_id, np.array([-box_info[base_box_id]['size'][0]/2.0, 0, box_info[base_box_id]['size'][2]/2.0]))

class MarkerData:
    def __init__(self, m, pos, rot):
        self.marker_data = m
        self.box_pos_from_marker = pos
        self.box_rot_from_marker = rot
        self.probability = 1.0

class BoxData:
    def __init__(self, bid):
        self.redetect_init()
        self.pos = np.array([0, 0, 0])
        self.rot = np.identity(3)
        self.quat = np.quaternion(1, 0, 0, 0)
        self.markers_data = {}
        self.localinitflag = True
        self.fixed_pos = np.array([0, 0, 0])
        self.fixed_rot = np.identity(3)
        #-1...world, -2...none(hand, air)
        self.fixed_id = -1
        self.on_id = -2
        self.myu = 1
        self.myudash = 1
        self.lift = False
        self.id = bid

    def redetect_init(self):
        self.box_pose_data = BoxPose()
        self.box_marker_data = Marker()
        self.probability = 1.0
        self.state_update_flag = False #cbはじめにFalseになり、pos,rot,quatを更新したらTrueになる
        self.initflag = True

    def local_to_camera(self, local_pos, local_rot = np.identity(3)):
        tmppos = self.pos + np.dot(self.rot, local_pos)
        tmprot = np.dot(self.rot, local_rot)
        return tmppos, tmprot

    def camera_to_local(self, camera_pos, camera_rot = np.identity(3)):
        tmppos = np.dot(self.rot.T, (camera_pos - self.pos))
        tmprot = np.dot(self.rot.T, camera_rot)
        return tmppos, tmprot

    def change_fixed_id(self, new_id):
        tmppos = self.pos
        tmprot = self.rot
        if self.fixed_id == -1:
            tmppos = np.dot(world_to_camera_rot.T, (self.fixed_pos - world_to_camera_pos))
            tmprot = np.dot(world_to_camera_rot.T, self.fixed_rot)
        elif self.fixed_id == -2:
            tmppos = np.dot(world_to_camera_rot.T, (self.fixed_pos - world_to_camera_pos))
            tmprot = np.dot(world_to_camera_rot.T, self.fixed_rot)
        else:
            if self.fixed_id in box_dict:
                tmppos, tmprot = box_dict[self.fixed_id].local_to_camera(self.fixed_pos, self.fixed_rot)
        if new_id == -1:
            self.fixed_pos = world_to_camera_pos + np.dot(world_to_camera_rot, tmppos)
            self.fixed_rot = np.dot(world_to_camera_rot, tmprot)
        elif new_id == -2:
            self.fixed_pos = world_to_camera_pos + np.dot(world_to_camera_rot, tmppos)
            self.fixed_rot = np.dot(world_to_camera_rot, tmprot)
        else:
            if new_id in box_dict:
                self.fixed_pos, self.fixed_rot = box_dict[new_id].camera_to_local(tmppos, tmprot)
        self.fixed_id = new_id

    def up_to_down_update(self):
        self.state_update_flag = True
        tmp_pos = self.pos
        tmp_rot = self.rot
        if self.fixed_id == -1: #world
            tmp_pos = world_to_camera_pos + np.dot(world_to_camera_rot, self.pos)
            tmp_rot = np.dot(world_to_camera_rot, self.rot)
        elif self.fixed_id == -2: #none(hand)
            tmp_pos = world_to_camera_pos + np.dot(world_to_camera_rot, self.pos)
            tmp_rot = np.dot(world_to_camera_rot, self.rot)
        else:
            if self.fixed_id in box_dict:
                if box_dict[self.fixed_id].probability > 0: #localの更新
                    tmp_pos, tmp_rot = box_dict[self.fixed_id].camera_to_local(self.pos, self.rot)
                else: #下の箱のposの更新
                    if box_dict[self.fixed_id].state_update_flag == False:
                        box_dict[self.fixed_id].pos, box_dict[self.fixed_id].rot = self.local_to_camera(np.dot(self.fixed_rot.transpose(),-self.fixed_pos), self.fixed_rot.transpose())
                        box_dict[self.fixed_id].quat = quaternion.from_rotation_matrix(box_dict[self.fixed_id].rot, nonorthogonal=True)
                        box_dict[self.fixed_id].up_to_down_update()
                    return
        #localの更新
        ft = 0.1
        if self.localinitflag:
            ft = 1.0
            self.localinitflag = False
        elif self.lift:
            ft = 0.3
        self.fixed_pos = (1.0-ft) * self.fixed_pos + ft * tmp_pos
        self.fixed_rot = (1.0-ft) * self.fixed_rot + ft * tmp_rot

    def disappear(self):
        #消えた(probabilityが0以下になった)瞬間の処理
        self.box_marker_data.color.r = 0.0
        self.box_marker_data.color.g = 0.0
        self.box_marker_data.color.b = 1.0

    def box_pose_data_update(self):
        self.box_pose_data.px = self.pos[0]
        self.box_pose_data.py = self.pos[1]
        self.box_pose_data.pz = self.pos[2]
        self.box_pose_data.rx = self.quat.x
        self.box_pose_data.ry = self.quat.y
        self.box_pose_data.rz = self.quat.z
        self.box_pose_data.rw = self.quat.w
        self.box_pose_data.id = self.id

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

    def down_to_up_update(self):
        global box_look_flag
        if self.fixed_id == -1: #world
            self.pos = np.dot(world_to_camera_rot.T, (self.fixed_pos - world_to_camera_pos))
            self.rot = np.dot(world_to_camera_rot.T, self.fixed_rot)
            self.quat = quaternion.from_rotation_matrix(self.rot, nonorthogonal=True)
        elif self.fixed_id == -2:
            self.pos = np.dot(world_to_camera_rot.T, (self.fixed_pos - world_to_camera_pos))
            self.rot = np.dot(world_to_camera_rot.T, self.fixed_rot)
            self.quat = quaternion.from_rotation_matrix(self.rot, nonorthogonal=True)
            rospy.loginfo("motteru noni mitenai")
            box_look_flag = False
        else:
            if self.fixed_id in box_dict:
                if box_dict[self.fixed_id].state_update_flag == False:
                    box_dict[self.fixed_id].down_to_up_update()
                self.pos, self.rot = box_dict[self.fixed_id].local_to_camera(self.fixed_pos, self.fixed_rot)
                self.quat = quaternion.from_rotation_matrix(self.rot, nonorthogonal=True)
        self.state_update_flag = True
    def check_slip(self, dangerous_safety, safety, modify_safety, on_weight, on_pos, on_calc_weight):
        global boxstates
        boxstate = BoxState()
        hogepos, hogerot = box_dict[hold_box_id].camera_to_local(self.pos, self.rot)
        hogequat = quaternion.from_rotation_matrix(hogerot, nonorthogonal=True)
        boxstate.pose.position.x = hogepos[0]
        boxstate.pose.position.y = hogepos[1]
        boxstate.pose.position.z = hogepos[2]
        boxstate.pose.orientation.x = hogequat.x
        boxstate.pose.orientation.y = hogequat.y
        boxstate.pose.orientation.z = hogequat.z
        boxstate.pose.orientation.w = hogequat.w
        boxstate.size = (np.array(box_info[self.id]['size']) * 1000).tolist()
        boxstate.color = [1.0, 1.0, 1.0, 0.5]
        box_states.boxstates.append(boxstate)
        if self.fixed_id > 0:
            local_on_pos = self.fixed_pos + np.dot(self.fixed_rot, on_pos)
            weight = box_info[self.id]['mass'] + on_weight
            #cogposはひとつ下の箱座標系
            cogpos = (self.fixed_pos * box_info[self.id]['mass'] + local_on_pos * on_weight) / weight
            tmppos, dummyrot = box_dict[hold_box_id].camera_to_local(self.pos)
            l1 = tmppos[2]
            tmppos, dummyrot = box_dict[hold_box_id].camera_to_local(box_dict[self.fixed_id].pos)
            l2 = tmppos[2]
            calc_weight = (l1 * l1 * self.myudash * box_info[self.id]['mass'] + on_calc_weight)
            #落ちそうなとき。揺すって修正したい
            #揺すって滑る長さは、持ってる箱(本当はハンド)からの高さ^2と動摩擦係数に比例
            if abs(cogpos[0]) > box_info[self.fixed_id]['size'][0]*0.5*dangerous_safety or abs(cogpos[1]) > box_info[self.fixed_id]['size'][1]*0.5*dangerous_safety:
                boxstate.color = [1.0, 0.5, 0.5, 0.5]
                return np.array([100, 100, 100])
            elif abs(cogpos[0]) > box_info[self.fixed_id]['size'][0]*0.5*safety:
                #下の箱座標系
                modify_distance = np.array([cogpos[0], cogpos[1], 0]) * (1.0 - (box_info[self.fixed_id]['size'][0]*0.5*modify_safety / abs(cogpos[0]))) * (l1 * l1 * weight / calc_weight) / (l1 * l1 * self.myudash - l2 * l2 * box_dict[self.fixed_id].myudash)
                #世界座標系に変換
                modify_distance = np.dot(world_to_camera_rot, np.dot(box_dict[self.fixed_id].rot, modify_distance))
                print(str(self.id) + " detect dangerous slip")
                tmp = box_dict[self.fixed_id].check_slip(dangerous_safety, safety, modify_safety, weight, cogpos, calc_weight)
                if np.linalg.norm(tmp) > np.linalg.norm(modify_distance):
                    modify_distance = tmp
                return modify_distance
            elif abs(cogpos[1]) > box_info[self.fixed_id]['size'][1]*0.5*safety:
                #下の箱座標系
                modify_distance = np.array([cogpos[0], cogpos[1], 0]) * (1.0 - (box_info[self.fixed_id]['size'][1]*0.5*modify_safety / abs(cogpos[1]))) * (l1 * l1 * weight / calc_weight) / (l1 * l1 * self.myudash - l2 * l2 * box_dict[self.fixed_id].myudash)
                #世界座標系に変換
                modify_distance = np.dot(world_to_camera_rot, np.dot(box_dict[self.fixed_id].rot, modify_distance))
                print(str(self.id) + " detect dangerous slip")
                tmp = box_dict[self.fixed_id].check_slip(dangerous_safety, safety, modify_safety, weight, cogpos, calc_weight)
                if np.linalg.norm(tmp) > np.linalg.norm(modify_distance):
                    modify_distance = tmp
                return modify_distance
            else:
                return box_dict[self.fixed_id].check_slip(dangerous_safety, safety, modify_safety, weight, cogpos, calc_weight)
        else:
            return np.array([0, 0, 0])

    def check_modified_slip(self, safety, modify_distance, on_weight, on_pos):
        global boxstates
        if self.fixed_id > 0:
            local_on_pos = self.fixed_pos + np.dot(self.fixed_rot, on_pos)
            weight = box_info[self.id]['mass'] + on_weight
            tmppos, dummyrot = box_dict[hold_box_id].camera_to_local(self.pos)
            l1 = tmppos[2]
            tmppos, dummyrot = box_dict[hold_box_id].camera_to_local(box_dict[self.fixed_id].pos)
            l2 = tmppos[2]
            modified_pos = self.fixed_pos - np.dot(box_dict[self.fixed_id].rot.T, np.dot(world_to_camera_rot.T, modify_distance)) * (l1 * l1 * self.myudash - l2 * l2 * box_dict[self.fixed_id].myudash)
            cogpos = (modified_pos * box_info[self.id]['mass'] + local_on_pos * on_weight) / weight
            modified_pos_for_boxstate = self.fixed_pos - np.dot(box_dict[self.fixed_id].rot.T, np.dot(world_to_camera_rot.T, modify_distance)) * (l1 * l1 * self.myudash)
            boxstate = BoxState()
            hogepos, hogerot = box_dict[self.fixed_id].local_to_camera(modified_pos_for_boxstate, self.fixed_rot)
            hogepos, hogerot = box_dict[hold_box_id].camera_to_local(hogepos, hogerot)
            hogequat = quaternion.from_rotation_matrix(hogerot, nonorthogonal=True)
            boxstate.pose.position.x = hogepos[0]
            boxstate.pose.position.y = hogepos[1]
            boxstate.pose.position.z = hogepos[2]
            boxstate.pose.orientation.x = hogequat.x
            boxstate.pose.orientation.y = hogequat.y
            boxstate.pose.orientation.z = hogequat.z
            boxstate.pose.orientation.w = hogequat.w
            boxstate.size = (np.array(box_info[self.id]['size']) * 1000).tolist()
            boxstate.color = [0.5, 1.0, 0.5, 0.5]
            box_states.boxstates.append(boxstate)
            if abs(cogpos[0]) > box_info[self.fixed_id]['size'][0]*0.5*safety or abs(cogpos[1]) > box_info[self.fixed_id]['size'][1]*0.5*safety:
                print(str(self.id) + " detect very dangerous slip")
                boxstate.color = [1.0, 0.5, 0.5, 0.5]
                box_dict[self.fixed_id].check_modified_slip(safety, modify_distance, weight, cogpos)
                return True
            else:
                return box_dict[self.fixed_id].check_modified_slip(safety, modify_distance, weight, cogpos)
        else:
            boxstate = BoxState()
            hogepos, hogerot = box_dict[hold_box_id].camera_to_local(self.pos, self.rot)
            hogequat = quaternion.from_rotation_matrix(hogerot, nonorthogonal=True)
            boxstate.pose.position.x = hogepos[0]
            boxstate.pose.position.y = hogepos[1]
            boxstate.pose.position.z = hogepos[2]
            boxstate.pose.orientation.x = hogequat.x
            boxstate.pose.orientation.y = hogequat.y
            boxstate.pose.orientation.z = hogequat.z
            boxstate.pose.orientation.w = hogequat.w
            boxstate.size = (np.array(box_info[self.id]['size']) * 1000).tolist()
            boxstate.color = [0.5, 1.0, 0.5, 0.5]
            box_states.boxstates.append(boxstate)
            return False


goal_box = BoxData(0)
goal_box.probability = 0.0

cb_count = 0

def callback(msg):
    global world_to_camera_pos, world_to_camera_rot, camera_to_hand_pos, camera_to_hand_rot, cb_count, box_states, check_cooltime, box_look_flag, look_timer, okinaoshi_counter, looked_time
    start_time = rospy.Time.now()
    try:
        p, q = listener.lookupTransform(world_tf, marker_frame_id, rospy.Time(0))
        world_to_camera_pos = np.array(p)
        world_to_camera_rot = quaternion.as_rotation_matrix(np.quaternion(q[3], q[0], q[1], q[2])) #w,x,y,z
        p, q = listener.lookupTransform(marker_frame_id, "/larm_end_coords", rospy.Time(0))
        r, s = listener.lookupTransform(marker_frame_id, "/rarm_end_coords", rospy.Time(0))
        lhand_pos = np.array(p)
        rhand_pos = np.array(p)
        lhand_rot = quaternion.as_rotation_matrix(np.quaternion(q[3], q[0], q[1], q[2])) #w,x,y,z
        rhand_rot = quaternion.as_rotation_matrix(np.quaternion(s[3], s[0], s[1], s[2])) #w,x,y,z
    except:
        print("tf listen error")
    a_time = rospy.Time.now()
    #state_update_flagをFalseに
    for b in box_dict:
        box_dict[b].state_update_flag = False
    box_look_flag = True

    #マーカーについて
    #TODO: 二重に書いてるのどうにかしようね
    if marker_type == "ar":
        for m in msg.markers:
            if np.linalg.norm(np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])) == 0:
                continue
            if m.id in marker_to_box_dict.keys():
                if not m.id in ignore_marker_dict.keys():
                    ignore_marker_dict[m.id] = 2
                    continue
                elif ignore_marker_dict[m.id] > 0:
                    ignore_marker_dict[m.id] -= 1
                    continue
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

                looked_time = m.header.stamp
                #delay_time = rospy.Time.now() - m.header.stamp
                #delay = delay_time.secs * 1000 + delay_time.nsecs / 1000000
                #rospy.loginfo("marker_id: " + str(m.id) + " delay: " + str(delay) + "ms")

                b_pos = np.array([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z])
                b_rot = np.dot(quaternion.as_rotation_matrix(np.quaternion(m.pose.pose.orientation.w, m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z)), np.array(box_info[marker_to_box_dict[m.id]]['markers'][m.id]['rot']).T)
                b_pos = m_pos + np.dot(b_rot, -np.array(box_info[marker_to_box_dict[m.id]]['markers'][m.id]['pos']))

                if not marker_to_box_dict[m.id] in box_dict:
                    box_dict[marker_to_box_dict[m.id]] = BoxData(marker_to_box_dict[m.id])
                    box_dict[marker_to_box_dict[m.id]].box_pose_data.header = m.header
                    box_dict[marker_to_box_dict[m.id]].box_marker_data.header = m.header
                    box_dict[marker_to_box_dict[m.id]].markers_data[m.id] = MarkerData(marker, b_pos, b_rot)
                    box_dict[marker_to_box_dict[m.id]].probability = 1.0
                else:
                    if box_dict[marker_to_box_dict[m.id]].probability < 0:
                        box_dict[marker_to_box_dict[m.id]].redetect_init()
                    if abs(np.linalg.norm(box_dict[marker_to_box_dict[m.id]].pos) - np.linalg.norm(b_pos)) > 1.0:
                        print("marker pos jamping id: " + str(m.id))
                    elif np.sum(np.abs(np.dot(box_dict[marker_to_box_dict[m.id]].rot, b_rot.T) - np.identity(3))) > 0.50:
                        print("marker rot jamping id: " + str(m.id) + " " + str(np.sum(np.dot(box_dict[marker_to_box_dict[m.id]].rot, b_rot) - np.identity(3))))
                    else:
                        box_dict[marker_to_box_dict[m.id]].box_pose_data.header = m.header
                        box_dict[marker_to_box_dict[m.id]].box_marker_data.header = m.header
                        box_dict[marker_to_box_dict[m.id]].markers_data[m.id] = MarkerData(marker, b_pos, b_rot)
                        box_dict[marker_to_box_dict[m.id]].probability = 1.0

    elif marker_type == "stag":
        for m in msg.stag_array:
            if np.linalg.norm(np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])) == 0:
                continue
            if m.id.data in marker_to_box_dict.keys():
                if not m.id.data in ignore_marker_dict.keys():
                    ignore_marker_dict[m.id.data] = 2
                    continue
                elif ignore_marker_dict[m.id.data] > 0:
                    ignore_marker_dict[m.id.data] -= 1
                    continue
                marker = Marker()
                marker.header = m.header
                marker.ns = "marker"
                marker.id = m.id.data
                marker.pose = m.pose
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

                looked_time = m.header.stamp
                #delay_time = rospy.Time.now() - m.header.stamp
                #delay = delay_time.secs * 1000 + delay_time.nsecs / 1000000
                #rospy.loginfo("marker_id: " + str(m.id) + " delay: " + str(delay) + "ms")

                b_pos = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
                b_rot = np.dot(quaternion.as_rotation_matrix(np.quaternion(m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z)), np.array(box_info[marker_to_box_dict[m.id.data]]['markers'][m.id.data]['rot']).T)
                b_pos = m_pos + np.dot(b_rot, -np.array(box_info[marker_to_box_dict[m.id.data]]['markers'][m.id.data]['pos']))

                if not marker_to_box_dict[m.id.data] in box_dict:
                    box_dict[marker_to_box_dict[m.id.data]] = BoxData(marker_to_box_dict[m.id.data])
                    box_dict[marker_to_box_dict[m.id.data]].box_pose_data.header = m.header
                    box_dict[marker_to_box_dict[m.id.data]].box_marker_data.header = m.header
                    box_dict[marker_to_box_dict[m.id.data]].markers_data[m.id.data] = MarkerData(marker, b_pos, b_rot)
                    box_dict[marker_to_box_dict[m.id.data]].probability = 1.0
                else:
                    if box_dict[marker_to_box_dict[m.id.data]].probability < 0:
                        box_dict[marker_to_box_dict[m.id.data]].redetect_init()
                    if abs(np.linalg.norm(box_dict[marker_to_box_dict[m.id.data]].pos) - np.linalg.norm(b_pos)) > 0.5:
                        print("marker pos jamping id: " + str(m.id.data))
                    elif np.sum(np.abs(np.dot(box_dict[marker_to_box_dict[m.id.data]].rot, b_rot.T) - np.identity(3))) > 0.40:
                        print("marker rot jamping id: " + str(m.id.data) + " " + str(np.sum(np.dot(box_dict[marker_to_box_dict[m.id.data]].rot, b_rot) - np.identity(3))))
                    else:
                        box_dict[marker_to_box_dict[m.id.data]].box_pose_data.header = m.header
                        box_dict[marker_to_box_dict[m.id.data]].box_marker_data.header = m.header
                        box_dict[marker_to_box_dict[m.id.data]].markers_data[m.id.data] = MarkerData(marker, b_pos, b_rot)
                        box_dict[marker_to_box_dict[m.id.data]].probability = 1.0



    #ここでprobabilityが正しくなる

    #箱について
    for b in box_dict:
        if len(box_dict[b].markers_data) != 0:
            pos = np.array([0, 0, 0])
            rot = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
            probability_sum = 0
            for m in box_dict[b].markers_data:
                if box_dict[b].markers_data[m].probability > 0:
                    pos = pos + box_dict[b].markers_data[m].probability * box_dict[b].markers_data[m].box_pos_from_marker
                    rot = rot + box_dict[b].markers_data[m].probability * box_dict[b].markers_data[m].box_rot_from_marker
                    probability_sum = probability_sum + box_dict[b].markers_data[m].probability
            if probability_sum == 0:
                continue

            pos = pos / probability_sum
            rot = rot / probability_sum
            quat = quaternion.from_rotation_matrix(rot, nonorthogonal=True)

            if box_dict[b].initflag:
                box_dict[b].initflag = False
            elif np.linalg.norm(box_dict[b].pos) - np.linalg.norm(pos) > 0.50:
                pos = box_dict[b].pos
                rot = box_dict[b].rot
                quat = box_dict[b].quat
                print("pos jamping")
            box_dict[b].pos = pos
            box_dict[b].rot = rot
            box_dict[b].quat = quat


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

    #local座標の更新
    for b in box_dict:
        if len(box_dict[b].markers_data) != 0: #見てる箱のみ
            box_dict[b].up_to_down_update()
    for b in box_dict:
        if box_dict[b].state_update_flag == False:
            box_dict[b].down_to_up_update()

    #ズレの認識
    if lift_now:
        box_states = BoxStates()
        dangerous_safety = 0.6
        safety = 0.25
        modify_safety = 0.05
        modify_distance = box_dict[top_box_id].check_slip(dangerous_safety, safety, modify_safety, 0, np.array([0, 0, 0]), 0)
        if np.linalg.norm(modify_distance) > 100:
            rospy.loginfo("okanakya yabaiwayo!!")
            if okinaoshi_counter > 0:
                okinaoshi_counter -= 1
        elif np.linalg.norm(modify_distance) > 0:
            rospy.loginfo("yabaiwayo!!")
            if box_dict[top_box_id].check_modified_slip(safety, modify_distance, 0, np.array([0, 0, 0])):
                rospy.loginfo("okanakya yabaiwayo!!")
                if okinaoshi_counter > 0:
                    okinaoshi_counter -= 1
            else:
                okinaoshi_counter = 3
                rospy.loginfo("katamukereba iiwayo!!")
                if check_cooltime == 0:
                    msg = EmergencyCommand()
                    msg.mode = 0
                    msg.period = 0.3
                    msg.amp = 0.01
                    msg.ampr = 0.15
                    msg.x = modify_distance[0]
                    msg.y = modify_distance[1]
                    msg.z = modify_distance[2]
                    emergency_command_pub.publish(msg)
                    check_cooltime = 30
        elif check_cooltime == 0:
            okinaoshi_counter = 3
            msg = EmergencyCommand()
            msg.mode = -1
            emergency_command_pub.publish(msg)
        else:
            okinaoshi_counter = 3


        box_states_pub.publish(box_states)
    if check_cooltime > 0:
        check_cooltime -= 1

    if okinaoshi_counter == 0:
        msg = EmergencyCommand()
        msg.mode = 1
        table_distance = -1
        table_id = 0
        for b in box_dict:
            if b >= 50 and (table_distance < 0 or np.linalg.norm(box_dict[b].pos) < table_distance):
                table_distance = np.linalg.norm(box_dict[b].pos)
                table_id = b
        if table_id >= 50:
            msg.put_id = table_id
            emergency_command_pub.publish(msg)
        else:
            msg.put_id = 50
            emergency_command_pub.publish(msg)
            print("no table")

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
            if box_dict[b].markers_data[m].probability >= 0:
                box_dict[b].markers_data[m].marker_data.action = Marker.ADD
                markers_data.markers.append(box_dict[b].markers_data[m].marker_data)
                box_dict[b].markers_data[m].probability -= 0.4
            else:
                box_dict[b].markers_data[m].marker_data.action = Marker.DELETE
                ignore_marker_dict.pop(m)
                markers_data.markers.append(box_dict[b].markers_data[m].marker_data)
                marker_del_list.append(m)
        for m in marker_del_list:
            box_dict[b].markers_data.pop(m)
        #箱
        if box_dict[b].probability > 0:
            box_poses_data.header.stamp = box_dict[b].box_pose_data.header.stamp
            box_dict[b].probability -= 0.4
        elif box_dict[b].probability > -5:
            box_dict[b].disappear()
            box_dict[b].probability = -10
        #br = tf.TransformBroadcaster()
        #br.sendTransform(
        #    (box_dict[b].pos[0],
        #     box_dict[b].pos[1],
        #     box_dict[b].pos[2]),
        #    (box_dict[b].quat.x,
        #     box_dict[b].quat.y,
        #     box_dict[b].quat.z,
        #     box_dict[b].quat.w),
        #    rospy.Time.now(), "box"+str(b), marker_frame_id.lstrip())
        if box_look_flag:
            box_dict[b].box_pose_data_update()
            box_poses_data.existence = True
            box_poses_data.poses.append(box_dict[b].box_pose_data)
        box_dict[b].marker_pose_update()
        markers_data.markers.append(box_dict[b].box_marker_data)

    if goal_box.probability > 0:
        goal_box.box_marker_data.action = Marker.ADD
        #br = tf.TransformBroadcaster()
        #br.sendTransform(
        #    (goal_box.pos[0],
        #     goal_box.pos[1],
        #     goal_box.pos[2]),
        #    (goal_box.quat.x,
        #     goal_box.quat.y,
        #     goal_box.quat.z,
        #     goal_box.quat.w),
        #    rospy.Time.now(), "goal_box", marker_frame_id.lstrip())
        #markers_data.markers.append(goal_box.box_marker_data)
        goal_box.probability -= 0.3
    elif goal_box.probability > -5:
        goal_box.box_marker_data.action = Marker.DELETE
        goal_box.probability = -10
        #markers_data.markers.append(goal_box.box_marker_data)

    markers_pub.publish(markers_data)

    delay_time = rospy.Time.now() - looked_time
    delay = delay_time.secs * 1000 + delay_time.nsecs / 1000000
    #rospy.loginfo(" delay: " + str(delay) + "ms")
    box_poses_data.delay = delay
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
        look_timer -= 0
        if look_timer > 40: #上の箱見る
            if base_box_id in box_dict:
                bid = base_box_id
                blocal = np.array([-box_info[base_box_id]['size'][0]/2.0, 0, box_info[base_box_id]['size'][2]/2.0])
            elif top_box_id in box_dict:
                bid = top_box_id
                blocal = np.array([-box_info[top_box_id]['size'][0]/2.0, 0, -box_info[top_box_id]['size'][2]/2.0])
            elif hold_box_id in box_dict:
                bid = hold_box_id
                blocal = np.array([-box_info[hold_box_id]['size'][0]/2.0, 0, box_info[hold_box_id]['size'][2]/2.0])
        else: #下の箱見る
            if hold_box_id in box_dict:
                bid = hold_box_id
                blocal = np.array([-box_info[hold_box_id]['size'][0]/2.0, 0, 0])
            elif base_box_id in box_dict:
                bid = base_box_id
                blocal = np.array([-box_info[base_box_id]['size'][0]/2.0, 0, -box_info[base_box_id]['size'][2]/2.0])
            elif top_box_id in box_dict:
                bid = top_box_id
                blocal = np.array([-box_info[top_box_id]['size'][0]/2.0, 0, -box_info[top_box_id]['size'][2]/2.0])
            if look_timer <= 0:
                look_timer = 200
    elif look_box_mode == "put-box":
        if put_box_id in box_dict:
            bid = put_box_id
            blocal = np.array([-box_info[hold_box_id]['size'][0]/2.0, 0, box_info[hold_box_id]['size'][2]/2.0])
        elif hold_box_id in box_dict:
            bid = hold_box_id
            blocal = np.array([-box_info[hold_box_id]['size'][0]/2.0, 0, -box_info[hold_box_id]['size'][2]/2.0])
    
    #tmp
    #if 51 in box_dict:
	#bid = 51
	#blocal = np.array([-0.2, 0, 0])
    if bid > 0:
        look_at_data.new_target(bid, blocal)
    look_at_data.publish()

    d_time = rospy.Time.now()

    #持つ箱について手の位置・体の位置の目標TFを出力
    cb_count += 1
    if cb_count%3 == 0:
        tag = 'rhand_pose'
    elif cb_count%3 == 1:
        tag = 'lhand_pose'
    elif cb_count%3 == 2:
        tag = 'body_pose'
    if cb_count <= 3:
        if hold_box_id in box_dict:
            br = tf.TransformBroadcaster()
            tfpos, tfrot = box_dict[hold_box_id].local_to_camera(np.array(box_info[hold_box_id][tag]['pos']), np.array(box_info[hold_box_id][tag]['rot']))
            tfquat = quaternion.from_rotation_matrix(tfrot, nonorthogonal=True)
            br.sendTransform(
                (tfpos[0], tfpos[1], tfpos[2]),
                (tfquat.x, tfquat.y, tfquat.z, tfquat.w),
                rospy.Time.now(), tag, marker_frame_id.lstrip())
    else:
        if goal_box.probability > 0:
            br = tf.TransformBroadcaster()
            tfpos, tfrot = goal_box.local_to_camera(np.array(box_info[hold_box_id][tag]['pos']), np.array(box_info[hold_box_id][tag]['rot']))
            tfquat = quaternion.from_rotation_matrix(tfrot, nonorthogonal=True)
            br.sendTransform(
                (tfpos[0], tfpos[1], tfpos[2]),
                (tfquat.x, tfquat.y, tfquat.z, tfquat.w),
                rospy.Time.now(), 'goal_'+tag, marker_frame_id.lstrip())
    if cb_count == 6:
        cb_count = 0

    e_time = rospy.Time.now()
    a_t = (a_time - start_time).secs + float((a_time - start_time).nsecs) / 1000000000
    b_t = (b_time - a_time).secs + float((b_time - a_time).nsecs) / 1000000000
    c_t = (c_time - b_time).secs + float((c_time - b_time).nsecs) / 1000000000
    d_t = (d_time - c_time).secs + float((d_time - c_time).nsecs) / 1000000000
    e_t = (e_time - d_time).secs + float((e_time - d_time).nsecs) / 1000000000
    all_t = (e_time - start_time).secs + float((e_time - start_time).nsecs) / 1000000000
    #if all_t > 0 :
    #    rospy.loginfo(
    #        " " + "{:.3f}".format(all_t) + "  "
    #        " " + "{:.3f}".format(a_t) + " "
    #        " " + "{:.3f}".format(b_t) + " "
    #        " " + "{:.3f}".format(c_t) + " "
    #        " " + "{:.3f}".format(d_t) + " "
    #        " " + "{:.3f}".format(e_t) + "  "
    #        " " + "{:.2f}".format(a_t / all_t) + " "
    #        " " + "{:.2f}".format(b_t / all_t) + " "
    #        " " + "{:.2f}".format(c_t / all_t) + " "
    #        " " + "{:.2f}".format(d_t / all_t) + " "
    #        " " + "{:.2f}".format(e_t / all_t) + " "
    #        )

def mode_cb(msg):
    global look_box_mode, look_timer
    look_box_mode = msg.data
    if look_box_mode == 'box-balancer':
        look_timer = 200
    print(look_box_mode)

def read_box_id(msg):
    global top_box_id, base_box_id, hold_box_id, put_box_id
    top_box_id = rospy.get_param("/boxpose_pub/top_box_id", 7)
    base_box_id = rospy.get_param("/boxpose_pub/base_box_id", 8)
    hold_box_id = rospy.get_param("/boxpose_pub/hold_box_id", 9)
    put_box_id = rospy.get_param("/boxpose_pub/put_box_id", 8)

def handle_lift_box(req):
    global box_dict, lift_now, check_cooltime
    print(req)
    box_list = req.boxes
    for bid in box_list:
        if not bid in box_info.keys():
            return LiftBoxResponse(-1)
    if len(box_list) == 0:
        lift_now = False
    else:
        lift_now = True
    check_cooltime = 10
    #持ち上げた箱の重さと持ち上げた/置いた時の処理
    weight = 0.0
    for b in box_dict:
        #持ち上げた箱
        if b in box_list:
            weight += box_info[b]['mass']
            box_dict[b].lift = True
        #持ち上げてない箱
        else:
            box_dict[b].change_fixed_id(-1)
            box_dict[b].lift = False
    #box同士の接続
    for i in range(len(box_list)):
        if i == 0:
            box_dict[box_list[i]].change_fixed_id(-2)
        else:
            box_dict[box_list[i]].change_fixed_id(box_list[i-1])
        if i == len(box_list) - 1:
            box_dict[box_list[i]].on_id = -2
        else:
            box_dict[box_list[i]].on_id = box_list[i+1]
    print("weight : " + str(weight))
    for b in box_dict:
        print(str(b) + " : " + str(box_dict[b].fixed_id))
    return LiftBoxResponse(weight)

s = rospy.Service('lift_box_id', LiftBox, handle_lift_box)

if marker_type == "ar":
    print("ar mode")
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)
elif marker_type == "stag":
    print("stag mode")
    rospy.Subscriber("stag_ros/markers", STagMarkerArray, callback)
rospy.Subscriber("look_box_mode", String, mode_cb)
rospy.Subscriber("update_box_id", Bool, read_box_id)
looked_time = rospy.Time.now()
print("start")
rospy.spin()
