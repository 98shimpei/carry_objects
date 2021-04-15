#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
import rospy
import math
import tf
import cv2
import yaml
import numpy as np
import time
import quaternion
import geometry_msgs.msg
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


rospy.init_node('objects_trakker')
dummy_image_pub = rospy.Publisher("dummy_image", Image, queue_size = 1)

sift = cv2.xfeatures2d.SIFT_create()
def mysift(img):
    keypoints, descriptors = sift.detectAndCompute(img, None)
    img_sift = cv2.drawKeypoints(img, keypoints, None, flags=4)
    cv2.imshow('image', img_sift)
    cv2.waitKey(1)

tracker = cv2.TrackerKCF_create()
tracker_name = str(tracker).split()[0][1:]
init_flag = True
def cvtracker(img):
    global init_flag
    if init_flag:
        bbox = (0, 0, 10, 10)
        bbox = cv2.selectROI(img, False)
        ok = tracker.init(img, bbox)
        if ok:
            init_flag = False
        return
    track, bbox = tracker.update(img)
    if track:
        p1 = (int(bbox[0]), int(bbox[1]))
        p2 = (int(bbox[0] + bbox[2]), int (bbox[1] + bbox[3]))
        cv2.rectangle(img, p1, p2, (0, 255, 0), 2, 1)
    else:
        print("error")

    cv2.imshow('image', img)
    cv2.waitKey(1)


def callback(msg):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(msg, "bgr8")
    img = cv2.resize(img, (int(img.shape[1]/2), int(img.shape[0]/2)))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #edges = cv2.Canny(img, 50, 80)

    mysift(img)

    #cv2.imshow('image', edges)
    #cv2.waitKey(1)
    #pubmsg = bridge.cv2_to_imgmsg(img, encoding="passthrough")
    #dummy_image_pub.publish(pubmsg)

rospy.Subscriber("/camera/color/image_rect_color", Image, callback)
rospy.spin()
