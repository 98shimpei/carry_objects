#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(ros::load-ros-manifest "jsk_recognition_msgs")

(jaxon_red-init)

(defun printbox (msg)
  (setq box (car (send msg :boxes)))
  (format t "printbox ~A~%" (send (car (send msg :boxes)) :pose :position :x))
  (let* ((cam->obj-coords (ros::tf-pose->coords (send box :pose)))
         (cam-coords (send (car (send *jaxon_red* :cameras)) :copy-worldcoords)))
    (print (send cam-coords :transform cam->obj-coords))
    )
  )

(defun camera-subscribe ()
  (ros::roseus "box-listener")
  (ros::subscribe "multisense_local/resize_1_1/boxes" jsk_recognition_msgs::BoundingBoxArray #'printbox)
  (ros::spin))
