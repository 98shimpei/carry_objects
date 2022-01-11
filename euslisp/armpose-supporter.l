#!/usr/bin/env roseus

(load "package://carry_objects/euslisp/jaxonred.l")
(load "package://carry_objects/euslisp/utils.l")
(ros::load-ros-manifest "roseus")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "carry_objects")


(ros::roseus "armpose_supporter_eus")
(setq *current-robot* (instance jaxon_red-robot :init))
(send *current-robot* :fix-leg-to-coords (make-coords))

(defun callback (msg)
  (objects (list *robot*))
  (send *robot* :rarm :angle-vector (v+ (send *robot* :rarm :angle-vector) (send msg :rarm)))
  (send *robot* :larm :angle-vector (v+ (send *robot* :larm :angle-vector) (send msg :larm)))
  (if (or (= (send msg :mode) 1) (= (send msg :mode) 2)) (send *ri* :angle-vector (send *robot* :angle-vector)))
  (if (= (send msg :mode) 3) (print (send *robot* :arms :angle-vector)))
  (mv)
  (x::window-main-one)
)

(ros::rate 100)
(ros::subscribe "keystate" carry_objects::KeyState #'callback)
(ros::spin)
