#!/usr/bin/env roseus

;; ObjectDetection型トピックを使うため
(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "roseus")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "image_view2")
(ros::roseus-add-msgs "posedetection_msgs")
(ros::roseus-add-msgs "hrpsys_ros_bridge")
(ros::roseus-add-msgs "carry_objects")

(setq use-display (ros::get-param "~use_display" t))

;;; 表示モデルなど
(if use-display (if (not (boundp '*irtviewer*)) (make-irtviewer)))

(ros::roseus "display_box_client")

(setq *color* 0)
;; ObjectDetection用コールバック関数定義
(defun cb (msg)
  (if (= *color* 0.0) (setq *color* 1.0) (setq *color* 0.0))
  (setq objlist (list 
    (make-cube 1000 1000 10 :coords (make-coords :pos #f(0 0 -100)))
    (make-cube 10 10 1000 :coords (make-coords :pos #f(500 500 450)))
    (make-cube 10 10 1000 :coords (make-coords :pos #f(500 -500 450)))
    (make-cube 10 10 1000 :coords (make-coords :pos #f(-500 500 450)))
    (make-cube 10 10 1000 :coords (make-coords :pos #f(-500 -500 450)))
    ))
  (mapcar #'(lambda (obj-pose) (progn
    (setq boxcoords (ros::tf-pose->coords (send obj-pose :pose)))
    (setq size (send obj-pose :size))
    (setq obj (make-cube (elt size 0) (elt size 1) (elt size 2) :coords boxcoords))
    (if (> (send obj-pose :color) 0.5) (send obj :set-color :pink) (send obj :set-color :pink))
    (if (= *color* (send obj-pose :color)) (setq objlist (append objlist (list obj))))))
    (send msg :boxstates))
  (objects objlist)
  )

(ros::subscribe "/box_states" carry_objects::BoxStates #'cb)

(ros::rate 30)
(do-until-key
  (ros::spin-once)
  (if use-display (send *irtviewer* :draw-objects))
  (if use-display (x::window-main-one))
  (ros::sleep)
  )

