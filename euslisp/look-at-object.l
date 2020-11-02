#!/usr/bin/env roseus

;; robotの初期化
(load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(jaxon_red-init)

;; ObjectDetection型トピックを使うため
(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "roseus")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "posedetection_msgs")
(ros::roseus-add-msgs "ar_track_alvar_msgs")

(ros::roseus "look-at-object")
(send *jaxon_red* :fix-leg-to-coords (make-coords))

(defun cb (msg)
  (send *jaxon_red* :angle-vector (send *ri* :state :potentio-vector))
  (setq dummy-robot *jaxon_red*)
  (send dummy-robot :angle-vector (send *ri* :reference-vector))
  (mapcar #'(lambda (obj-pose)
    (let* (;; (1) カメラ相対の座標系は、geometry_msgs/Poseという型で得られるので、Euslispのcoordsに変換する
      (cam->obj-coords (ros::tf-pose->coords (send (send obj-pose :pose) :pose)))
      ;; (2) *jaxon_red*モデルがカメラの座標系をもってるので、取得する
      (cam-coords (send (car (send *jaxon_red* :cameras)) :copy-worldcoords)))
      ;; (3) Euslisp内部でのworld座標系の値にして、そこにcheckerboardモデルを配置する
      (setq *target-coords* (send cam-coords :transform cam->obj-coords))
      (if (= (send obj-pose :id) 7) (progn (send dummy-robot :head :look-at (v+ #f(0 0 100) (send *target-coords* :pos))) (print "hoge")))
      ))
    (send msg :markers))
  (if (boundp '*target-coords*) (print (send *target-coords* :pos)))
  (send *ri* :angle-vector (send dummy-robot :angle-vector))
  (send *ri* :wait-interpolation)
  )

(ros::subscribe "/ar_pose_marker" ar_track_alvar_msgs::AlvarMarkers #'cb)

(ros::rate 10)
(print "look at object start")
(do-until-key
  (ros::spin-once)
  (ros::sleep)
  )

