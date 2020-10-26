#!/usr/bin/env roseus

;; robotの初期化
(load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(jaxon_red-init)

;; ObjectDetection型トピックを使うため
(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "roseus")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "image_view2")
(ros::roseus-add-msgs "posedetection_msgs")
(ros::roseus-add-msgs "hrpsys_ros_bridge")
(ros::roseus-add-msgs "ar_track_alvar_msgs")

(setq use-display (ros::get-param "~use_display" t))

;;; 表示モデルなど
(load "models/chessboard-30-7x5-object.l")
(if use-display (if (not (boundp '*irtviewer*)) (make-irtviewer)))
(setq *target-object* (chessboard-30-7x5 :name "/test_object"))
(if use-display (objects (list *target-object* *jaxon_red*)))

(setq *tfb* (instance ros::transform-broadcaster :init))

(ros::roseus "objectdetection_client")

;; ObjectDetection用コールバック関数定義
(defun objectdetection-cb (msg)
  (print "cb")
  ;; 物体モデルを配置
  (setq box-poses-msg (instance hrpsys_ros_bridge::BoxPoses :init))
  (send box-poses-msg :existence nil)
  (setq box-poses-list nil)
  (send *jaxon_red* :angle-vector (send *ri* :state :potentio-vector))
  (setq dummy-robot *jaxon_red*)
  (send dummy-robot :angle-vector (send *ri* :reference-vector))
  (mapcar #'(lambda (obj-pose)
    (let* (;; (1) カメラ相対の座標系は、geometry_msgs/Poseという型で得られるので、Euslispのcoordsに変換する
      (cam->obj-coords (ros::tf-pose->coords (send (send obj-pose :pose) :pose)))
      ;; (2) *jaxon_red*モデルがカメラの座標系をもってるので、取得する
      (cam-coords (send (car (send *jaxon_red* :cameras)) :copy-worldcoords))
      (box-pose (instance hrpsys_ros_bridge::BoxPose :init)))
      ;; (3) Euslisp内部でのworld座標系の値にして、そこにcheckerboardモデルを配置する
      (send *target-object* :newcoords (send cam-coords :transform cam->obj-coords))
      (send box-pose :px (aref (send *target-object* :pos) 0))
      (send box-pose :py (aref (send *target-object* :pos) 1))
      (send box-pose :pz (aref (send *target-object* :pos) 2))
      ;; lispのquaternionは(w, x, y, z)の順
      (send box-pose :rx (aref (matrix2quaternion (send *target-object* :rot)) 1))
      (send box-pose :ry (aref (matrix2quaternion (send *target-object* :rot)) 2))
      (send box-pose :rz (aref (matrix2quaternion (send *target-object* :rot)) 3))
      (send box-pose :rw (aref (matrix2quaternion (send *target-object* :rot)) 0))
      (send box-pose :id (send obj-pose :id))
      (send box-poses-msg :existence t)
      (setq box-poses-list (cons box-pose box-poses-list))
      (if (= (send obj-pose :id) 7) (send dummy-robot :head :look-at (send *target-object* :pos)))
      ))
    (send msg :markers))
  (send box-poses-msg :poses box-poses-list)
  (ros::publish "box_pose" box-poses-msg)
  ;; (send *ri* :angle-vector (send dummy-robot :angle-vector))
  )

(ros::advertise "box_pose" hrpsys_ros_bridge::BoxPoses 1)
(ros::subscribe "/ar_pose_marker" ar_track_alvar_msgs::AlvarMarkers #'objectdetection-cb)

(ros::rate 10)
(do-until-key
  (ros::spin-once)
  (if use-display (send *irtviewer* :draw-objects))
  (if use-display (x::window-main-one))
  (ros::sleep)
  )

