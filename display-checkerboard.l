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

;;; 表示モデルなど
(load "models/chessboard-30-7x5-object.l")
(if (not (boundp '*irtviewer*)) (make-irtviewer))
(setq *target-object* (chessboard-30-7x5 :name "/test_object"))
(objects (list *target-object* *jaxon_red*))

(setq *tfb* (instance ros::transform-broadcaster :init))

(ros::roseus "objectdetection_client")

;; ObjectDetection用コールバック関数定義
;; (defun objectdetection-cb (msg)
;;   (let ((mrk (instance image_view2::ImageMarker2 :init)) (box-pose (instance hrpsys_ros_bridge::QuatPose :init)) frame-id type ret)
;;     ;; 物体モデルを配置
;;     ;; (setq frame-id (concatenate string "/" (send msg :header :frame_id)))
;;     (mapcar #'(lambda (obj-pose)
;;                 (let* (;; (1) カメラ相対の座標系は、geometry_msgs/Poseという型で得られるので、Euslispのcoordsに変換する
;;                        (cam->obj-coords (ros::tf-pose->coords (send obj-pose :pose)))
;;                        ;; (2) *jaxon_red*モデルがカメラの座標系をもってるので、取得する
;;                        (cam-coords (send (car (send *jaxon_red* :cameras)) :copy-worldcoords)))
;;                   ;; (3) Euslisp内部でのworld座標系の値にして、そこにcheckerboardモデルを配置する
;;                   (send *target-object* :newcoords (send cam-coords :transform cam->obj-coords))
;;                   ))
;;             (send msg :objects))
;;     ;; image_markerを出力
;;     ;; (dolist (obj-pose (send msg :objects))
;;     ;;   (setq type (send obj-pose :type))
;;     ;;   (unless (eq (char type 0) #\/) (setq type (concatenate string "/" type)))
;;     ;;   (setq ret (ros::tf-pose->coords (send obj-pose :pose)))
;;     ;;   (send mrk :type image_view2::ImageMarker2::*FRAMES*)
;;     ;;   (send mrk :frames (list type))
;;     ;;   (send *tfb* :send-transform ret frame-id type)
;;     ;;   (ros::ros-info "~A ~A ~A" ret frame-id type)
;;     ;;   (ros::publish "image_marker" mrk)
;;     ;;   )
;; 
;;     (dolist (obj-pose (send msg :objects))
;;       (send box-pose :px (aref (send *target-object* :pos) 0))
;;       (send box-pose :py (aref (send *target-object* :pos) 1))
;;       (send box-pose :pz (aref (send *target-object* :pos) 2))
;;       ;; lispのquaternionは(w, x, y, z)の順
;;       (send box-pose :rx (aref (matrix2quaternion (send *target-object* :rot)) 1))
;;       (send box-pose :ry (aref (matrix2quaternion (send *target-object* :rot)) 2))
;;       (send box-pose :rz (aref (matrix2quaternion (send *target-object* :rot)) 3))
;;       (send box-pose :rw (aref (matrix2quaternion (send *target-object* :rot)) 0))
;;       (ros::publish "box_pose" box-pose)
;;       )
;;     ))

(defun objectdetection-cb (msg)
  (print "cb")
  ;; 物体モデルを配置
  (setq box-poses-msg (instance hrpsys_ros_bridge::BoxPoses :init))
  (send box-poses-msg :existence nil)
  (setq box-poses-list nil)
  (mapcar #'(lambda (obj-pose)
    (let* (;; (1) カメラ相対の座標系は、geometry_msgs/Poseという型で得られるので、Euslispのcoordsに変
換する
      (cam->obj-coords (ros::tf-pose->coords (send obj-pose :pose)))
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
      (send box-pose :id 7)
      (send box-poses-msg :existence t)
      (setq box-poses-list (cons box-pose box-poses-list))
      ))
    (send msg :objects))
  (send box-poses-msg :poses box-poses-list)
  (ros::publish "box_pose" box-poses-msg)
  )

(ros::advertise "box_pose" hrpsys_ros_bridge::BoxPoses 1)
(ros::subscribe "/checkerdetector/ObjectDetection" posedetection_msgs::ObjectDetection #'objectdetection-cb)

(ros::rate 10)
(do-until-key
  (ros::spin-once)
  (send *irtviewer* :draw-objects)
  (x::window-main-one)
  (ros::sleep)
  )

