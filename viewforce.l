#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(ros::load-ros-manifest "jsk_recognition_msgs")

(jaxon_red-init)
(setq *robot* *jaxon_red*)

(defun force->vec (force)
  (float-vector (send force :x) (send force :y) (send force :z)))

(defun update-larm (msg)
  (setq *larm-force* (force->vec (send msg :Wrench :force)))
  (setq lh-coords (send (car (send *robot* :force-sensors)) :copy-worldcoords))
  (setq *larm-force* (matrix-row (m* (matrix *larm-force*) (transpose (send lh-coords :rot))) 0))
  (if (not (boundp '*larm-force-offset*)) (setq *larm-force-offset* *larm-force*))
  ;; (setq *larm-force* (v- *larm-force* *larm-force-offset*))
  
  (setq *larm-torque* (force->vec (send msg :Wrench :torque)))
  (setq lh-coords (send (car (send *robot* :force-sensors)) :copy-worldcoords))
  (setq *larm-torque* (matrix-row (m* (matrix *larm-torque*) (transpose (send lh-coords :rot))) 0))
  (if (not (boundp '*larm-torque-offset*)) (setq *larm-torque-offset* *larm-torque*))
  (setq *larm-torque* (v- *larm-torque* *larm-torque-offset*))

  (setq *lflag* nil)
  t
  )

(defun update-rarm (msg)
  (setq *rarm-force* (force->vec (send msg :Wrench :force)))
  (setq rh-coords (send (cadr (send *robot* :force-sensors)) :copy-worldcoords))
  (setq *rarm-force* (matrix-row (m* (matrix *rarm-force*) (transpose (send rh-coords :rot))) 0))
  (if (not (boundp '*rarm-force-offset*)) (setq *rarm-force-offset* *rarm-force*))
  ;; (setq *rarm-force* (v- *rarm-force* *rarm-force-offset*))
  (setq *rflag* nil)
  t
  )

(defun force-subscribe ()
  (ros::roseus "force-listener")
  (setq *larm-force* #f(0 0 0))
  (setq *rarm-force* #f(0 0 0))
  (setq *larm-torque* #f(0 0 0))
  (setq *rarm-torque* #f(0 0 0))
  (setq *lflag* t)
  (setq *rflag* t)
  (ros::subscribe "lhsensor" geometry_msgs::WrenchStamped #'update-larm)
  (ros::subscribe "rhsensor" geometry_msgs::WrenchStamped #'update-rarm)
  (ros::spin-once)
  (while (ros::ok)
    (while (or *lflag* *rflag*) (ros::spin-once))
    (setq *lflag* t)
    (setq *rflag* t)
    (setq init (norm (v+ *larm-force-offset* *rarm-force-offset*)))
    (setq raw (norm (v+ *larm-force* *rarm-force*)))
    (format t "~A ~A~%" raw (- raw init))
    ;; (print *larm-torque*)
    )
  )
