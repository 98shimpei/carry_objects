#!/usr/bin/env roseus

;; (load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
;; (jaxon_red-init)
(load "../jaxonred.l")
(load "../utils.l")
(setq *robot* *jaxon_red*)
(send *robot* :angle-vector (send *ri* :state :potentio-vector))
(send *ri* :angle-vector (send *ri* :state :potentio-vector))
(start-footguided-modification)
(send *ri* :set-auto-balancer-param
           :is-hand-fix-mode t
           :ik-mode 1)
(send *ri* :set-auto-balancer-param
    :use-force-mode :MODE-REF-FORCE-WITH-FOOT
    :additional-force-applied-link-name "RARM_JOINT7"
    :additional-force-applied-point-offset #F(0.1 0.3 0)
    )
(send *ri* :set-ref-forces
    (list
     #f(0 0 650) ;;rleg 1300
     #f(0 0 650) ;;lleg 1300
     #f(0 0 0) ;;rarm -260
     #f(0 0 0) ;;larm -285
     )
     1000)
(unix::sleep 2)
(send *ri* :start-auto-balancer :limbs '(:rarm :larm :rleg :lleg))
(send *ri* :start-st)

(defun update-larm-force (msg)
  (setq *larm-force* (send msg :Wrench :force :y))
  (print "hoge")
  )

(defun update-rarm-force (msg)
  (setq *rarm-force* (send msg :Wrench :force :y))
  (print "hoge")
  )

(defun lift-box ()
  (send *ri* :start-impedance :arms :K-r 1000.0 :D-r 200.0 :M-r 1
                                    :K-p 10000.0 :D-p 20000.0 :M-p 1)
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(480 -400 1100) :rpy (list 0 0 (deg2rad 90))))
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(480 400 1100) :rpy (list 0 0 (deg2rad -90))))
  (send *ri* :angle-vector (send *robot* :angle-vector) 1500)
  (send *ri* :wait-interpolation)
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(480 -400 1000) :rpy (list 0 0 (deg2rad 90))))
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(480 400 1000) :rpy (list 0 0 (deg2rad -90))))
  (send *ri* :angle-vector (send *robot* :angle-vector) 1500)
  (send *ri* :wait-interpolation)
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(480 -220 1000) :rpy (list 0 0 (deg2rad 90))))
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(480 220 1000) :rpy (list 0 0 (deg2rad -90))))
  (send *ri* :angle-vector (send *robot* :angle-vector) 1500)
  (send *ri* :wait-interpolation)
  (send *ri* :set-ref-forces
      (list
       #f(0 0 700) ;;rleg 1300
       #f(0 0 700) ;;lleg 1300
       #f(0 0 0) ;;rarm -260
       #f(0 0 0) ;;larm -285
       )
       3000)
  (unix::sleep 4)
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(480 -220 1200) :rpy (list 0 0 (deg2rad 90))))
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(480 220 1200) :rpy (list 0 0 (deg2rad -90))))
  (send *ri* :angle-vector (send *robot* :angle-vector) 1500)
  (send *ri* :wait-interpolation)
  )

(defun monitor-force (leg msg)
  (format t "~A ~A~%" leg (send msg :Wrench :force :z))
  )

(defun monitor-l-force (msg) (monitor-force :lleg msg))
(defun monitor-r-force (msg) (monitor-force :rleg msg))

(defun monitor-imu (msg)
  (setq now-rot (ros::tf-quaternion->rot (send msg :orientation)))
  (if *init-flag*
    (progn
      (setq *init-flag* nil)
      (setq *init-rot* now-rot))
    (progn
      (setq *rot-diff* (m* now-rot (inverse-matrix *init-rot*)))
      (objects (list *robot* (make-coords :pos #f(100 0 0) :rot *rot-diff*)))
      (format t "~A ~%" *rot-diff*)))
  )

(defun keep-hand-orientation ()
  (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  (objects (list *robot*))

  ;; (ros::roseus "force-listener")
  ;; (setq *init-flag* t)
  ;; (ros::subscribe "imu_rootlink" sensor_msgs::imu #'monitor-imu)

  (setq init-rot (ros::tf-quaternion->rot (send (send *ri* :state :imu) :orientation)))
  (setq init-pose (send *ri* :state :potentio-vector))
  (send *robot* :angle-vector init-pose)
  (setq init-coords (send *robot* :rarm :end-coords :copy-worldcoords))

  (setq hoge 1)
  (while t  
    (let* ((now-rot (ros::tf-quaternion->rot (send (send *ri* :state :imu) :orientation)))
           (rot-diff (m* now-rot (inverse-matrix init-rot))))
      ;; (send *robot* :angle-vector (send *ri* :state :potentio-vector))
      (setq hoge (* hoge -1))
      (send *robot* :angle-vector init-pose)
      (send *robot* :rarm :inverse-kinematics (send (make-coords :pos (send init-coords :pos) :rot (m* (inverse-matrix rot-diff) (send init-coords :rot))) :rotate (deg2rad (* hoge 5)) :x :world))
      (send *ri* :angle-vector (send *robot* :angle-vector) -1 nil 0 :min-time 0.5)
      (send *ri* :wait-interpolation)
      (objects (list *robot* (make-coords :pos #f(300 0 1000) :rot rot-diff)))
      (format t "~A ~%" rot-diff)
    ))
  )
