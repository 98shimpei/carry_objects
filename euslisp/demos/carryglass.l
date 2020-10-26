#!/usr/bin/env roseus

(load "package://carry_objects/euslisp/jaxonred.l")
(load "package://carry_objects/euslisp/utils.l")

(defun start (&optional (simulation nil))
  (setq *robot* *jaxon_red*)
  (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  (send *ri* :angle-vector (send *ri* :state :potentio-vector))
  (send *ri* :stop-st)
  (send *ri* :stop-auto-balancer)
  (if simulation (start-footguided-modification) (start-footguided-modification-zikki))
  (send *ri* :set-auto-balancer-param
             :is-hand-fix-mode nil
             :ik-mode 1)
  (if (not simulation) (let ()
    (print "press key to start-auto-balancer")
    (read-line)
    ))
  (send *ri* :start-auto-balancer :limbs '(:rarm :rleg :lleg))
  (if (not simulation) (let ()
    (print "press key to start-st")
    (read-line)
    ))
  (send *ri* :start-st)
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
