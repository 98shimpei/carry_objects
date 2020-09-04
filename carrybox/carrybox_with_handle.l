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
(send *ri* :set-ref-forces
    (list
     #f(0 0 650) ;;rleg 1300
     #f(0 0 650) ;;lleg 1300
     #f(0 0 -2) ;;rarm -260
     #f(0 0 -2) ;;larm -285
     )
     1000)
(unix::sleep 2)
(print "press key to start-auto-balancer")
(read-line)
(send *ri* :start-auto-balancer :limbs '(:rarm :larm :rleg :lleg))
(print "press key to start-st")
(read-line)
(send *ri* :start-st)

(defun lift-box ()
  (send *ri* :start-impedance :arms :K-r 200.0 :D-r 100.0 :M-r 1
                                    :K-p 100.0 :D-p 100.0 :M-p 1) ;; 10000 / 300
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(570 200 1050) :rpy (list 0 0 0)))
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(570 -200 1050) :rpy (list 0 0 0)))
  (send *ri* :angle-vector (send *robot* :angle-vector) 1500)
  (send *ri* :wait-interpolation)
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(570 200 1150) :rpy (list 0 0 0)))
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(570 -200 1150) :rpy (list 0 0 0)))
  (send *ri* :angle-vector (send *robot* :angle-vector) 1500)
  (send *ri* :wait-interpolation)
  (send *ri* :set-ref-forces
      (list
       #f(0 0 650) ;;rleg 1300
       #f(0 0 650) ;;lleg 1300
       #f(0 0 -42) ;;rarm -260
       #f(0 0 -42) ;;larm -285
       )
       3000)
  (unix::sleep 4)
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(570 200 1300) :rpy (list 0 0 0)))
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(570 -200 1300) :rpy (list 0 0 0)))
  (send *ri* :angle-vector (send *robot* :angle-vector) 1500)
  (send *ri* :wait-interpolation)
  
;;  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(630 200 1300) :rpy (list 0 0 0)))
;;  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(630 -200 1300) :rpy (list 0 0 0)))
;;  (send *ri* :angle-vector (send *robot* :angle-vector) 1500)
;;  (send *ri* :wait-interpolation)
  
  )

(defun put-box ()
  (send *ri* :set-ref-forces
      (list
       #f(0 0 650) ;;rleg 1300
       #f(0 0 650) ;;lleg 1300
       #f(0 0 -2) ;;rarm -260
       #f(0 0 -2) ;;larm -285
       )
       3000)
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(570 200 1300) :rpy (list 0 0 0)))
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(570 -200 1300) :rpy (list 0 0 0)))
  (send *ri* :angle-vector (send *robot* :angle-vector) 1500)
  (send *ri* :wait-interpolation)
  )

(defun hoge ()
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(630 150 1050) :rpy (list 0 0 0)))
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(630 -150 1050) :rpy (list 0 0 0)))
  (send *ri* :angle-vector (send *robot* :angle-vector) 1500)
  (send *ri* :wait-interpolation)
)
(defun force->vec (force)
  (float-vector (send force :x) (send force :y) (send force :z)))

(defun update-larm (msg)
  (setq *hogemsg* msg)
  (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  (setq *larm-force* (force->vec (send msg :Wrench :force)))
  (setq *now_time* (send msg :Header :seq))
  (setq lh-coords (send (car (send *robot* :force-sensors)) :copy-worldcoords))
  (setq *larm-force* (matrix-row (m* (matrix *larm-force*) (transpose (send lh-coords :rot))) 0))
  (if (not (boundp '*larm-force-offset*)) (progn (setq *larm-force-offset* *larm-force* ) (setq *start_time* *now_time*)))
  (setq *larm-force* (v- *larm-force* *larm-force-offset*))
  (setq *lflag* nil)
  t
  )

(defun update-rarm (msg)
  (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  (setq *rarm-force* (force->vec (send msg :Wrench :force)))
  (setq rh-coords (send (cadr (send *robot* :force-sensors)) :copy-worldcoords))
  (setq *rarm-force* (matrix-row (m* (matrix *rarm-force*) (transpose (send rh-coords :rot))) 0))
  (if (not (boundp '*rarm-force-offset*)) (setq *rarm-force-offset* *rarm-force*))
  (setq *rarm-force* (v- *rarm-force* *rarm-force-offset*))
  (setq *rflag* nil)
  t
  )

(defun coop ()
  (send *ri* :start-impedance :arms :K-r 200.0 :D-r 100.0 :M-r 1
                                    :K-p 100.0 :D-p 50.0 :M-p 1) ;; 10000 / 300
  (ros::roseus "coop")
  (setq *larm-force* #f(0 0 0))
  (setq *rarm-force* #f(0 0 0))
  (setq *larm-force-1* #f(0 0 0))
  (setq *rarm-force-1* #f(0 0 0))
  (setq *larm-force-2* #f(0 0 0))
  (setq *rarm-force-2* #f(0 0 0))
  (setq *before-time* 0.0)
  (setq *lflag* t)
  (setq *rflag* t)
  (ros::subscribe "lhsensor" geometry_msgs::WrenchStamped #'update-larm)
  (ros::subscribe "rhsensor" geometry_msgs::WrenchStamped #'update-rarm)
  (ros::spin-once)
  
  (print "press key to coop_mode")
  (read-line)

  (while (ros::ok)
    (while (or *lflag* *rflag*) (ros::spin-once))
    (setq *lflag* t)
    (setq *rflag* t)
    (setq F 2.0)
    (setq *now_time* (/ (float *now_time*) 100.0))
    (setq dt (- *now_time* *before-time*))
    (setq tmpl (v+ (scale (- 1 (* dt F)) *larm-force-2*) (scale (* F dt) *larm-force-1*)))
    (setq tmpr (v+ (scale (- 1 (* dt F)) *rarm-force-2*) (scale (* F dt) *rarm-force-1*)))
    (setq tmpforce (v+ tmpl tmpr))
    (if (< (norm tmpforce) 6.0) (setq tmpforce #f(0 0 0)))
    (send *ri* :go-velocity (* (elt tmpforce 0) 0.1) (* (elt tmpforce 1) 0.1) 0)
;;    (if (> tmpforce 6)
;;      (send *ri* :go-velocity 0 (* tmpforce 0.1) 0)
;;      (if (< tmpforce -6)
;;        (send *ri* :go-velocity 0 (* tmpforce 0.1) 0)
;;        (send *ri* :go-velocity 0 0 0)))
    (format t "~A ~%"
;;      (/ (+ (elt *larm-force* 0) (elt *larm-force-1* 0) (elt *larm-force-2* 0)) 3.0)
;;      (/ (+ (elt *larm-force* 1) (elt *larm-force-1* 1) (elt *larm-force-2* 1)) 3.0)
;;      (/ (+ (elt *larm-force* 2) (elt *larm-force-1* 2) (elt *larm-force-2* 2)) 3.0)
;;      (/ (+ (elt *rarm-force* 0) (elt *rarm-force-1* 0) (elt *rarm-force-2* 0)) 3.0)
;;      (/ (+ (elt *rarm-force* 1) (elt *rarm-force-1* 1) (elt *rarm-force-2* 1)) 3.0)
;;      (/ (+ (elt *rarm-force* 2) (elt *rarm-force-1* 2) (elt *rarm-force-2* 2)) 3.0)
      (+ (elt tmpl 1) (elt tmpr 1))
      )
    (setq *larm-force-2* tmpl)
    (setq *larm-force-1* *larm-force*)
    (setq *rarm-force-2* tmpr)
    (setq *rarm-force-1* *rarm-force*)
    (setq *before-time* *now_time*)
    )
  )

