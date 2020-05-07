#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(jaxon_red-init)
(setq *robot* *jaxon_red*)

(defun nipbox (&optional (armangle 85))
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(430 350 1050) :rpy (list 0 0 (deg2rad 80))))
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(430 -350 1050) :rpy (list 0 0 (deg2rad -80))))
  (send *robot* :legs :move-end-pos #f(0 0 130) :world)
  (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (print "hoge")
  
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(430 250 1050) :rpy (list 0 0 (deg2rad armangle))))
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(430 -250 1050) :rpy (list 0 0 (deg2rad (* -1 armangle)))))
  (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (print "hoge")

  t
  )

(defun liftbox (&optional (armangle 85))
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(430 250 1200) :rpy (list 0 0 (deg2rad armangle))))
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(430 -250 1200) :rpy (list 0 0 (deg2rad (* -1 armangle)))))
  (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  (send *robot* :legs :move-end-pos #f(0 0 -130) :world)
  (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  t
  )

(defun move ()
  (send *ri* :set-gait-generator-param
             :default-step-time 0.7)
  (send *ri* :go-pos -0.5 0 0)
  (send *ri* :go-pos 0 2 -90)
  (send *ri* :go-pos 0.5 1.5 0)
)

(defun putbox ()
  (send *robot* :legs :move-end-pos #f(0 0 180) :world)
  (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  (send *robot* :arms :move-end-pos #f(0 0 -100) :world)
  (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  t
  )

(defun carrybox ()
  (nipbox)
  (liftbox)
  (move)
  (putbox)
  t
  )
