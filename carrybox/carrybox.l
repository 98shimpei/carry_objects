#!/usr/bin/env roseus

(load "../jaxonred.l")
(load "../utils.l")

(defun start (&optional (simulation nil))
  (if simulation (setq *simulation_mode* t))
  (setq *robot* *jaxon_red*)
  (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  (send *ri* :angle-vector (send *robot* :reset-pose))
  (send *ri* :stop-st)
  (send *ri* :stop-auto-balancer)
  (send *ri* :set-auto-balancer-param
             :is-hand-fix-mode t
             :ik-mode 1)
  ;; (send *ri* :set-auto-balancer-param
  ;;     :use-force-mode :MODE-REF-FORCE-WITH-FOOT
  ;;     :additional-force-applied-link-name "RARM_JOINT7"
  ;;     :additional-force-applied-point-offset #F(0.1 0.3 0)
  ;;     )
  (send *ri* :set-ref-forces
      (list
       (float-vector 0 0 (* (send *robot* :weight) 4.9)) 
       (float-vector 0 0 (* (send *robot* :weight) 4.9)) 
       (float-vector 0 0 0) ;;rarm -260  mg/2
       (float-vector 0 0 0) ;;larm -285
       )
       3000)
  (if (not simulation) (let ()
    (print "press key to start-auto-balancer")
    (read-line)
    ))
  (send *ri* :start-auto-balancer :limbs '(:rarm :larm :rleg :lleg))
  (if simulation (start-footguided-modification) (start-footguided-modification-zikki))
  (if (not simulation) (let ()
    (print "press key to start-st")
    (read-line)
    ))
  (send *ri* :start-st)
  (setq *state* 'put)
  )

(defun lift-box (box_weight &optional (nip_force 50) (box_width 310))
  (unless (string= *state* 'put)
    (print "robot carries box now")
    (return-from lift-box nil))
  (setq *state* 'lift-box)
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(480 -400 1100) :rpy (list 0 0 (deg2rad -90)))) 
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(480 400 1100) :rpy (list 0 0 (deg2rad 90)))) 
  (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
  (send *ri* :wait-interpolation)
  (send *ri* :stop-impedance :arms)
  (send *ri* :start-impedance :arms :K-r 100.0 :D-r 100.0 :M-r 1
                                    :K-p 70.0 :D-p 70.0 :M-p 5)

  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(480 -400 1000) :rpy (list 0 0 (deg2rad -90)))) 
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(480 400 1000) :rpy (list 0 0 (deg2rad 90)))) 
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (print "press key to nip box")
  (read-line)
  (unix:sleep 10)
  (send *robot* :rarm :inverse-kinematics (make-coords :pos (float-vector 480 (/ box_width -2.0) 1000) :rpy (list 0 0 (deg2rad -90))))
  (send *robot* :larm :inverse-kinematics (make-coords :pos (float-vector 480 (/ box_width  2.0) 1000) :rpy (list 0 0 (deg2rad 90))))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (print "press key to set-ref-forces")
  (read-line)
  (send *ri* :set-ref-forces
      (list
       (float-vector 0 0 (* (+ (send *robot* :weight) box_weight) 4.9)) 
       (float-vector 0 0 (* (+ (send *robot* :weight) box_weight) 4.9)) 
       (float-vector 0 (* -1 nip_force) (* -1.0 (* box_weight 4.9))) ;;rarm -260  mg/2
       (float-vector 0 (*  1 nip_force) (* -1.0 (* box_weight 4.9))) ;;larm -285
       )
       3000)
  (unix::sleep 4)
  (send *robot* :rarm :inverse-kinematics (make-coords :pos (float-vector 520 (/ box_width -2.0) 1050) :rpy (list 0 0 (deg2rad -90))))
  (send *robot* :larm :inverse-kinematics (make-coords :pos (float-vector 520 (/ box_width  2.0) 1050) :rpy (list 0 0 (deg2rad 90))))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )

(defun lift-box-with-handle (box_weight)
  (unless (string= *state* 'put)
    (print "robot carries box now")
    (return-from lift-box-with-handle nil))
  (setq *state* 'lift-box-with-handle)
  (send *ri* :stop-impedance :arms)
  (send *ri* :start-impedance :arms :K-r 200.0 :D-r 200.0 :M-r 5
                                    :K-p 100.0 :D-p 100.0 :M-p 5) ;; 10000 / 300
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(470 100 1000) :rpy (list 0 0 0)))
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(470 -100 1000) :rpy (list 0 0 0)))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(470 100 1100) :rpy (list 0 0 0)))
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(470 -100 1100) :rpy (list 0 0 0)))
  (send *ri* :angle-vector (send *robot* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  (print "press key to set-ref-forces")
  (read-line)
  (unix:sleep 10)
  (send *ri* :set-ref-forces
      (list
       (float-vector 0 0 (* (+ (send *robot* :weight) box_weight) 4.9)) 
       (float-vector 0 0 (* (+ (send *robot* :weight) box_weight) 4.9)) 
       (float-vector 0 0 (* -1.0 (* box_weight 4.9))) ;;rarm -260  mg/2
       (float-vector 0 0 (* -1.0 (* box_weight 4.9))) ;;larm -285
       )
       3000)
  (unix::sleep 4)
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(470 100 1100) :rpy (list 0 0 0)))
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(470 -100 1100) :rpy (list 0 0 0)))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
;;  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(630 200 1100) :rpy (list 0 0 0)))
;;  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(630 -200 1100) :rpy (list 0 0 0)))
;;  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
;;  (send *ri* :wait-interpolation)
  
  )

(defun lift-box-side (box_weight)
  (unless (string= *state* 'put)
    (print "robot carries box now")
    (return-from lift-box-side nil))
  (setq *state* 'lift-box-side)
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(450 350 1100) :rpy (list 0 0 0)))
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(450 -350 1100) :rpy (list 0 0 0)))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (if *simulation_mode* (send *ri* :go-pos 0.1 0 0))
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(380 180 1100) :rpy (list (deg2rad -90) 0 0)))
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(380 -180 1100) :rpy (list (deg2rad 90) 0 0)))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :stop-impedance :arms)
  (send *ri* :start-impedance :arms :K-r 200.0 :D-r 200.0 :M-r 5
                                    :K-p 1000.0 :D-p 1000.0 :M-p 5) ;; 10000 / 300
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(380 180 1150) :rpy (list (deg2rad -90) 0 0)))
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(380 -180 1150) :rpy (list (deg2rad 90) 0 0)))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (print "press key to set-ref-forces")
  (read-line)
  (unix:sleep 10)
  (send *ri* :set-ref-forces
      (list
       (float-vector 0 0 (* (+ (send *robot* :weight) box_weight) 4.9)) 
       (float-vector 0 0 (* (+ (send *robot* :weight) box_weight) 4.9)) 
       (float-vector 0 0 (* -1.0 (* box_weight 4.9))) ;;rarm -260  mg/2
       (float-vector 0 0 (* -1.0 (* box_weight 4.9))) ;;larm -285
       )
       3000)
  (unix::sleep 4)
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(400 180 1250) :rpy (list (deg2rad -90) 0 0)))
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(400 -180 1250) :rpy (list (deg2rad 90) 0 0)))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (if *simulation_mode* (send *ri* :go-pos -1 0 0))
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(350 180 1050) :rpy (list (deg2rad -90) 0 0)))
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(350 -180 1050) :rpy (list (deg2rad 90) 0 0)))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
)

(defun lift-box-on-floor (box_weight &optional (nip_force 50) (box_width 310))
  (unless (string= *state* 'put)
    (print "robot carries box now")
    (return-from lift-box-on-floor nil))
  (setq *state* 'lift-box-on-floor)
  (send *ri* :stop-impedance :arms)
  (send *ri* :start-impedance :arms :K-r 200.0 :D-r 200.0 :M-r 5
                                    :K-p 1000.0 :D-p 500.0 :M-p 5) ;; 10000 / 300
  (send *robot* :reset-pose)
  (send *robot* :rarm :shoulder-r :joint-angle -80)
  (send *robot* :larm :shoulder-r :joint-angle 80)
  (send *robot* :torso-waist-p :joint-angle 30)
  (send *robot* :legs :crotch-p :joint-angle -35)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :legs :move-end-pos #f(150 0 400))
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(550 300 150) :rpy (list 0 (deg2rad 50) (deg2rad 90))))
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(550 -300 150) :rpy (list 0 (deg2rad 50) (deg2rad -90))))
  (send *ri* :angle-vector (send *robot* :angle-vector) 15000)
  (send *ri* :wait-interpolation)

  (print "press key to nip box")
  (read-line)
  (unix:sleep 10)
  (send *robot* :rarm :inverse-kinematics (make-coords :pos (float-vector 550 (/ box_width -2.0) 150) :rpy (list 0 (deg2rad 40) (deg2rad -90))))
  (send *robot* :larm :inverse-kinematics (make-coords :pos (float-vector 550 (/ box_width  2.0) 150) :rpy (list 0 (deg2rad 40) (deg2rad 90))))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (print "press key to set-ref-forces")
  (read-line)
  (send *ri* :set-ref-forces
      (list
       (float-vector 0 0 (* (+ (send *robot* :weight) box_weight) 4.9))
       (float-vector 0 0 (* (+ (send *robot* :weight) box_weight) 4.9))
       (float-vector 0 (* -1 nip_force) (* -1.0 (* box_weight 4.9))) ;;rarm -260  mg/2
       (float-vector 0 (*  1 nip_force) (* -1.0 (* box_weight 4.9))) ;;larm -285
       )
       3000)
  (unix::sleep 4)

  (send *robot* :rarm :inverse-kinematics (make-coords :pos (float-vector 550 (/ box_width -2.0) 400) :rpy (list 0 (deg2rad 40) (deg2rad -90))))
  (send *robot* :larm :inverse-kinematics (make-coords :pos (float-vector 550 (/ box_width  2.0) 400) :rpy (list 0 (deg2rad 40) (deg2rad 90))))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :rarm :inverse-kinematics (make-coords :pos (float-vector 500 (/ box_width -2.0) 950) :rpy (list 0 (deg2rad 40) (deg2rad -90))))
  (send *robot* :larm :inverse-kinematics (make-coords :pos (float-vector 500 (/ box_width  2.0) 950) :rpy (list 0 (deg2rad 40) (deg2rad 90))))
  (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
  (send *ri* :wait-interpolation)

  )

(defun put-box ()
  (when (string= *state* 'put)
    (print "robot does not carry box now")
    (return-from put-box nil))
  
  (when (string= *state* 'lift-box)
    (send *ri* :set-ref-forces
        (list
         (float-vector 0 0 (* (send *robot* :weight) 4.9)) 
         (float-vector 0 0 (* (send *robot* :weight) 4.9)) 
         (float-vector 0 0 0) ;;rarm -260  mg/2
         (float-vector 0 0 0) ;;larm -285
         )
         3000)
    (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(480 -350 1100) :rpy (list 0 0 (deg2rad -90))))
    (send *robot* :larm :inverse-kinematics (make-coords :pos #f(480 350 1100) :rpy (list 0 0 (deg2rad 90))))
    (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    (setq *state* 'put)
    (return-from put-box t))

  (when (string= *state* 'lift-box-with-handle)
    (send *ri* :set-ref-forces
        (list
         (float-vector 0 0 (* (send *robot* :weight) 4.9)) 
         (float-vector 0 0 (* (send *robot* :weight) 4.9)) 
         (float-vector 0 0 0) ;;rarm -260  mg/2
         (float-vector 0 0 0) ;;larm -285
         )
         3000)
    (send *robot* :larm :inverse-kinematics (make-coords :pos #f(470 150 1050) :rpy (list 0 0 0)))
    (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(470 -150 1050) :rpy (list 0 0 0)))
    (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    (setq *state* 'put)
    (return-from put-box t))
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
  ;; (if (not (boundp '*larm-force-offset*)) (progn (setq *larm-force-offset* *larm-force* ) (setq *start_time* *now_time*)))
  ;; (setq *larm-force* (v- *larm-force* *larm-force-offset*))
  (setq *lflag* nil)
  t
  )

(defun update-rarm (msg)
  (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  (setq *rarm-force* (force->vec (send msg :Wrench :force)))
  (setq rh-coords (send (cadr (send *robot* :force-sensors)) :copy-worldcoords))
  (setq *rarm-force* (matrix-row (m* (matrix *rarm-force*) (transpose (send rh-coords :rot))) 0))
  ;; (if (not (boundp '*rarm-force-offset*)) (setq *rarm-force-offset* *rarm-force*))
  ;; (setq *rarm-force* (v- *rarm-force* *rarm-force-offset*))
  (setq *rflag* nil)
  t
  )

(defun coop ()
  (when (string= *state* 'put)
    (send *robot* :larm :inverse-kinematics (make-coords :pos #f(470 200 1050) :rpy (list 0 0 0)))
    (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(470 -200 1050) :rpy (list 0 0 0)))
    (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    (print "press key to set impedance")
    (read-line)
    ;; (send *ri* :stop-impedance :arms)
    ;; (send *ri* :start-impedance :arms :K-r 200.0 :D-r 200.0 :M-r 5
    ;;                                :K-p 200.0 :D-p 250.0 :M-p 5) ;; 10000 / 300
    )
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
  (unix:sleep 10)

  ;; initialize
  (while (or *lflag* *rflag*) (ros::spin-once))
  (setq *lflag* t)
  (setq *rflag* t)
  (setq *now_time* (/ (float *now_time*) 300.0))
  (setq *larm-force-2* *larm-force*)
  (setq *larm-force-1* *larm-force*)
  (setq *rarm-force-2* *rarm-force*)
  (setq *rarm-force-1* *rarm-force*)
  (setq *before-time* *now_time*)

  (setq init_cycle 100)
  (setq F 2.0)
  (while (ros::ok)
    (while (or *lflag* *rflag*) (ros::spin-once))
    (setq *lflag* t)
    (setq *rflag* t)
    (setq *now_time* (/ (float *now_time*) 100.0))
    (setq dt (- *now_time* *before-time*))
    (setq tmpl (v+ (scale (- 1 (* dt F)) *larm-force-2*) (scale (* F dt) *larm-force-1*)))
    (setq tmpr (v+ (scale (- 1 (* dt F)) *rarm-force-2*) (scale (* F dt) *rarm-force-1*)))
    (setq tmpforce (v+ tmpl tmpr))

    (when (> init_cycle 0) (setq init_cycle (- init_cycle 1)))
    (when (= init_cycle 0) (setq init_cycle -1) (setq force_offset tmpforce) (print "coop init finish"))
    (when (< init_cycle 0)
      (setq tmpforce (v- tmpforce force_offset))
      (if (< (norm (float-vector (elt tmpforce 0) (elt tmpforce 1))) 6.0) (setq tmpforce #f(0 0 0)))
      (send *ri* :go-velocity (* (elt tmpforce 0) 0.008) (* (elt tmpforce 1) 0.008) 0)
      (format t "~A ~A ~%"
        (elt tmpforce 0)
        (elt tmpforce 1)
        (elt tmpforce 2)
        )
      )
    (setq *larm-force-2* tmpl)
    (setq *larm-force-1* *larm-force*)
    (setq *rarm-force-2* tmpr)
    (setq *rarm-force-1* *rarm-force*)
    (setq *before-time* *now_time*)
    )
  (print "coop")
  )

