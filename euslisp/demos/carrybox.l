#!/usr/bin/env roseus

(load "package://carry_objects/euslisp/jaxonred.l")
(load "package://carry_objects/euslisp/utils.l")
(ros::load-ros-manifest "roseus")
(ros::roseus-add-msgs "geometry_msgs")

(setq *impedance_mode* nil)

(defun start (&optional (simulation nil))
  (if simulation (setq *simulation_mode* t) (setq *simulation_mode* nil))
  (setq *robot* *jaxon_red*)
  ;; (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  ;; (send *ri* :angle-vector (send *robot* :reset-pose))
  (send *ri* :stop-st)
  (send *ri* :stop-auto-balancer)
  (send *ri* :set-auto-balancer-param
             :is-hand-fix-mode nil
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
  (if simulation (start-footguided-modification nil) (start-footguided-modification t))
  (if (not simulation) (let ()
    (print "press key to start-st")
    (read-line)
    ))
  (send *ri* :start-st)
  (send *ri* :remove-force-sensor-offset-rmfo-arms)
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *state* 'put)
  )

(defun lift-box (&optional (box_weight -1) (nip_force 50) (box_width 310))
  (unless (string= *state* 'put)
    (print "robot carries box now")
    (return-from lift-box nil))
  (setq *state* 'lift-box)
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(530 -350 1000) :rpy (list 0 0 (deg2rad -90)))) 
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(530 350 1000) :rpy (list 0 0 (deg2rad 90)))) 
  (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
  (send *ri* :wait-interpolation)
  (send *ri* :stop-impedance :arms)
  (send *ri* :start-impedance :arms :K-r 50.0 :D-r 50.0 :M-r 1
                                    :K-p 70.0 :D-p 70.0 :M-p 5)
  (setq *impedance_mode* t)

  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(530 -350 950) :rpy (list 0 0 (deg2rad -90)))) 
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(530 350 950) :rpy (list 0 0 (deg2rad 90)))) 
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (print "press key to nip box")
  (read-line)
  (unix:sleep 5)
  (send *robot* :rarm :inverse-kinematics (make-coords :pos (float-vector 530 (/ (+ box_width 50) -2.0) 950) :rpy (list 0 0 (deg2rad -90))))
  (send *robot* :larm :inverse-kinematics (make-coords :pos (float-vector 530 (/ (+ box_width 50)  2.0) 950) :rpy (list 0 0 (deg2rad 90))))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (print "press key to set-ref-forces")
  (read-line)
  (if (< box_weight 0) (setq box_weight (send (send *ri* :get-box-weight) :operation_return)))
  (print box_weight)
  (send *ri* :set-ref-forces
      (list
       (float-vector 0 0 (* (+ (send *robot* :weight) box_weight) 4.9)) 
       (float-vector 0 0 (* (+ (send *robot* :weight) box_weight) 4.9)) 
       (float-vector 0 (* -1 nip_force) (* -1.0 (* box_weight 4.9))) ;;rarm -260  mg/2
       (float-vector 0 (*  1 nip_force) (* -1.0 (* box_weight 4.9))) ;;larm -285
       )
       3000)
  (unix::sleep 4)
  (send *robot* :rarm :inverse-kinematics (make-coords :pos (float-vector 600 (/ (+ box_width 50) -2.0) 1100) :rpy (list 0 0 (deg2rad -90))))
  (send *robot* :larm :inverse-kinematics (make-coords :pos (float-vector 600 (/ (+ box_width 50)  2.0) 1100) :rpy (list 0 0 (deg2rad 90))))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )

(defun lift-box-with-handle (&optional (box_weight -1))
  (unless (string= *state* 'put)
    (print "robot carries box now")
    (return-from lift-box-with-handle nil))
  (setq *state* 'lift-box-with-handle)
  (send *ri* :stop-impedance :arms)
  (send *ri* :start-impedance :arms :K-r 200.0 :D-r 200.0 :M-r 5
                                    :K-p 100.0 :D-p 100.0 :M-p 5) ;; 10000 / 300
  (setq *impedance_mode* t)
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(470 110 950) :rpy (list 0 0 0)))
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(470 -110 950) :rpy (list 0 0 0)))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(470 110 1050) :rpy (list 0 0 0)))
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(470 -110 1050) :rpy (list 0 0 0)))
  (send *ri* :angle-vector (send *robot* :angle-vector) 4000)
  (send *ri* :wait-interpolation)
  (print "press key to set-ref-forces")
  (read-line)
  (unix:sleep 10)
  (if (< box_weight 0) (setq box_weight (send (send *ri* :get-box-weight) :operation_return)))
  (print box_weight)
  (send *ri* :set-ref-forces
      (list
       (float-vector 0 0 (* (+ (send *robot* :weight) box_weight) 4.9)) 
       (float-vector 0 0 (* (+ (send *robot* :weight) box_weight) 4.9)) 
       (float-vector 0 0 (* -1.0 (* box_weight 4.9))) ;;rarm -260  mg/2
       (float-vector 0 0 (* -1.0 (* box_weight 4.9))) ;;larm -285
       )
       3000)
  (unix::sleep 4)
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(470 110 1050) :rpy (list 0 0 0)))
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(470 -110 1050) :rpy (list 0 0 0)))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
;;  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(630 200 1050) :rpy (list 0 0 0)))
;;  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(630 -200 1050) :rpy (list 0 0 0)))
;;  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
;;  (send *ri* :wait-interpolation)
  
  )

(defun lift-box-side ()
  (unless (string= *state* 'put)
    (print "robot carries box now")
    (return-from lift-box-side nil))
  (setq *state* 'lift-box-side)
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(450 350 1050) :rpy (list 0 0 0)))
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(450 -350 1050) :rpy (list 0 0 0)))
  (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
  (send *ri* :wait-interpolation)
  (if *simulation_mode* (send *ri* :go-pos 0.1 0 0))
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(380 135 1050) :rpy (list (deg2rad -90) 0 0)))
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(380 -135 1050) :rpy (list (deg2rad 90) 0 0)))
  (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
  (send *ri* :wait-interpolation)
  (send *ri* :stop-impedance :arms)
  (send *ri* :start-impedance :arms :K-r 200.0 :D-r 200.0 :M-r 5
                                    :K-p 1000.0 :D-p 1000.0 :M-p 5) ;; 10000 / 300
  (setq *impedance_mode* t)
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(380 135 1100) :rpy (list (deg2rad -90) 0 0)))
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(380 -135 1100) :rpy (list (deg2rad 90) 0 0)))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (print "press key to set-ref-forces")
  (read-line)
  (unix:sleep 3)
  (setq box_weight (send (send *ri* :get-box-weight) :operation_return))
  (print box_weight)
  (send *ri* :set-ref-forces
      (list
       (float-vector 0 0 (* (+ (send *robot* :weight) box_weight) 4.9)) 
       (float-vector 0 0 (* (+ (send *robot* :weight) box_weight) 4.9)) 
       (float-vector 0 0 (* -1.0 (* box_weight 4.9))) ;;rarm -260  mg/2
       (float-vector 0 0 (* -1.0 (* box_weight 4.9))) ;;larm -285
       )
       3000)
  (unix::sleep 4)
  ;; (send *robot* :larm :inverse-kinematics (make-coords :pos #f(400 135 1150) :rpy (list (deg2rad -90) 0 0)))
  ;; (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(400 -135 1150) :rpy (list (deg2rad 90) 0 0)))
  ;; (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  ;; (send *ri* :wait-interpolation)
  (if *simulation_mode* (send *ri* :go-pos -1 0 0))
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(300 135 1000) :rpy (list (deg2rad -90) 0 0)))
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(300 -135 1000) :rpy (list (deg2rad 90) 0 0)))
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
  (setq *impedance_mode* t)
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

(ros::roseus "carrybox")
(setq *tfl* (instance ros::transform-listener :init))
(ros::advertise "look_box_mode" std_msgs::string 1)
(setq *current-robot* (instance jaxon_red-robot :init))
(send *current-robot* :fix-leg-to-coords (make-coords))

(defun update-tf-pose (frames)
  (send *current-robot* :angle-vector (send *ri* :state :potentio-vector))
  (send *current-robot* :fix-leg-to-coords (make-coords))
  (send *robot* :fix-leg-to-coords (make-coords))
  (dolist (frame frames) (progn
    (unless (send *tfl* :wait-for-transform "BODY" frame (ros::time 0) 3) (return nil))
    (setq body->target-coords (send *tfl* :lookup-transform "BODY" frame (ros::time 0)))
    (if (string= frame "rhand_pose") (setq *rhand_pose* (send (send *current-robot* :links :body :copy-worldcoords) :transform body->target-coords)))
    (if (string= frame "lhand_pose") (setq *lhand_pose* (send (send *current-robot* :links :body :copy-worldcoords) :transform body->target-coords)))
    (if (string= frame "goal_rhand_pose") (setq *goal_rhand_pose* (send (send *current-robot* :links :body :copy-worldcoords) :transform body->target-coords)))
    (if (string= frame "goal_lhand_pose") (setq *goal_lhand_pose* (send (send *current-robot* :links :body :copy-worldcoords) :transform body->target-coords)))
    (if (string= frame "body_pose") (setq *body_pose* (send (send *current-robot* :links :body :copy-worldcoords) :transform body->target-coords)))
    (if (string= frame "goal_body_pose") (setq *goal_body_pose* (send (send *current-robot* :links :body :copy-worldcoords) :transform body->target-coords)))
    ))
  )

(defun put-on-box (put-box-id)
  (unless (string= *state* 'lift-box)
    (print "robot not carries box now")
    (return-from put-on-box nil))
  (ros::set-param "/boxpose_pub/put_box_id" put-box-id)
  (setq msg (instance std_msgs::string :init))
  (send msg :data (format nil "put-box"))
  (ros::publish "look_box_mode" msg)
  (print "change look_box_mode: put-box")
  (unix:sleep 1)

  ;;箱のtf取得
  (update-tf-pose (list "rhand_pose" "lhand_pose" "goal_rhand_pose" "goal_lhand_pose"))
  (setq rdiff (v- (send *goal_rhand_pose* :worldpos) (send *rhand_pose* :worldpos)))
  (setq ldiff (v- (send *goal_lhand_pose* :worldpos) (send *lhand_pose* :worldpos)))
  (setq hand_ave_diff (* 0.5 (+ (norm rdiff) (norm ldiff))))
  (if (> hand_ave_diff 200) (progn
    ;;高さ変更
    (print "高さ変更")
    (setq move-z (+ (elt (scale 0.5 (v+ rdiff ldiff)) 2) 100))
    (while (> move-z 100) (progn
      (send *robot* :arms :move-end-pos (float-vector 0 0 100) :world)
      (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      (setq move-z (- move-z 100))))
    (send *robot* :arms :move-end-pos (float-vector 0 0 move-z) :world)
    (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    
    ;;置く位置へ移動
    (print "置く位置へ移動")
    (update-tf-pose (list "goal_body_pose"))
    (setq body-x (transform (send *goal_body_pose* :worldrot) #f(1 0 0)))
    (setq theta (* 57.2957795131 (atan2 (elt body-x 1) (elt body-x 0)))) ;;360/(2*pi)　書くのめんどかった
    (send *ri* :go-pos (* 0.001 (elt (send *goal_body_pose* :worldpos) 0)) (* 0.001 (elt (send *goal_body_pose* :worldpos) 1)) theta)
    (unix:sleep 1)

    ;;箱の状態取得
    (update-tf-pose (list "rhand_pose" "lhand_pose" "goal_rhand_pose" "goal_lhand_pose"))
    (setq rdiff (v- (send *goal_rhand_pose* :worldpos) (send *rhand_pose* :worldpos)))
    (setq ldiff (v- (send *goal_lhand_pose* :worldpos) (send *lhand_pose* :worldpos)))
    ))
  ;;置く
  ;; TODO : 回転で正しくない
  (print "置く位置決め")
  (setq hand_ave_diff (* 0.5 (+ (norm (float-vector (elt rdiff 0) (elt rdiff 1))) (norm (float-vector (elt ldiff 0) (elt ldiff 1))))))
  (while (> hand_ave_diff 10) (progn
    (print hand_ave_diff)
    (send *robot* :rarm :move-end-pos rdiff :world)
    (send *robot* :larm :move-end-pos ldiff :world)
    (send *robot* :arms :move-end-pos #f(0 0 15) :world)
    (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    ;;箱の状態取得
    (update-tf-pose (list "rhand_pose" "lhand_pose" "goal_rhand_pose" "goal_lhand_pose"))
    (setq rdiff (v- (send *goal_rhand_pose* :worldpos) (send *rhand_pose* :worldpos)))
    (setq ldiff (v- (send *goal_lhand_pose* :worldpos) (send *lhand_pose* :worldpos)))
    (setq hand_ave_diff (* 0.5 (+ (norm (float-vector (elt rdiff 0) (elt rdiff 1))) (norm (float-vector (elt ldiff 0) (elt ldiff 1))))))
  ))
  (print hand_ave_diff)

  (print "力を弱める")
  (send *ri* :set-ref-forces
        (list
         (float-vector 0 0 (* (send *robot* :weight) 4.9))
         (float-vector 0 0 (* (send *robot* :weight) 4.9))
         (float-vector 0 0 0) ;;rarm -260  mg/2
         (float-vector 0 0 0) ;;larm -285
         )
         2000)
  (unix:sleep 2)
  (print "置く")
  (send *robot* :rarm :inverse-kinematics (send (send *goal_rhand_pose* :copy-worldcoords) :transform (make-coords :pos #f(0 0 -150))))
  (send *robot* :larm :inverse-kinematics (send (send *goal_lhand_pose* :copy-worldcoords) :transform (make-coords :pos #f(0 0 -150))))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (setq *state* 'put)
  )


(defun lift-box-on-table (target-list box_weight &optional (nip_force 50))
  (unless (string= *state* 'put)
    (print "robot carries box now")
    (return-from lift-box-on-table nil))

  ;;target設定
  (ros::set-param "/boxpose_pub/hold_box_id" (elt target-list 0))

  ;; 見る場所の変更
  (setq msg (instance std_msgs::string :init))
  (send msg :data (format nil "lift-box"))
  (ros::publish "look_box_mode" msg)
  (print "change look_box_mode: lift-box")
  (unix:sleep 3)

  ;;把持位置へ移動
  (update-tf-pose (list "body_pose"))
  (setq body-x (transform (send *body_pose* :worldrot) #f(1 0 0)))
  (setq theta (* 57.2957795131 (atan2 (elt body-x 1) (elt body-x 0)))) ;;360/(2*pi)　書くのめんどかった
  (if (or (> (norm (float-vector (elt (send *body_pose* :worldpos) 0) (elt (send *body_pose* :worldpos) 1) 0)) 150) (> theta 20)) (progn
    (go-lift-pose)
    ;;一度20cm手前に移動
    (print "20cm手前へ移動")
    (setq *body_pose* (send *body_pose* :transform (make-coords :pos #f(-200 0 0))))
    (setq body-x (transform (send *body_pose* :rot) #f(1 0 0)))
    (setq theta (* 57.2957795131 (atan2 (elt body-x 1) (elt body-x 0)))) ;;360/(2*pi)　書くのめんどかった
    (send *ri* :go-pos (* 0.001 (elt (send *body_pose* :worldpos) 0)) (* 0.001 (elt (send *body_pose* :worldpos) 1)) theta)
    (unix:sleep 1)

    ;;再度*body_pose*を設定し、移動
    (print "把持位置へ移動")
    (update-tf-pose (list "body_pose"))
    (setq body-x (transform (send *body_pose* :rot) #f(1 0 0)))
    (setq theta (* 57.2957795131 (atan2 (elt body-x 1) (elt body-x 0)))) ;;360/(2*pi)　書くのめんどかった
    (send *ri* :go-pos (* 0.001 (elt (send *body_pose* :worldpos) 0)) (* 0.001 (elt (send *body_pose* :worldpos) 1)) theta)
    (unix:sleep 1)
  ))

  ;;把持高さへ移動
  (update-tf-pose (list "rhand_pose" "lhand_pose"))
  (setq hand_ave (scale 0.5 (v+ (send *rhand_pose* :worldpos) (send *lhand_pose* :worldpos))))
  (setq z_move 0)
  (if (< (elt hand_ave 2) 1000) (progn
    (print "かがむ")
    (setq z_move (- 1000 (elt hand_ave 2)))
    (send *robot* :legs :move-end-pos (float-vector 0 0 z_move))
    (send *ri* :angle-vector (send *robot* :angle-vector) 6000)
    (send *ri* :wait-interpolation)
    (unix:sleep 1)
    (update-tf-pose (list "rhand_pose" "lhand_pose"))
    ))
  
  ;;見る場所を戻す
  (setq msg (instance std_msgs::string :init))
  (send msg :data (format nil "box-balancer"))
  (ros::publish "look_box_mode" msg)
  (print "change look_box_mode: box-balancer")

  ;;挟む
  (print "挟む")
  (send *robot* :rarm :inverse-kinematics (send (send *rhand_pose* :copy-worldcoords) :transform (make-coords :pos #f(-150 0 -100))))
  (send *robot* :larm :inverse-kinematics (send (send *lhand_pose* :copy-worldcoords) :transform (make-coords :pos #f(-150 0 -100))))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *robot* :rarm :inverse-kinematics (send (send *rhand_pose* :copy-worldcoords) :transform (make-coords :pos #f(0 0 -100))))
  (send *robot* :larm :inverse-kinematics (send (send *lhand_pose* :copy-worldcoords) :transform (make-coords :pos #f(0 0 -100))))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (unless *impedance_mode* (progn
    (send *ri* :stop-impedance :arms)
    (send *ri* :start-impedance :arms :K-r 100.0 :D-r 100.0 :M-r 1
                                      :K-p 300.0 :D-p 300.0 :M-p 5)
    (setq *impedance_mode* t)))
  (send *robot* :rarm :inverse-kinematics (send *rhand_pose* :copy-worldcoords))
  (send *robot* :larm :inverse-kinematics (send *lhand_pose* :copy-worldcoords))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  ;;力を入れる
  (print "力を入れる")
  (if (< box_weight 0) (setq box_weight (send (send *ri* :get-box-weight) :operation_return)))
  (print box_weight)
  (send *ri* :set-ref-forces
      (list
       (float-vector 0 0 (* (+ (send *robot* :weight) box_weight) 4.9)) 
       (float-vector 0 0 (* (+ (send *robot* :weight) box_weight) 4.9)) 
       (float-vector 0 (* -1 nip_force) (* -1.0 (* box_weight 4.9))) ;;rarm -260  mg/2
       (float-vector 0 (*  1 nip_force) (* -1.0 (* box_weight 4.9))) ;;larm -285
       )
       3000)
  (unix::sleep 3)

  ;; ;;持ち上げる
  ;; (send *robot* :arms :move-end-pos #f(0 0 50) :world)
  ;; (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  ;; (send *ri* :wait-interpolation)

  ;;高さ戻す
  (print "かがんだ分高さ戻す")
  (send *robot* :legs :move-end-pos (float-vector 0 0 (* -1.0 z_move)) :world)
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (setq *state* 'lift-box)
  )

(defun align-boxes (target-list)
  (print target-list)
  (setq put-target (car target-list))
  (setq lift-targets (cdr target-list))
  (if lift-targets (progn
    (ros::set-param "/boxpose_pub/put_box_id" put-target)
    (ros::set-param "/boxpose_pub/hold_box_id" (elt lift-targets 0))
    (setq msg (instance std_msgs::string :init))
    (send msg :data (format nil "lift-box"))
    (ros::publish "look_box_mode" msg)
    (print "change look_box_mode: lift-box")
    (unix:sleep 2)
    (unless (send *tfl* :wait-for-transform "goal_rhand_pose" "rhand_pose" (ros::time 0) 3) (return nil))
    (unless (send *tfl* :wait-for-transform "goal_lhand_pose" "lhand_pose" (ros::time 0) 3) (return nil))
    (setq rnow->rgoal (send *tfl* :lookup-transform "rhand_pose" "goal_rhand_pose" (ros::time 0)))
    (setq lnow->lgoal (send *tfl* :lookup-transform "lhand_pose" "goal_lhand_pose" (ros::time 0)))
    (setq hand_ave_diff (* 0.5 (+ (norm (send rnow->rgoal :pos)) (norm (send lnow->lgoal :pos)))))
    (if (> hand_ave_diff 200) (progn (print "箱を持つ前に下がっておく") (go-lift-pose) (send *ri* :go-pos -0.2 0 0)))
    (if (> hand_ave_diff 40) (progn
      (print "lift-box-on-table")
      (lift-box-on-table lift-targets (* 1.5 (length lift-targets)) 100)
      (print "put-on-box")
      (put-on-box put-target)))
    (align-boxes lift-targets))))

(defun put-box (&optional (wait-time 10))
  (unix:sleep wait-time)
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
    (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(480 -350 1050) :rpy (list 0 0 (deg2rad -90))))
    (send *robot* :larm :inverse-kinematics (make-coords :pos #f(480 350 1050) :rpy (list 0 0 (deg2rad 90))))
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
    (send *robot* :larm :inverse-kinematics (make-coords :pos #f(470 150 1000) :rpy (list 0 0 0)))
    (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(470 -150 1000) :rpy (list 0 0 0)))
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
    (send *robot* :larm :inverse-kinematics (make-coords :pos #f(470 200 1000) :rpy (list 0 0 0)))
    (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(470 -200 1000) :rpy (list 0 0 0)))
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

