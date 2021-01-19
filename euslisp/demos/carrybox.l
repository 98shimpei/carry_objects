#!/usr/bin/env roseus

(load "package://carry_objects/euslisp/jaxonred.l")
(load "package://carry_objects/euslisp/utils.l")
(ros::load-ros-manifest "roseus")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-srvs "carry_objects")
(ros::roseus-add-msgs "carry_objects")

(setq *impedance_mode* nil)
(setq *dummy-spin* t)
(setq *return* nil)
(setq *lift-targets* nil)

(ros::roseus "carrybox_eus")
(setq *tfl* (instance ros::transform-listener :init))
(ros::advertise "update_box_id" std_msgs::Bool 1)
(ros::advertise "look_box_mode" std_msgs::string 1)
(setq *current-robot* (instance jaxon_red-robot :init))
(send *current-robot* :fix-leg-to-coords (make-coords))

(defun start (&optional (simulation nil))
  (if simulation (setq *simulation_mode* t) (setq *simulation_mode* nil))
  (setq *robot* *jaxon_red*)
  ;; (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  ;; (send *ri* :angle-vector (send *robot* :reset-pose))
  (send *ri* :stop-st)
  (send *ri* :stop-auto-balancer)
  ;; (send *ri* :set-auto-balancer-param
  ;;     :use-force-mode :MODE-REF-FORCE-WITH-FOOT
  ;;     :additional-force-applied-link-name "RARM_JOINT7"
  ;;     :additional-force-applied-point-offset #F(0.1 0.3 0)
  ;;     )
  (send *ri* :set-ref-forces
      (list
       (float-vector 0 0 (* (send *robot* :weight) 0.0049)) 
       (float-vector 0 0 (* (send *robot* :weight) 0.0049)) 
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
  (send *ri* :set-auto-balancer-param
             :is-hand-fix-mode nil
             :hand-fix-fall-gain 0.995 ;;995
             :hand-fix-slip-gain 0.995 ;;995
             :hand-fix-slip-limit 0.5 ;;5
             :hand-fix-body-gain (list 0.001 0.001 0.0003) ;;(list 0.001 0.001 0.0003)
             :ik-mode 1)
  (if (not simulation) (let ()
    (print "press key to start-st")
    (read-line)
    ))
  (send *ri* :start-st)
  (send *ri* :remove-force-sensor-offset-rmfo-arms)
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *state* 'put)
  (print "close hands")
  (send *ri* :move-gripper :arms 0.2 :wait t)
  )

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
    (if (string= frame "rarm_end_coords") (setq *rarm_end_coords* (send (send *current-robot* :links :body :copy-worldcoords) :transform body->target-coords)))
    (if (string= frame "larm_end_coords") (setq *larm_end_coords* (send (send *current-robot* :links :body :copy-worldcoords) :transform body->target-coords)))
    ))
  )

(defun put-on-box (put-box-id &optional (active-modification nil))
  (if active-modification (print "active-modification mode"))
  (unless (string= *state* 'lift-box)
    (print "robot not carries box now")
    (return-from put-on-box nil))
  (ros::set-param "/boxpose_pub/put_box_id" put-box-id)
  (setq msg (instance std_msgs::Bool :init))
  (send msg :data t)
  (ros::publish "update_box_id" msg)
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
    (if (> move-z 0) (progn
      (while (> move-z 100) (progn
        (send *robot* :arms :move-end-pos (float-vector 0 0 100) :world)
        (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
        (send *ri* :wait-interpolation)
        (setq move-z (- move-z 100))))
      (send *robot* :arms :move-end-pos (float-vector 0 0 move-z) :world)
      (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      )
    )
    
    ;;置く位置へ移動
    (print "置く位置へ移動")
    (update-tf-pose (list "goal_body_pose"))
    (setq body-x (transform (send *goal_body_pose* :worldrot) #f(1 0 0)))
    (setq theta (* 57.2957795131 (atan2 (elt body-x 1) (elt body-x 0)))) ;;360/(2*pi)　書くのめんどかった
    (setq *dummy-spin* t)
    (ros::spin-once)
    (while (or (> (norm (float-vector (elt (send *goal_body_pose* :worldpos) 0) (elt (send *goal_body_pose* :worldpos) 1))) 50) (> theta 10)) (progn
      (setq *dummy-spin* nil)
      (setq vel (float-vector (elt (send *goal_body_pose* :worldpos) 0) (elt (send *goal_body_pose* :worldpos) 1)))
      (setq vel (scale (/ 100.0 (norm vel)) vel))
      (setq omega_abs (min 4.0 (abs (* 0.8 theta)))) ;;360/(2*pi)　書くのめんどかった
      (setq omega (if (> theta 0) omega_abs (- omega_abs)))
      (print vel)
      (print omega)
      (send *ri* :go-velocity (* 0.001 (elt vel 0)) (* 0.001 (elt vel 1)) omega)
      (if active-modification (progn (print "check misalignment") (ros::spin-once)))
      (if *return* (progn (setq *return* nil) (return-from put-on-box nil)))
      (unix:sleep 1)
      (update-tf-pose (list "goal_body_pose"))
      (setq body-x (transform (send *goal_body_pose* :rot) #f(1 0 0)))
      (setq theta (* 57.2957795131 (atan2 (elt body-x 1) (elt body-x 0)))) ;;360/(2*pi)　書くのめんどかった
      ))
    (setq *dummy-spin* t)
    (send *ri* :go-stop)
    (setq msg (instance std_msgs::string :init))
    (send msg :data (format nil "put-box"))
    (ros::publish "look_box_mode" msg)
    (print "change look_box_mode: put-box")
    (send *ri* :stop-box-balancer 1.0)
    (unix:sleep 1)
    (update-tf-pose (list "goal_body_pose"))
    (setq body-x (transform (send *goal_body_pose* :worldrot) #f(1 0 0)))
    (setq theta (* 57.2957795131 (atan2 (elt body-x 1) (elt body-x 0)))) ;;360/(2*pi)　書くのめんどかった
    (send *ri* :go-pos (* 0.001 (elt (send *goal_body_pose* :worldpos) 0)) (* 0.001 (elt (send *goal_body_pose* :worldpos) 1)) theta)
    (unix:sleep 1)

    ;;把持高さへ移動
    (update-tf-pose (list "rhand_pose" "lhand_pose" "goal_rhand_pose" "goal_lhand_pose"))
    (setq rdiff (v- (send *goal_rhand_pose* :worldpos) (send *rhand_pose* :worldpos)))
    (setq ldiff (v- (send *goal_lhand_pose* :worldpos) (send *lhand_pose* :worldpos)))
    (setq hand_ave (scale 0.5 (v+ (send *goal_rhand_pose* :worldpos) (send *goal_lhand_pose* :worldpos))))
    (setq z_move 0)
    (if (< (elt hand_ave 2) 1000) (progn
      (print "かがむ")
      (setq z_move (- 1000 (elt hand_ave 2)))
      (send *robot* :legs :move-end-pos (float-vector 0 0 z_move))
      (send *ri* :angle-vector (send *robot* :angle-vector) 9000)
      (send *ri* :wait-interpolation)
      (unix:sleep 1)
      ))
    ))
  (setq msg (instance std_msgs::string :init))
  (send msg :data (format nil "put-box"))
  (ros::publish "look_box_mode" msg)
  (print "change look_box_mode: put-box")
  (unix:sleep 1)
  ;;箱の状態取得
  (update-tf-pose (list "rhand_pose" "lhand_pose" "goal_rhand_pose" "goal_lhand_pose"))
  (setq rdiff (v- (send *goal_rhand_pose* :worldpos) (send *rhand_pose* :worldpos)))
  (setq ldiff (v- (send *goal_lhand_pose* :worldpos) (send *lhand_pose* :worldpos)))
  ;;置く
  ;; TODO : 回転で正しくない
  (print "置く位置決め")
  (setq hand_ave_diff (* 0.5 (+ (norm (float-vector (elt rdiff 0) (elt rdiff 1))) (norm (float-vector (elt ldiff 0) (elt ldiff 1))))))
  (print hand_ave_diff)
  (send *robot* :rarm :move-end-pos rdiff :world)
  (send *robot* :larm :move-end-pos ldiff :world)
  (send *robot* :arms :move-end-pos #f(0 0 15) :world)
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (unix:sleep 3)
  ;;箱の状態取得
  (update-tf-pose (list "rhand_pose" "lhand_pose" "goal_rhand_pose" "goal_lhand_pose"))
  (setq rdiff (v- (send *goal_rhand_pose* :worldpos) (send *rhand_pose* :worldpos)))
  (setq ldiff (v- (send *goal_lhand_pose* :worldpos) (send *lhand_pose* :worldpos)))
  (setq hand_ave_diff (* 0.5 (+ (norm (float-vector (elt rdiff 0) (elt rdiff 1))) (norm (float-vector (elt ldiff 0) (elt ldiff 1))))))
  (while (> hand_ave_diff 10) (progn
    (print hand_ave_diff)
    (send *robot* :rarm :move-end-pos rdiff :world)
    (send *robot* :larm :move-end-pos ldiff :world)
    (send *robot* :arms :move-end-pos #f(0 0 15) :world)
    (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    (unix:sleep 1)
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
         (float-vector 0 0 (* (send *robot* :weight) 0.0049))
         (float-vector 0 0 (* (send *robot* :weight) 0.0049))
         (float-vector 0 0 0) ;;rarm -260  mg/2
         (float-vector 0 0 0) ;;larm -285
         )
         2000)
  (unix:sleep 2)
  (setq req (instance carry_objects::LiftBoxRequest :init))
  (send req :boxes (list))
  (setq box_weight (send (ros::service-call "lift_box_id" req) :weight))
  (print "置く")
  (send *robot* :rarm :inverse-kinematics (send (send *goal_rhand_pose* :copy-worldcoords) :transform (make-coords :pos #f(0 0 -150))))
  (send *robot* :larm :inverse-kinematics (send (send *goal_lhand_pose* :copy-worldcoords) :transform (make-coords :pos #f(0 0 -150))))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (setq *state* 'put)
  (if (> z_move 0) (print "かがんだ分高さ戻す"))
  (send *robot* :legs :move-end-pos (float-vector 0 0 (* -1.0 z_move)) :world)
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :stop-box-balancer 0.998)
  (print "stop-box-balancer 0.998")
  )


(defun lift-box-simple (target-list &optional (nip_force 50))
  (unless (string= *state* 'put)
    (print "robot carries box now")
    (return-from lift-box-simple nil))

  ;;target設定
  (ros::set-param "/boxpose_pub/hold_box_id" (elt target-list 0))
  (ros::set-param "/boxpose_pub/top_box_id" (elt target-list (- (length target-list) 1)))
  (if (= (length target-list) 1) (ros::set-param "/boxpose_pub/base_box_id" (elt target-list 0)) (ros::set-param "/boxpose_pub/base_box_id" (elt target-list (- (length target-list) 2))))
  (setq msg (instance std_msgs::Bool :init))
  (send msg :data t)
  (ros::publish "update_box_id" msg)

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
    (setq *pre_body_pose* (send *body_pose* :transform (make-coords :pos #f(-200 0 0))))
    (setq body-x (transform (send *pre_body_pose* :rot) #f(1 0 0)))
    (setq theta (* 57.2957795131 (atan2 (elt body-x 1) (elt body-x 0)))) ;;360/(2*pi)　書くのめんどかった
    (print "20cm手前へ移動")
  ;;一度20cm手前に移動
  (while (or (> (norm (float-vector (elt (send *pre_body_pose* :worldpos) 0) (elt (send *pre_body_pose* :worldpos) 1))) 100) (> theta 15)) (progn
    (setq vel (float-vector (elt (send *pre_body_pose* :worldpos) 0) (elt (send *pre_body_pose* :worldpos) 1)))
    (setq vel (scale (/ 100.0 (norm vel)) vel))
    (setq omega_abs (min 4.0 (abs (* 0.8 theta)))) ;;360/(2*pi)　書くのめんどかった
    (setq omega (if (> theta 0) omega_abs (- omega_abs)))
    (print vel)
    (print omega)
    (send *ri* :go-velocity (* 0.001 (elt vel 0)) (* 0.001 (elt vel 1)) omega)
    (unix:sleep 1)
    (update-tf-pose (list "body_pose"))
    (setq *pre_body_pose* (send *body_pose* :transform (make-coords :pos #f(-200 0 0))))
    (setq body-x (transform (send *body_pose* :rot) #f(1 0 0)))
    (setq theta (* 57.2957795131 (atan2 (elt body-x 1) (elt body-x 0)))) ;;360/(2*pi)　書くのめんどかった
    ))
    (send *ri* :go-stop)
    (unix:sleep 1)

    ;;再度*body_pose*を設定し、移動
    (print "把持位置へ移動")
    (update-tf-pose (list "body_pose"))
    (setq body-x (transform (send *body_pose* :rot) #f(1 0 0)))
    (setq theta (* 57.2957795131 (atan2 (elt body-x 1) (elt body-x 0)))) ;;360/(2*pi)　書くのめんどかった
    (send *ri* :go-pos (* 0.001 (elt (send *body_pose* :worldpos) 0)) (* 0.001 (elt (send *body_pose* :worldpos) 1)) theta)
    (unix:sleep 2)
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
  
  ;;挟む
  (print "挟む")
  (send *robot* :rarm :inverse-kinematics (send (send *rhand_pose* :copy-worldcoords) :transform (make-coords :pos #f(-100 0 -100))))
  (send *robot* :larm :inverse-kinematics (send (send *lhand_pose* :copy-worldcoords) :transform (make-coords :pos #f(-100 0 -100))))
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
  (update-tf-pose (list "rhand_pose" "lhand_pose" "rarm_end_coords" "larm_end_coords"))
  (setq rdiff (v- (send *rhand_pose* :worldpos) (send *rarm_end_coords* :worldpos)))
  (setq ldiff (v- (send *lhand_pose* :worldpos) (send *larm_end_coords* :worldpos)))
  (send *robot* :rarm :move-end-pos rdiff :world)
  (send *robot* :larm :move-end-pos ldiff :world)
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  ;;力を入れる
  (print "力を入れる")
  (setq req (instance carry_objects::LiftBoxRequest :init))
  (send req :boxes target-list)
  (setq box_weight (send (ros::service-call "lift_box_id" req) :weight))
  (if (< box_weight 0) (setq box_weight (send (send *ri* :get-box-weight) :operation_return)))
  (print box_weight)
  (send *ri* :set-ref-forces
      (list
       (float-vector 0 0 (* (+ (* (send *robot* :weight) 0.001) box_weight) 4.9)) 
       (float-vector 0 0 (* (+ (* (send *robot* :weight) 0.001) box_weight) 4.9)) 
       (float-vector 0 (* -1 nip_force) (* -1.0 (* box_weight 4.9))) ;;rarm -260  mg/2
       (float-vector 0 (*  1 nip_force) (* -1.0 (* box_weight 4.9))) ;;larm -285
       )
       3000)
  (unix::sleep 3)

  ;; ;;持ち上げる
  ;; (send *robot* :arms :move-end-pos #f(0 0 50) :world)
  ;; (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  ;; (send *ri* :wait-interpolation)

  ;;見る場所を戻す
  (setq msg (instance std_msgs::string :init))
  (send msg :data (format nil "box-balancer"))
  (ros::publish "look_box_mode" msg)
  (print "change look_box_mode: box-balancer")

  ;;高さ戻す
  (if (> z_move 0) (print "かがんだ分高さ戻す"))
  (send *robot* :legs :move-end-pos (float-vector 0 0 (* -1.0 z_move)) :world)
  (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  (setq *lift-targets* target-list)
  (setq *state* 'lift-box)
  )

(defun lift-box-on-table (target-list)
  (while (align-boxes target-list) (print "align-boxes"))
  (print "lift")
  (ros::set-param "/boxpose_pub/top_box_id" (elt target-list (- (length target-list) 1)))
  (if (= (length target-list) 1) (ros::set-param "/boxpose_pub/base_box_id" (elt target-list 0)) (ros::set-param "/boxpose_pub/base_box_id" (elt target-list (- (length target-list) 2))))
  (unix:sleep 1)
  (send *ri* :set-box-balancer (ros::get-param "/boxpose_pub/top_box_id") (ros::get-param"/boxpose_pub/base_box_id"))
  (lift-box-simple target-list 130)
  (send *ri* :start-box-balancer nil nil 7 8 2.5 0.0045 0.001 0.4)
  (send *robot* :arms :move-end-pos #f(0 0 100) :world)
  (send *ri* :angle-vector (send *robot* :angle-vector))
  (send *ri* :wait-interpolation)
  )

(defun align-boxes (target-list)
  (setq align-flag nil)
  (print target-list)
  (setq put-target (car target-list))
  (setq lift-targets (cdr target-list))
  (if lift-targets (progn
    (ros::set-param "/boxpose_pub/put_box_id" put-target)
    (ros::set-param "/boxpose_pub/hold_box_id" (elt lift-targets 0))
    (setq msg (instance std_msgs::Bool :init))
    (send msg :data t)
    (ros::publish "update_box_id" msg)
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
      (print "lift-box-simple")
      (lift-box-simple lift-targets 130)
      (print "put-on-box")
      (put-on-box put-target)
      (setq align-flag t)))
    (or (align-boxes lift-targets) align-flag)
    )
    nil
    )
  )

(defun put-and-lift (put-target lift-targets)
  (put-on-box put-target nil)
  (lift-box-on-table lift-targets)
  )

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

(setq *okinaoshi-phase* 0)

(defun emergency_command_listener (msg)
  (print "emergency_command subscribed")
  (unless *dummy-spin* (progn
    (print "do active modification")
    (if (= (send msg :mode) 0)
      (progn
        (print "shake boxes")
        (format t "shake param: ~A ~A ~A ~A ~A ~A ~%" (send msg :period) (send msg :amp) (send msg :ampr) (send msg :x) (send msg :y) (send msg :z))
        (send *ri* :jamp-box (send msg :period) (send msg :amp) (send msg :ampr) (send msg :x) (send msg :y) (send msg :z))
      )
    )

    (if (and (= (send msg :mode) 1) (= *okinaoshi-phase* 0))
      (progn
        (print "机に置こうかな")
        (setq old-put-target (ros::get-param "/boxpose_pub/put_box_id"))
        (send *ri* :go-stop)
        (unix:sleep 3)
        (setq *okinaoshi-phase* 1)
        (ros::spin-once)
        (setq *okinaoshi-phase* 0)
        (put-on-box old-put-target t)
        (setq *return* t)
      )
    )

    (if (and (= (send msg :mode) 1) (= *okinaoshi-phase* 1))
      (progn
        (print "机に置くよ")
        (put-and-lift (send msg :put_id) *lift-targets*)
      )
    )
  ))
)

(ros::subscribe "emergency_command" carry_objects::EmergencyCommand #'emergency_command_listener)
