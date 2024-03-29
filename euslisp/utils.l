(defun savelog (&optional (fname "carryobject"))
  (progn
    (send *ri* :set-log-maxlength 2000000)
    (send *ri* :start-log)
    (read-line) 
    (send *ri* :save-log (format nil "~A~A" "/home/shimpei/log/" fname))
    )
  )

(defun mv ()
  (objects (list *robot*))
  t
  )

(defun logstart (&optional (fname "carryobject"))
  (progn
    (send *ri* :set-log-maxlength 2000000)
    (send *ri* :start-log)))

(defun logstop (&optional (fname "carryobject"))
    (send *ri* :save-log (format nil "~A~A" "/home/shimpei/log/" fname))
    )

(defun go-coop-pose ()
  ;; (send *robot* :larm :inverse-kinematics (make-coords :pos #f(680 230 950) :rpy (list 0 0 0))) ;; 580 230 1000
  ;; (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(680 -230 950) :rpy (list 0 0 0)))
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(580 230 947) :rpy (list 0 -0.002 0))) ;; 580 230 1000
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(580 -230 950) :rpy (list 0 -0.002 0)))
  (send *robot* :head :angle-vector #f(0 10))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )

(defun go-lift-pose ()
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(250 470 1050) :rpy (list (deg2rad 0) 0 0))) ;; 400->370
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(250 -470 1050) :rpy (list (deg2rad 0) 0 0)))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )

(defun go-reset-pose ()
  (send *ri* :angle-vector (send *robot* :reset-pose))
  (send *ri* :wait-interpolation)
  )

(defun go-step-pose ()
  (send *robot* :head :angle-vector #f(0 10))
  (send *robot* :larm :move-end-pos #f(-50 150 50))
  (send *robot* :rarm :move-end-pos #f(-50 -150 50))
  (send *ri* :angle-vector (send *robot* :angle-vector))
  (send *ri* :wait-interpolation)
  )

(defun go-case-pose ()
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(270 -240 900) :rpy (list (deg2rad 10) (deg2rad 80) (deg2rad 10))))
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(270 240 900) :rpy (list (deg2rad -15) (deg2rad 80) (deg2rad -15))))
  (send *robot* :legs :crotch-p :joint-angle -10)
  (send *ri* :angle-vector (send *robot* :angle-vector))
  (send *ri* :wait-interpolation)
  )

(defun go-sit-pose ()
  (send *robot* :torso-waist-p :joint-angle 30)
  (send *robot* :legs :crotch-p :joint-angle -30)
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :legs :move-end-pos #f(150 0 400))
  (send *robot* :fix-leg-to-coords (make-coords))
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(500 400 250) :rpy (list 0 (deg2rad 40) (deg2rad 90))))
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(500 -400 250) :rpy (list 0 (deg2rad 40) (deg2rad -90))))
  (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
  (send *ri* :wait-interpolation)
  )

(defun go-new-pose ()
  (send *robot* :reset-pose)
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(480 180 1200) :rpy (list (deg2rad -20) (deg2rad -30) (deg2rad 80))))
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(480 -180 1200) :rpy (list (deg2rad 20) (deg2rad -30) (deg2rad -80))))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :stop-impedance :arms)
  (send *ri* :start-impedance :arms :K-r 100.0 :D-r 100.0 :M-r 1
                                    :K-p 700.0 :D-p 700.0 :M-p 5)
  )

(defun go-cart-pose ()
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(550 270 1100) :rpy (list (deg2rad 0) 0 (deg2rad 90))))
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(550 -270 1100) :rpy (list (deg2rad 0) 0 (deg2rad -90))))
  (send *ri* :angle-vector (send *robot* :angle-vector) 7000)
  (send *ri* :wait-interpolation)
  (print "set cart")
  (read-line)
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(550 170 1100) :rpy (list (deg2rad 0) 0 (deg2rad 90))))
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(550 -170 1100) :rpy (list (deg2rad 0) 0 (deg2rad -90))))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(550 170 900) :rpy (list (deg2rad 0) 0 (deg2rad 90))))
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(550 -170 900) :rpy (list (deg2rad 0) 0 (deg2rad -90))))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  )
  
(defun go-wheelbarrow-pose ()
  (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(450 -295 800) :rpy (list (deg2rad 0) (deg2rad 90) (deg2rad 0))))
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(450 295 800) :rpy (list (deg2rad 0) (deg2rad 90) (deg2rad 0))))
  (send *ri* :angle-vector (send *robot* :angle-vector) 7000)
  (send *ri* :wait-interpolation)
  )

(defun govel (&optional (wait-time 5))
  (send *ri* :go-velocity 0 0 0)
  (unix:sleep wait-time)
  (send *ri* :go-stop)
  )

(defun capture-pose ()
  (print "3")
  (unix:sleep 1)
  (print "2")
  (unix:sleep 1)
  (print "1")
  (unix:sleep 1)
  (print "0")
  (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  (send *ri* :angle-vector (send *robot* :angle-vector)))

(defun send-pose ()
  (send *robot* :angle-vector (send *ri* :reference-vector))
  (send *ri* :angle-vector (send *robot* :angle-vector))
  )

(defun start-toe-mode
    ()
  (send *ri* :set-gait-generator-param
	:toe-angle 20 :heel-angle 0
	:toe-heel-phase-ratio #f(0.05 0.3 0.4 0.0 0.1 0.1 0.05)
	:use-toe-heel-transition t
	:use-toe-heel-auto-set t)
  )
