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
  (send *robot* :larm :inverse-kinematics (make-coords :pos #f(470 200 1050) :rpy (list 0 0 0)))
    (send *robot* :rarm :inverse-kinematics (make-coords :pos #f(470 -200 1050) :rpy (list 0 0 0)))
    (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
  )

(defun go-reset-pose ()
  (send *ri* :angle-vector (send *robot* :reset-pose))
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

(defun govel (&optional (wait-time 5))
  (send *ri* :go-velocity 0 0 0)
  (unix:sleep wait-time)
  (send *ri* :go-stop)
  )
