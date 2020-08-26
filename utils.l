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
