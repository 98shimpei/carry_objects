#!/usr/bin/env roseus

;; (load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
;; (jaxon_red-init)
(load "jaxonred.l")
(setq *robot* *jaxon_red*)
(push-init t)

(defun mv ()
  (objects (list *robot*))
  t
  )

