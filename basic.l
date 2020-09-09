#!/usr/bin/env roseus

(load "jaxonred.l")
(load "utils.l")

(defun start (&optional (simulation nil))
  (setq *robot* *jaxon_red*)
  ;; (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  ;; (send *ri* :angle-vector (send *ri* :state :potentio-vector))
  (send *ri* :stop-st)
  (send *ri* :stop-auto-balancer)
  (if simulation (start-footguided-modification) (start-footguided-modification-zikki))
  ;; (send *ri* :set-auto-balancer-param
  ;;           :is-hand-fix-mode nil
  ;;           :ik-mode 1)
  (if (not simulation) (progn
    (print "press key to start-auto-balancer")
    (read-line)
    ))
  (send *ri* :start-auto-balancer :limbs '(:rleg :lleg))
  (if (not simulation) (progn
    (print "press key to start-st")
    (read-line)
    ))
  ;; (send *ri* :set-st-param :transition-time 10)
  (send *ri* :start-st)
  )
