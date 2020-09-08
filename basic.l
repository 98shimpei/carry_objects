#!/usr/bin/env roseus

(load "jaxonred.l")
(load "utils.l")

(defun start (&optional (simulation nil))
  (setq *robot* *jaxon_red*)
  ;; (print (send *robot* :angle-vector (send *ri* :state :potentio-vector)))
  ;; (send *ri* :angle-vector (send *robot* :angle-vector))
  (if simulation (start-footguided-modification) (start-footguided-modification-zikki))
  ;; (send *ri* :set-auto-balancer-param
  ;;           :is-hand-fix-mode nil
  ;;           :ik-mode 1)
  (print "press key to start-auto-balancer")
  (read-line)
  (send *ri* :start-auto-balancer :limbs '(:rleg :lleg))
  (print "press key to start-st")
  (read-line)
  ;; (send *ri* :set-st-param :transition-time 10)
  (send *ri* :start-st)
  )
