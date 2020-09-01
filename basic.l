#!/usr/bin/env roseus

;; (load "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
;; (jaxon_red-init)
(load "jaxonred.l")
(load "utils.l")
(setq *robot* *jaxon_red*)
(print (send *robot* :angle-vector (send *ri* :state :potentio-vector)))
(send *ri* :angle-vector (send *robot* :angle-vector))
(start-footguided-modification)
(send *ri* :set-auto-balancer-param
           :is-hand-fix-mode nil
           :ik-mode 1)
(send *ri* :set-ref-forces
    (list
     #f(0 0 650) ;;rleg
     #f(0 0 650) ;;lleg
     #f(0 0 -2) ;;rarm
     #f(0 0 -2) ;;larm
     )
     1000)
(print "press key to start-auto-balancer")
(read-line)
(send *ri* :start-auto-balancer :limbs '(:rleg :lleg))
(print "press key to start-st")
(read-line)
(send *ri* :start-st)
