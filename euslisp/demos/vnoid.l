#!/usr/bin/env roseus

(load "~/catkin_ws/kojio/src/rtm-ros-robotics/rtmros_tutorials/hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(unless (boundp '*robot*)
  (jaxon_red-init)
  (setq *robot* *jaxon_red*)
  )

(load "~/catkin_ws/sato/src/euslib/demo/kojio/util/robot-init.l")
(load "~/catkin_ws/sato/src/euslib/demo/kojio/push-recovery/vnoid_kojio_param.l")
;; (vision-mode)
;; (send *robot* :head :angle-vector #f(0 20))
;; (send *ri* :angle-vector (send *robot* :angle-vector))
;; (print "ここでomori2simを上げ直す。終わったらEnterでstair-mode")
;; (read-line)
;; (stair-mode)
;; (send *robot* :head :angle-vector #f(0 -10))
;; (send *ri* :angle-vector (send *robot* :angle-vector))
;; (print "go-velを呼ぶ")
