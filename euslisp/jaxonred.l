(load "package://control_tools/play-hrpsysfile.l")
(load "package://carry_objects/../euslib/demo/kojio/push-recovery/push-recovery-foot-guided.l")
;; (load "~/catkin_ws/sato/src/euslib/demo/kojio/push-recovery/vnoid_kojio_param.l")
;; (load "~/catkin_ws/sato/src/euslib/demo/kojio/push-recovery/push-recovery-foot-guided.l")

(send *ri* :set-log-maxlength 150000)
(send *ri* :start-log)
