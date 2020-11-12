(load "package://control_tools/play-hrpsysfile.l")
(load "~/catkin_ws/euslib/demo/kojio/push-recovery/push-recovery-foot-guided.l")

(send *ri* :set-log-maxlength 150000)
(send *ri* :start-log)
