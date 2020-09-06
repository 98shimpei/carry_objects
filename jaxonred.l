(require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
(unless (boundp '*jaxon_red*)
  (jaxon_red-init)
  (setq *robot* *jaxon_red*)
  )


(defun start-footguided-modification
   ()
 (let* ((step-time 0.7)
        (double-ratio 0.1))
       (cond ((boundp '*chidori*)
              (send *ri* :set-gait-generator-param
                    :leg-margin #f(0.15 0.09 0.065 0.065)
                    :safe-leg-margin #f(0.13 0.07 0.05 0.05)
                    ))
             ((boundp '*jaxon_red*)
              (send *ri* :set-gait-generator-param
                    :leg-margin #f(0.12 0.09 0.05 0.05)
                    :safe-leg-margin #f(0.07 0.055 0.05 0.05)
                    :dcm-offset 0.03
                    ))
             ((boundp '*rhp4b*)
              (send *ri* :set-gait-generator-param
                    :leg-margin #f(0.12 0.09 0.07 0.06)
                    :safe-leg-margin #f(0.07 0.055 0.07 0.06)
                    :dcm-offset 0.02
                    ))
             )
       (send *ri* :set-gait-generator-param
             :stride-limitation-type 1
             :stride-limitation-for-circle-type #f(0.15 0.3 15 0.1 0.14)
             :default-orbit-type :rectangle
             :default-step-height 0.05
             :default-step-time step-time
             :default-double-support-ratio double-ratio
             :swing-trajectory-delay-time-offset (* step-time (- 1.0 double-ratio) 0.25)
             :overwritable-stride-limitation #f(0.35 0.5 0 0.35 0.14)
             :margin-time-ratio 0.1
             :min-time-mgn 0.3
             :min-time 0.6
             :use-disturbance-compensation t
             :zmp-delay-time-const 0.05
             :dc-gain 1e-3
             :emergency-step-time #f(0.08 0.6 0.7)
             :modify-footsteps t
             )
       (send *ri* :set-auto-balancer-param
             :is-emergency-step-mode t
             :is-emergency-touch-wall-mode nil
             :touch-wall-retrieve-time 0.4
             )
       ))

(defun start-footguided-modification-zikki
   ()
 (let* ((step-time 1.2)
        (double-ratio 0.2))
       (cond ((boundp '*chidori*)
              (send *ri* :set-gait-generator-param
                    :leg-margin #f(0.15 0.09 0.065 0.065)
                    :safe-leg-margin #f(0.13 0.07 0.05 0.05)
                    ))
             ((boundp '*jaxon_red*)
              (send *ri* :set-gait-generator-param
                    :leg-margin #f(0.12 0.09 0.05 0.05) 
                    :safe-leg-margin #f(0.07 0.055 0.05 0.05)
                    :dcm-offset 0.01
                    ))
             ((boundp '*rhp4b*)
              (send *ri* :set-gait-generator-param
                    :leg-margin #f(0.12 0.09 0.07 0.06) 
                    :safe-leg-margin #f(0.07 0.055 0.07 0.06)
                    :dcm-offset 0.02
                    ))
             )
       (send *ri* :set-gait-generator-param
             :stride-limitation-type 1
             :stride-limitation-for-circle-type #f(0.15 0.3 15 0.1 0.14)
             :default-orbit-type :rectangle
             :default-step-height 0.05
             :default-step-time step-time
             :default-double-support-ratio double-ratio
             :swing-trajectory-delay-time-offset (* step-time (- 1.0 double-ratio) 0.25)
             :overwritable-stride-limitation #f(0.35 0.5 0 0.35 0.14)
             :margin-time-ratio 0.1
             :min-time-mgn 0.3
             :min-time 1.0
             :use-disturbance-compensation nil
             :zmp-delay-time-const 0.05
             :dc-gain 1e-3
             :emergency-step-time #f(0.08 0.6 0.7)
             :use-act-states nil
             :modify-footsteps nil
             )
       (send *ri* :set-auto-balancer-param
             :is-emergency-step-mode nil
             :is-emergency-touch-wall-mode nil
             :touch-wall-retrieve-time 0.4
             )
       ))


(defun push-init
  (&optional (use-hand? nil))
  (send *robot* :angle-vector (send *ri* :state :potentio-vector))
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (send *ri* :autobalancerservice_stopStabilizer)
  (send *ri* :stop-auto-balancer)
  (send *ri* :set-auto-balancer-param
             :is-hand-fix-mode t
             :ik-mode 1)
  (if use-hand?
    (send *ri* :start-auto-balancer :limbs '(:larm :rarm :rleg :lleg))
    (send *ri* :start-auto-balancer :limbs '(:rleg :lleg)))
  (send *ri* :autobalancerservice_startStabilizer)
  )

;; (start-footguided-modification)
