#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "roseus")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "ar_track_alvar_msgs")

(ros::roseus "test_sub_eus")

(defun cb (msg)
  (print "sub_eus"))

(ros::subscribe "/test" ar_track_alvar_msgs::AlvarMarkers #'cb)

(ros::rate 10)
(do-until-key
  (ros::spin-once)
  (ros::sleep)
  )

