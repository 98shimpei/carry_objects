#!/usr/bin/env python

import rospy
import sys
import datetime
from geometry_msgs.msg import WrenchStamped


class Takelog:
  def __init__(self):
    dt_now = datetime.datetime.now()
    args = sys.argv
    if len(args) == 1:
      filename = "log/"+dt_now.strftime("%m%d%H%M%S")+".log"
    else:
      filename = "log/"+args[1]+"-"+dt_now.strftime("%m%d%H%M%S")+".log"
    self.logfile = open(filename, mode="w")
    self.listener()

  def __del__(self):
    self.logfile.close()

  def callback(self, data):
    print(data.header.stamp.secs)
    self.logfile.write(
      str(data.header.stamp.secs) + str(float(data.header.stamp.nsecs / 1000000)/1000).lstrip("0") + " " + 
      str(data.wrench.force.x) + " " +
      str(data.wrench.force.y) + " " +
      str(data.wrench.force.z) + " " +
      str(data.wrench.torque.x) + " " +
      str(data.wrench.torque.y) + " " +
      str(data.wrench.torque.z) +
      "\n")

  def listener(self):
    rospy.init_node('loglistener')
    rospy.Subscriber("rhsensor", WrenchStamped, self.callback)
    rospy.spin()

if __name__=="__main__":
  logger = Takelog()
