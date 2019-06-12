#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

class Pose_pub:
  def __init__(self):
    self._sub_pos = rospy.Subscriber("controller_r", PoseStamped, self.pose_callback)
    
  def pose_callback(self, message):
    rospy.loginfo(message.pose.position)

if __name__ == '__main__':
  rospy.init_node('vr_controller')
  pose_pub = Pose_pub()
  rospy.spin()
