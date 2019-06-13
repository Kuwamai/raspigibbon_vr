#!/usr/bin/env python
# coding: utf-8

import time
import rospy
from geometry_msgs.msg import PoseStamped

class Pose_pub:
    def __init__(self):
        self._sub_pos = rospy.Subscriber("controller_r", PoseStamped, self.pose_callback)
        self.reset_f = True

    def pose_callback(self, message):
        self.pose = message.pose
        rospy.loginfo(message.pose)

        if self.reset_f:
            self.reset_pose()
            self.reset_f = False

        else:
            self.ik()
        
    def reset_pose(self):
        self.zero_pose = self.pose

    def ik(self):
        rospy.loginfo(self.pose.position.x - self.zero_pose.position.x)

if __name__ == '__main__':
    try:
        rospy.init_node('vr_controller')
        pose_pub = Pose_pub()
        rospy.spin()

    except rospy.ROSIntteruptException:
        pass
