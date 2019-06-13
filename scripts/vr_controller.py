#!/usr/bin/env python
# coding: utf-8

import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np

class Pose_pub:
    def __init__(self):
        self._sub_pos = rospy.Subscriber("controller_r", PoseStamped, self.pose_callback)
        self.reset_f = True
        self.scale_fac = 0.1

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
        r_ref = np.array([[self.pose.position.x - self.zero_pose.position.x],
                          [self.pose.position.y - self.zero_pose.position.y],
                          [self.pose.position.z - self.zero_pose.position.z]])

        rospy.loginfo(r_ref)
        r_ref *= self.scale_fac
        rospy.loginfo(r_ref)

if __name__ == '__main__':
    try:
        rospy.init_node('vr_controller')
        pose_pub = Pose_pub()
        rospy.spin()

    except rospy.ROSIntteruptException:
        pass
