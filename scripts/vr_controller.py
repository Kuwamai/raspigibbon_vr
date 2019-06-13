#!/usr/bin/env python
# coding: utf-8

import time
import rospy
from geometry_msgs.msg import PoseStamped

class Pose_pub:
    def __init__(self):
        self._sub_pos = rospy.Subscriber("controller_r", PoseStamped, self.pose_callback)
        self.r = rospy.Rate(10)

    def pose_callback(self, message):
        rospy.loginfo(message.pose)
        self.pose = message.pose
        
    def reset_pose(self):
        self.zero_pose = self.pose

    def ik(self):
        while not rospy.is_shutdown():
            rospy.loginfo(self.pose.position.x - self.zero_pose.position.x)
            self.r.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('vr_controller')
        pose_pub = Pose_pub()
        time.sleep(0.1)
        pose_pub.reset_pose()
        pose_pub.ik()
        rospy.spin()

    except rospy.ROSIntteruptException:
        pass
