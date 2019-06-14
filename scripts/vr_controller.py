#!/usr/bin/env python
# coding: utf-8

import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np

class Pose_pub:
    def __init__(self):
        self._sub_pos = rospy.Subscriber("controller_r", PoseStamped, self.pose_callback)
        self.r = rospy.Rate(10)
        self.reset_f = True
        self.scale_fac = 0.5
        self.z_offset = 0.05
        self.q = np.array([[0.0],
                           [0.2],
                           [2.5]])

    def pose_callback(self, message):
        self.pose = message.pose
        #rospy.loginfo(message.pose)

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
                          [self.pose.position.z - self.zero_pose.position.z + self.z_offset]])

        #rospy.loginfo(r_ref)
        r_ref *= self.scale_fac
        #rospy.loginfo(r_ref)

        r_ref_norm = np.linalg.norm(r_ref, ord=2)
        
        if r_ref_norm > 2.0:
            rospy.loginfo("wawawa")
            r_ref /= r_ref_norm
            r_ref *= 1.99

        for i in range(10):
            r = self.fk(self.q)
            self.q = self.q - np.linalg.inv(self.J(self.q)).dot((r - r_ref))

        rospy.loginfo(self.q.T)
        rospy.loginfo(r_ref - r)
        self.r.sleep()

    def trans_m(self, a, alpha, d, theta):
        m = np.array([[np.cos(theta), -np.sin(theta), 0., a],
            [np.cos(alpha)*np.sin(theta), np.cos(alpha)*np.cos(theta), -np.sin(alpha), -np.sin(alpha)*d],
            [np.sin(alpha)*np.sin(theta), np.sin(alpha)*np.cos(theta),  np.cos(alpha),  np.cos(alpha)*d],
            [0., 0., 0., 1.]])
        return m

    def fk(self, theta):
        tm0_1 = self.trans_m(0, 0,       0, theta[0,0])
        tm1_2 = self.trans_m(0, np.pi/2, 0, theta[1,0])
        tm2_3 = self.trans_m(1, 0,       0, theta[2,0])
        tm3_4 = self.trans_m(1, 0,       0, 0)
        pos = tm0_1.dot(tm1_2).dot(tm2_3).dot(tm3_4)[0:3,3:4]
        return pos

    def J(self, theta):
        e = 1.0e-10
        diff_q1 = (self.fk(theta+np.array([[e],[0.],[0.]]))-self.fk(theta))/e
        diff_q2 = (self.fk(theta+np.array([[0.],[e],[0.]]))-self.fk(theta))/e
        diff_q3 = (self.fk(theta+np.array([[0.],[0.],[e]]))-self.fk(theta))/e
        return np.hstack((diff_q1, diff_q2, diff_q3))

if __name__ == '__main__':
    try:
        rospy.init_node('vr_controller')
        pose_pub = Pose_pub()
        rospy.spin()

    except rospy.ROSIntteruptException:
        pass
