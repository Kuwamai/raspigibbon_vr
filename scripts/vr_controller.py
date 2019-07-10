#!/usr/bin/env python
# coding: utf-8

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import numpy as np

class Pose_pub:
    def __init__(self):
        self._sub_pos = rospy.Subscriber("controller_r", PoseStamped, self.pose_callback)
        self._sub_tri = rospy.Subscriber("trigger_r", Float64, self.trigger_callback)
        self.pub = rospy.Publisher("/raspigibbon/master_joint_state", JointState, queue_size=10)
        
        #コントローラの初期位置を取得
        self.zero_pose = rospy.wait_for_message("controller_r", PoseStamped).pose
        self.r = rospy.Rate(10)
        #コントローラ位置のスケール
        self.scale_fac = 1.
        #アーム手先位置のオフセット
        self.offset = 0.5
        #関節角最大速度
        self.max_vel = 0.3
        #逆運動学計算用初期値
        self.q = self.q_old = np.array([[0.],
                                        [0.4],
                                        [-2.]])

    def pose_callback(self, message):
        self.pose = message.pose

    def trigger_callback(self, message):
        self.trigger = message.data

    #逆運動学計算
    def ik(self):
        while not rospy.is_shutdown():
            #目標手先位置
            r_ref = np.array([[self.pose.position.x - self.zero_pose.position.x + self.offset],
                              [self.pose.position.y - self.zero_pose.position.y],
                              [self.pose.position.z - self.zero_pose.position.z + self.offset]])

            #コントローラ位置のスケール
            r_ref *= self.scale_fac
            #rospy.loginfo(r_ref)

            #コントローラ位置がアームの可動範囲2を超えた際は1.9にスケールする
            r_ref_norm = np.linalg.norm(r_ref, ord=2)
            
            if r_ref_norm > 2.0:
                rospy.loginfo("Out of movable range")
                r_ref /= r_ref_norm
                r_ref *= 1.99

            #数値計算
            for i in range(10):
                r = self.fk(self.q)
                self.q = self.q - np.linalg.inv(self.J(self.q)).dot((r - r_ref))

            #rospy.loginfo(self.q.T)
            #rospy.loginfo(r_ref - r)

            self.angular_vel_limit()

            q_deg = np.rad2deg(self.q)
            grip  = self.trigger * 90.

            js = JointState()
            js.name=["joint{}".format(i) for i in range(1,6)]
            js.position = [q_deg[0,0], q_deg[1,0], q_deg[2,0], 0.0, -grip, grip]
            self.pub.publish(js)
            self.r.sleep()

    #同次変換行列
    def trans_m(self, a, alpha, d, theta):
        m = np.array([[np.cos(theta), -np.sin(theta), 0., a],
            [np.cos(alpha)*np.sin(theta), np.cos(alpha)*np.cos(theta), -np.sin(alpha), -np.sin(alpha)*d],
            [np.sin(alpha)*np.sin(theta), np.sin(alpha)*np.cos(theta),  np.cos(alpha),  np.cos(alpha)*d],
            [0., 0., 0., 1.]])
        return m

    #順運動学
    def fk(self, theta):
        tm0_1 = self.trans_m(0, 0,       0, theta[0,0])
        tm1_2 = self.trans_m(0, np.pi/2, 0, theta[1,0]+np.pi/2)
        tm2_3 = self.trans_m(1, 0,       0, theta[2,0])
        tm3_4 = self.trans_m(1, 0,       0, 0)
        pos = tm0_1.dot(tm1_2).dot(tm2_3).dot(tm3_4)[0:3,3:4]
        return pos

    #ヤコビ行列
    def J(self, theta):
        e = 1.0e-10
        diff_q1 = (self.fk(theta+np.array([[e],[0.],[0.]]))-self.fk(theta))/e
        diff_q2 = (self.fk(theta+np.array([[0.],[e],[0.]]))-self.fk(theta))/e
        diff_q3 = (self.fk(theta+np.array([[0.],[0.],[e]]))-self.fk(theta))/e
        return np.hstack((diff_q1, diff_q2, diff_q3))

    #角速度制限
    def angular_vel_limit(self):
        q_diff = self.q_old - self.q
        q_diff_max = np.abs(q_diff).max()

        if(q_diff_max > self.max_vel):
            rospy.loginfo("Too fast")
            q_diff /= q_diff_max
            q_diff *= self.max_vel
            self.q += q_diff

        self.q_old = self.q

if __name__ == '__main__':
    try:
        rospy.init_node('vr_controller')
        pose_pub = Pose_pub()
        pose_pub.ik()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
