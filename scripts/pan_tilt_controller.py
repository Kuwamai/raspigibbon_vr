#!/usr/bin/env python
# coding: utf-8

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import numpy as np

class Pose_pub:
    def __init__(self):
        self._sub_pos = rospy.Subscriber("head", PoseStamped, self.pose_callback)
        self.pub = rospy.Publisher("/raspigibbon/master_joint_state", JointState, queue_size=10)
        
        #コントローラの初期位置を取得
        self.zero_pose = rospy.wait_for_message("head", PoseStamped).pose
        quaternion = [self.zero_pose.orientation.x, self.zero_pose.orientation.y, self.zero_pose.orientation.z, self.zero_pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(quaternion, axes='rzyx')
        self.zero_pan = euler[0]
        #10Hzで動作
        self.r = rospy.Rate(10)
        #コントローラ位置のスケール
        self.scale_fac = 1.
        #アーム手先位置のオフセット
        self.r_offset = 0.8
        self.q_old = np.array([0., 0., 0., 0., 0., 0.])
        #最大関節角速度
        self.max_vel = 0.5

    def pose_callback(self, message):
        self.pose = message.pose

    #逆運動学計算
    def ik(self):
        while not rospy.is_shutdown():
            #目標手先位置
            r_ref = self.pose.position.z - self.zero_pose.position.z
            
            #位置のスケール
            r_ref *= self.scale_fac

            #アーム手先位置のオフセット
            r_ref += self.r_offset

            #手先位置が稼働範囲内に収まっているかチェック
            r_ref = self.check_movable_range(r_ref)

            theta = np.arccos(r_ref)

            pan, tilt, _ = self.calc_pan_tilt_angle()
            rospy.loginfo(pan)

            q = np.array([-pan - self.zero_pan, theta, -2 * theta, -tilt + theta, 0, 0])

            q = self.angular_vel_limit(q)

            q_deg = np.rad2deg(q)

            js = JointState()
            js.name=["joint{}".format(i) for i in range(1,6)]
            js.position = q_deg
            self.pub.publish(js)
            self.r.sleep()

    #角速度制限
    def angular_vel_limit(self, q):
        q_diff = self.q_old - q
        q_diff_max = np.abs(q_diff).max()

        if(q_diff_max > self.max_vel):
            rospy.loginfo("Too fast")
            q_diff /= q_diff_max
            q_diff *= self.max_vel
            q = self.q_old - q_diff

        self.q_old = q
        return q

    #ピッチ角計算
    def calc_pan_tilt_angle(self):
        quaternion = [self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(quaternion, axes='rzyx')
        return euler

    def check_movable_range(self, r_ref):
        if r_ref > 1:
            rospy.loginfo("Out of movable range")
            r_ref = 1

        return r_ref

if __name__ == '__main__':
    try:
        rospy.init_node('pan_tilt_controller')
        pose_pub = Pose_pub()
        pose_pub.ik()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
