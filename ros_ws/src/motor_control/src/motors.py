#!/usr/bin/env python3
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from tf.transformations import quaternion_from_euler
from tf2_geometry_msgs import do_transform_pose
from std_msgs.msg import Int16MultiArray
from nav_msgs.msg import Odometry
from init_param import Parameters
from math import sin, cos, pi
import RPi.GPIO as GPIO
from pid import PID
import rospy
import time
import tf
import os


class MotorControl():
    def __init__(self):
        os.nice(19)
        GPIO.setmode(GPIO.BCM)
        self.param = Parameters()
        self.odom = Odometry()
        self.odom.child_frame_id = self.param.base_frame_id
        self.odom.header.frame_id = self.param.odom_frame_id
        self.odom.pose.covariance = [0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 1000000, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 1000000, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 1000000, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.0, 1000000]
        self.pose_stamped = PoseStamped()
        self.pose_stamped.header.frame_id = self.param.base_frame_id
        self.transform_stamped = TransformStamped()
        self.transform_stamped.child_frame_id = self.param.base_frame_id
        self.transform_stamped.header.frame_id = self.param.odom_frame_id
        self.rate = rospy.Rate(self.param.frequency)
        rospy.Subscriber('cmd_vel', Twist, self.speed_update)
        self.moto_msg = Int16MultiArray()
        self.pub_arduino_moto = rospy.Publisher('arduino_moto', Int16MultiArray, queue_size=10)
        self.speed_odom_linear_x = 0
        self.speed_odom_angular_z = 0
        self.speed_left_wheel = 0
        self.speed_right_wheel = 0
        self.pid_left = PID(self.param)
        self.pid_right = PID(self.param)
        self.pub_odometry = rospy.Publisher(self.param.odom_frame_id, Odometry, queue_size=10)
        self.transformed_pose = PoseStamped()
        self.transformed_pose.header.frame_id = self.param.base_frame_id
        self.transformed_pose.header.stamp = rospy.Time.now()
        self.sequence = 0
        self.brodcaster = tf.TransformBroadcaster()
        rospy.Subscriber('arduino_odom', Int16MultiArray, self.pub_tf_odom_base)

    def speed_update(self, data):
        linear = data.linear.x
        angular = data.angular.z
        ang_left = -angular*self.param.wheel_separation/2
        ang_right = -ang_left
        self.speed_left_wheel = linear + ang_left
        self.speed_right_wheel = linear + ang_right

    def publish_odom(self, pose):
        self.odom.header.seq = self.sequence
        self.odom.header.stamp = rospy.Time.now()
        self.odom.pose.pose = pose
        self.odom.twist.twist.linear.x = self.speed_odom_linear_x
        self.odom.twist.twist.angular.z = self.speed_odom_angular_z
        self.pub_odometry.publish(self.odom)

    def get_transform_pose(self):
        self.transform_stamped.header.seq = self.sequence
        self.transform_stamped.header.stamp = rospy.Time.now()
        self.transform_stamped.transform.translation.x = self.transformed_pose.pose.position.x
        self.transform_stamped.transform.translation.y = self.transformed_pose.pose.position.y
        self.transform_stamped.transform.rotation.z = self.transformed_pose.pose.orientation.z
        self.transform_stamped.transform.rotation.w = self.transformed_pose.pose.orientation.w
        return self.transform_stamped

    def position_transform(self, enkoder_left, enkoder_right):
        lft = enkoder_left/self.param.encoder_pulses_rev*pi*self.param.wheel_diameter
        rght = enkoder_right/self.param.encoder_pulses_rev*pi*self.param.wheel_diameter
        abs_lft = abs(lft)
        abs_rght = abs(rght)
        if lft == rght:
            x = lft
            y = 0 
            alfa = 0
        elif abs_rght > abs_lft and rght*lft > 0:
            r = abs_rght*self.param.wheel_separation/(abs_rght-abs_lft)
            alfa = rght/r
            x = (r - self.param.wheel_separation/2)*sin(alfa)
            y = (r - self.param.wheel_separation/2)*(1-cos(alfa))
        elif abs_rght > abs_lft and rght*lft <= 0:
            r = abs_rght*self.param.wheel_separation/(abs_rght+abs_lft)
            alfa = rght/r
            x = (r - self.param.wheel_separation/2)*sin(alfa)
            y = (r - self.param.wheel_separation/2)*(1-cos(alfa))
        elif abs_rght < abs_lft and rght*lft > 0:
            r = abs_lft*self.param.wheel_separation/(abs_lft-abs_rght)
            alfa = -lft/r
            x = -(r - self.param.wheel_separation/2)*sin(alfa)
            y = (r - self.param.wheel_separation/2)*(cos(alfa)-1)
        elif abs_rght < abs_lft and rght*lft <= 0:
            r = abs_lft*self.param.wheel_separation/(abs_lft+abs_rght)
            alfa = -lft/r
            x = -(r - self.param.wheel_separation/2)*sin(alfa)
            y = (r - self.param.wheel_separation/2)*(cos(alfa)-1)
        elif rght == -lft:
            alfa = rght/(self.param.wheel_separation/2)
            x = 0
            y = 0
        return x, y, alfa

    def get_pose_move(self, steps_left, steps_right):
        x, y, alfa = self.position_transform(steps_left, steps_right)
        _, _, z, w = quaternion_from_euler(0,0, alfa)
        self.pose_stamped.header.stamp = rospy.Time.now()
        self.pose_stamped.header.seq = self.sequence
        self.pose_stamped.pose.position.x = x
        self.pose_stamped.pose.position.y = y
        self.pose_stamped.pose.orientation.z = z
        self.pose_stamped.pose.orientation.w = w
        return self.pose_stamped

    def pub_tf_odom_base(self, data):
        steps_left, steps_right, time_ms = data.data
        speed_left, speed_right = self.wheels_speed(steps_left, steps_right, time_ms)

        self.speed_calc(speed_left, speed_right)
        pid_l = self.pid_left.pid(self.speed_left_wheel, speed_left)
        pid_r = self.pid_right.pid(self.speed_right_wheel, speed_right)
        self.call_motor(pid_l, pid_r)

        pose_stamped = self.get_pose_move(steps_left, steps_right)
        transform = self.get_transform_pose()
        self.transformed_pose = do_transform_pose(pose_stamped, transform)
        pose = self.transformed_pose.pose.position
        position = (pose.x ,pose.y, pose.z)
        orient = self.transformed_pose.pose.orientation
        orientation = (orient.x, orient.y, orient.z, orient.w)
        self.publish_odom(self.transformed_pose.pose)
        self.brodcaster.sendTransform(position, orientation, rospy.Time.now(),
                                        self.param.base_frame_id,self.param.odom_frame_id)
        self.sequence +=1

    def speed_calc(self, speed_left, speed_right):
        '''converts linear speed of each wheel into linear and angular'''
        self.speed_odom_linear_x = (speed_left + speed_right) / 2
        self.speed_odom_angular_z = -(speed_left - self.speed_odom_linear_x) / (self.param.wheel_separation/2)

    def call_motor(self,  l_pid_value, r_pid_value):
        l_reverse = 0 if l_pid_value < 0 else 1
        # r_reverse = 1 if l_pid_value < 0 else 1
        r_reverse = 0 if r_pid_value < 0 else 1
        self.moto_msg.data = [l_reverse, int(l_pid_value*255), r_reverse, int(r_pid_value*255)]
        self.pub_arduino_moto.publish(self.moto_msg)

    def calc_wheel_speed(self, steps, time_ms):
        # calc wheel speed in m/s
        return (steps/time_ms)/self.param.encoder_pulses_rev*1000*pi*self.param.wheel_diameter

    def wheels_speed(self, steps_left, steps_right, time_ms):
        speed_left = self.calc_wheel_speed(steps_left, time_ms)
        speed_right = self.calc_wheel_speed(steps_right, time_ms)
        return speed_left, speed_right

    def control_loop(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
