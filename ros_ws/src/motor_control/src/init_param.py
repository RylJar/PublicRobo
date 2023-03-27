#!/usr/bin/env python3
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped, TransformStamped
from tf.transformations import quaternion_from_euler
from tf2_geometry_msgs import do_transform_pose
from nav_msgs.msg import Odometry
from math import sin, cos, pi
from pid import PID
import RPi.GPIO as GPIO
import rospy
import time
import tf
import os

# silnik max 3obr/s

class Parameters():
    '''read parameters from parameter server'''
    def __init__(self):
        self.base_frame_id = rospy.get_param("~base_frame_id")
        self.odom_frame_id = rospy.get_param("~odom_frame_id")
        self.wheel_diameter = rospy.get_param("~wheel_diameter")
        self.wheel_separation = rospy.get_param("~wheel_separation")
        self.frequency = rospy.get_param("~frequency")
        self.encoder_pulses_rev = rospy.get_param("~encoder_pulses_rev")
        self.kp = rospy.get_param("~kp")
        self.ki = rospy.get_param("~ki")
        self.kd = rospy.get_param("~kd")
        self.integrator_max = rospy.get_param("~integrator_max")
        self.limit_max = rospy.get_param("~pid_max")
        self.limit_min = rospy.get_param("~pid_min")
