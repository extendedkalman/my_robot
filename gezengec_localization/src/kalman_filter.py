#!/usr/env/bin python3

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

class KalmanFilter(object):
    def __init__(self) -> None:
        self.odom_sub_ = rospy.Subscriber("gezengec_controller/odom_noisy", Odometry, self.odomCallback)
        self.imu_sub_ = rospy.Subscriber("imu", Imu, self.imuCallback)