#!/usr/bin/env python

import rospy

import message_filters
from geometry_msgs.msg import Vector3Stamped, TwistStamped, TwistWithCovarianceStamped
from sensor_msgs.msg import Imu

rospy.init_node("message_filter")

imu_pub = rospy.Publisher("imu/data_raw", Imu, queue_size=5)
odom_pub = rospy.Publisher(
    "wheel_odom_with_covariance", TwistWithCovarianceStamped, queue_size=5)
imu_msg = Imu()
odom_msg = TwistWithCovarianceStamped()

wheel_odom_cov = rospy.get_param("~wheel_odom_covariance_diagonal")
imu_ang_vel_cov = rospy.get_param("~imu_angular_velocity_covariance_diagonal")
imu_lin_acc_cov = rospy.get_param(
    "~imu_linear_acceleration_covariance_diagonal")
imu_orient_cov = rospy.get_param(
    "~imu_orientation_convariance")

for i in range(6):
    odom_msg.twist.covariance[i*6] = wheel_odom_cov[i]

for i in range(3):
    
    imu_msg.angular_velocity_covariance[i*3] = imu_ang_vel_cov[i]
    imu_msg.linear_acceleration_covariance[i*3] = imu_lin_acc_cov[i]


def imu_callback(imu):
    imu_msg.header.stamp = imu.header.stamp
    imu_msg.header.frame_id = imu.header.frame_id
    

    imu_msg.orientation = imu.orientation
    imu_msg.angular_velocity = imu.angular_velocity
    imu_msg.linear_acceleration = imu.linear_acceleration

    imu_pub.publish(imu_msg)


def odom_callback(odom):
    odom_msg.header.stamp = odom.header.stamp
    odom_msg.twist.twist = odom.twist
    odom_msg.twist.twist.angular.z = odom.twist.angular.z/2

    odom_pub.publish(odom_msg)


rospy.wait_for_message("sbg_out_Imu", Imu)

accel_sub = rospy.Subscriber("sbg_out_Imu", Imu, imu_callback)
odom_sub = rospy.Subscriber("wheel_odom", TwistStamped, odom_callback)



rospy.spin()


