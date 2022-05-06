#!/usr/bin/env python
# ROS python API
import rospy

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
import std_msgs
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

def main():
	rospy.init_node('setpoint_node', anonymous=True)
	x=0
	pub=rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
	rate=rospy.Rate(20)

	while not rospy.is_shutdown():
		pub.publish(AttitudeTarget(header=std_msgs.msg.Header(seq=x,stamp=rospy.Time.now(),frame_id="base_link"),type_mask=128+7, thrust=1))
		x+=1
		rate.sleep()

main()
