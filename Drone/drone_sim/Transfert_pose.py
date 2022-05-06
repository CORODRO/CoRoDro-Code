#!/usr/bin/env python
# ROS python API
import rospy

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import std_msgs
# import all mavros messages and services



def callback(m):
    path=PoseStamped(header=std_msgs.msg.Header(seq=m.header.seq,stamp=rospy.Time.now(),frame_id="base_link"),pose=m.pose.pose)
    pub.publish(path)

rospy.init_node("Pose_transfert")
pub=rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)
sub=rospy.Subscriber('/T265/odom/sample', Odometry, callback)

rate=rospy.Rate(60)

try:
	while not rospy.is_shutdown():
  		rate.sleep()
except:
	pass