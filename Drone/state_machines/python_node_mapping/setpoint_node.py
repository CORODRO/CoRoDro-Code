#!/usr/bin/env python
# ROS python API
### https://akshayk07.weebly.com/offboard-control-of-pixhawk.html ###


import rospy

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from time import sleep

###
frequence=10
###

class Controller:
    # initialization method
    def __init__(self):
        # Instantiate a setpoints message
        self.sp = PoseStamped()
        self.sp.header=Header()
        
        # initial values for setpoints
        self.sp.pose.position.x = 0
        self.sp.pose.position.y = 0
        self.sp.pose.position.z = 0
    
        self.sp.pose.orientation.x = 0
        self.sp.pose.orientation.y = 0
        self.sp.pose.orientation.z = 0
        self.sp.pose.orientation.w = 1

    def change_target(self, msg):
        self.sp.pose.position.x = msg.pose.position.x
        self.sp.pose.position.y = msg.pose.position.y
        self.sp.pose.position.z = msg.pose.position.z
	rospy.loginfo("Target received: {} {} {}".format(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))

        if msg.pose.position.z <= -1: # end signal
            print("setpoint_node ending...")
            exit(0)

            
# Main function
def main():

    # initiate node
    rospy.init_node('setpoint_node', anonymous=True)

    cnt = Controller()

    # ROS loop rate
    rate = rospy.Rate(frequence)

    # Setpoint publisher    
    sp_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
    # Orders form SM
    rospy.Subscriber('/SM/orders', PoseStamped, cnt.change_target)

    print "Topics: ok"

    # Waiting for 1st order to be sent
    #_=rospy.wait_for_message('SM/orders', PoseStamped)

    while not rospy.is_shutdown():
        cnt.sp.header.stamp=rospy.Time.now()
    	sp_pub.publish(cnt.sp)          
    	rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
