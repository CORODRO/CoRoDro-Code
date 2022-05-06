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
ALT=1.5 # Altitude
K=[3, 3, 1] #PID coeff
MAX_VELOCITY=0.5 #max velocity
EPSILON=[0.2, 0.2, 0.2] #epsilon for position precision
NUMBER_OF_TIME_ON_POSE=10 #nombre de fois qu'il doit etre "arrived" pour le croire (pour rappel voir frequence pour le temps)
###

# Flight modes class
# Flight modes are activated using ROS services
class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self):
    	rospy.wait_for_service('mavros/cmd/takeoff')
    	try:
    		takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
    		takeoffService(altitude = ALT)
    	except rospy.ServiceException, e:
    		print "Service takeoff call failed: %s"%e

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s"%e

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Altitude Mode could not be set."%e

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Position Mode could not be set."%e

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException, e:
               print "service set_mode call failed: %s. Autoland Mode could not be set."%e


    def setMaxVelocity(self,velocity):
        rospy.wait_for_service('mavros/param/set')
        try:
            set_max_vel = rospy.ServiceProxy('mavros/param/set', mavros_msgs.srv.ParamSet)
            set_max_vel("MPC_XY_VEL_MAX",velocity)
            set_max_vel("MPC_XY_CRUISE",velocity)
        except rospy.ServiceException, e:
               print "service param_set_velocity call failed: %s. Velocity param could not be set."%e        

class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PoseStamped()
        self.sp.header=Header()
        # PID coefficient and velocity parameters
        self.K=K
        self.MAX_VELOCITY=MAX_VELOCITY
        self.EPSILON=EPSILON

        # Real position
        msg=rospy.wait_for_message("T265/odom/sample", Odometry)
        self.real_zero_alt = msg.pose.pose.position.z - 0.05
        self.linear = Point(0,0,self.real_zero_alt)
        self.angular = Point(0,0,0)

        # Target position
        self.target_linear = Point(0,0,0)
        self.target_angular = Point(0,0,0)

        # Previous position
        self.previous_linear = Point(0,0,0)
        self.previous_angular = Point(0,0,0)

        # initial values for setpoints
        self.sp.pose.position.x = 0.0
        self.sp.pose.position.y = 0.0
        self.sp.pose.position.z = ALT
    
        self.sp.pose.orientation.x = 0.0
        self.sp.pose.orientation.y = 0.0
        self.sp.pose.orientation.z = 0
        self.sp.pose.orientation.w = 1

        # speed of the drone is set using MPC_XY_CRUISE parameter in MAVLink
        # using QGroundControl. By default it is 5 m/s.

	# Callbacks

    ## local position callback
    def posCb(self, msg):
        self.linear.x = msg.pose.pose.position.x
        self.linear.y = msg.pose.pose.position.y
        self.linear.z = msg.pose.pose.position.z
        self.angular.x = msg.pose.pose.orientation.x
        self.angular.y = msg.pose.pose.orientation.y
        self.angular.z = msg.pose.pose.orientation.z
        # self.linear.x = msg.pose.position.x
        # self.linear.y = msg.pose.position.y
        # self.linear.z = msg.pose.position.z
        # self.angular.x = msg.pose.orientation.x
        # self.angular.y = msg.pose.orientation.y
        # self.angular.z = msg.pose.orientation.z


    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    def is_arrived_x(self):
        return abs(self.linear.x - self.target_linear.x) <= self.EPSILON[0]
    
    def is_arrived_y(self):
        return abs(self.linear.y - self.target_linear.y) <= self.EPSILON[1]
    
    def is_arrived_z(self):
        return abs(self.linear.z - self.target_linear.z) <= self.EPSILON[2]
    
    def is_arrived(self):
        return  self.is_arrived_x() and self.is_arrived_y() and self.is_arrived_z()

    # def update(self):
    #     signe_x = ((self.target_linear.x > self.linear.x) - 1 ) * 2 + 1
    #     signe_y = ((self.target_linear.y > self.linear.y) - 1 ) * 2 + 1
    #     signe_z = ((self.target_linear.z > self.linear.z) - 1 ) * 2 + 1

    #     self.sp.twist.linear.x = signe_x * min ( min( K[0]*(abs( self.target_linear.x - self.linear.x)), K[0]*(abs( self.previous_linear.x - self.linear.x ))), self.MAX_VELOCITY)
    #     self.sp.twist.linear.y = signe_y * min ( min( K[1]*(abs( self.target_linear.y - self.linear.y)), K[1]*(abs( self.previous_linear.y - self.linear.y ))), self.MAX_VELOCITY)
    #     self.sp.twist.linear.z = signe_z * min( K[2]*abs( self.target_linear.z - self.linear.z) , self.MAX_VELOCITY)

    #     # if self.is_arrived_x():
    #     #     self.sp.twist.linear.x = 0
    #     # if self.is_arrived_y():
    #     #     self.sp.twist.linear.y = 0
    #     # if self.is_arrived_z():
    #     #     self.sp.twist.linear.z = 0
        
        

    def change_target(self, couple):
        self.previous_linear.x = self.target_linear.x
        self.previous_linear.y = self.target_linear.y
        self.target_linear.x = couple[0]
        self.target_linear.y = couple[1]
        self.sp.pose.position.x = couple[0]
        self.sp.pose.position.y = couple[1]
    
    def change_alt(self, z):
        self.previous_linear.z = self.target_linear.z
        self.target_linear.z = z
        self.sp.pose.position.z = z
    




# Main function
def main():

    # initiate node
    rospy.init_node('setpoint_node', anonymous=True)

    # flight mode object
    modes = fcuModes()
    print "fcu"
    # controller object
    cnt = Controller()

    # ROS loop rate
    rate = rospy.Rate(frequence)

    # Subscribe to drone state
    #rospy.Subscriber('mavros/state', State, cnt.stateCb)
    print "ici"
    # Subscribe to drone's local position
    rospy.Subscriber('/T265/odom/sample', Odometry, cnt.posCb)
    # rospy.Subscriber('/camera/odom/sample', Odometry, cnt.posCb)
    # Setpoint publisher    
    sp_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
    print "Topics: ok"

    # Make sure the drone is armed
    # c=0
    # while not cnt.state.armed:
    #     c+=1
    #     print "waiting arming"
    #     # modes.setArm()
    #     sleep(0.5)
    #     if c>20:
    #         exit(0)
    print "Armed"
    # set in takeoff mode and takeoff to default altitude (3 m)
    # modes.setTakeoff()
    # rate.sleep()

    # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect
    for k in range(20):
        sp_pub.publish(cnt.sp)
        rate.sleep()
    print "waiting offboard"
    # activate OFFBOARD mode
    # modes.setOffboardMode()
    print "Offboard mode activate"
    # ROS main loop
    cnt.change_alt(ALT)
    print "Starting ascenscion"
    #cross=[(0,0), (0,1), (0,0)]
    #path=[]
    # cross=[]
    cross=[[5,0],[0,0],[-5,0],[0,0],[0,5],[0,0],[0,-5],[0,0]]
    path=[[5,-5],[-5,-5],[-5,-2.5],[5,-2.5],[5,2.5],[-5,2.5],[-5,5],[5,5],[0,0]]
    for i in range(len(path)):
        path[i][0]*=5/5.0
        path[i][1]*=5/5.0
    targets=cross + path
    num_target=-1
    fini=False
    num_arrived=0
    cnt.is_arrived()
    sleep(15)
    while not rospy.is_shutdown():
    	# cnt.update()
        cnt.sp.header.stamp=rospy.Time.now()
    	sp_pub.publish(cnt.sp)

        if cnt.is_arrived():
            num_arrived+=1
            if not(fini):
                if num_arrived==NUMBER_OF_TIME_ON_POSE:
                    num_arrived=0
                    print cnt.sp.pose.position.x,cnt.sp.pose.position.y 
                    num_target+=1
                    if num_target<len(targets):
                        cnt.change_target(targets[num_target])
                    else:
                        sleep(15)
                        cnt.change_alt(0)
                        print "Atterissage"
                        fini=True
            else:
                #modes.setDisarm()
                pass #return 0  
        else:
            num_arrived=0           
    	rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
