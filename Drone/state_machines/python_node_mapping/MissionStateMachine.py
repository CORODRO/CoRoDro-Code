#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Jan  4 16:46:45 2021

@author: Jasmine Rimani
"""

import math
import time
import actionlib
import numpy as np
import rospy
import smach
import smach_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, PoseStamped, TwistStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from ar_track_alvar_msgs.msg import AlvarMarkers
from sensor_msgs.msg import Image, LaserScan, JointState
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
import subprocess
import os

# import pandas as pd

###
frequence = 10
ALT = rospy.get_param("altitude", 3)  # Altitude
EPSILON = [0.2, 0.2, 0.2]  # epsilon for position precision
NUMBER_OF_TIME_ON_POSE = 10  # nombre de fois qu'il doit etre "arrived" pour le croire (pour rappel voir frequence pour le temps)


###


class Controller:
    # initialization method
    def __init__(self):

        # Instantiate a setpoints message
        self.sp = PoseStamped()
        self.sp.header = Header()

        self.EPSILON = EPSILON

        self.ALT = ALT
        # ROS loop rate
        self.rate = rospy.Rate(frequence)

        self.number_of_time_on_pose = 0

        # Real position
        # msg = rospy.wait_for_message("T265/odom/sample", Odometry)
        self.real_zero_alt = 0  # msg.pose.pose.position.z - 0.05
        self.linear = Point(0, 0, self.real_zero_alt)
        self.angular = Point(0, 0, 0)

        # Topic
        self.sp_pub = rospy.Publisher("/SM/orders", PoseStamped, queue_size=1)
	self.sub =rospy.Subscriber('/T265/odom/sample', Odometry, self.posCb)


        # initial values for setpoints
        self.sp.pose.position.x = 0.0
        self.sp.pose.position.y = 0.0
        self.sp.pose.position.z = 0.0

        self.sp.pose.orientation.x = 0.0
        self.sp.pose.orientation.y = 0.0
        self.sp.pose.orientation.z = 0.0
        self.sp.pose.orientation.w = 1

    # Callbacks

    ## local position callback
    def posCb(self, msg):
        self.linear.x = msg.pose.pose.position.x
        self.linear.y = msg.pose.pose.position.y
        self.linear.z = msg.pose.pose.position.z
        self.angular.x = msg.pose.pose.orientation.x
        self.angular.y = msg.pose.pose.orientation.y
        self.angular.z = msg.pose.pose.orientation.z

    # self.angular.w = msg.pose.pose.orientation.w

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    def is_arrived_x(self):
        return abs(self.linear.x - self.sp.pose.position.x) <= self.EPSILON[0]

    def is_arrived_y(self):
        return abs(self.linear.y - self.sp.pose.position.y) <= self.EPSILON[1]

    def is_arrived_z(self):
        return abs(self.linear.z - self.sp.pose.position.z) <= self.EPSILON[2]

    def is_arrived(self):
        if self.number_of_time_on_pose >= NUMBER_OF_TIME_ON_POSE:
            return True
        elif self.is_arrived_x() and self.is_arrived_y() and self.is_arrived_z():
            self.number_of_time_on_pose += 1
        return False

    def change_target(self, couple):
        self.sp.pose.position.x = couple[0]
        self.sp.pose.position.y = couple[1]
        self.number_of_time_on_pose = 0  # Reinit because target is changing
        self.send_order()  # sending new target

    def change_alt(self, z):
        self.sp.pose.position.z = z
        self.number_of_time_on_pose = 0  # Reinit because target is changing
        self.send_order()  # sending new target

    def send_order(self):
        self.sp_pub.publish(self.sp)
        self.sp_pub.publish(self.sp)
        rospy.loginfo("New target sent: {} {} {}".format(self.sp.pose.position.x,
                                                         self.sp.pose.position.y, self.sp.pose.position.z))


# Start The State Machine
class InitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start'],
                             output_keys=['index', 'plan_out', 'tp_param', 'time_out_trying', 'failed_instance'])
        # index - at which step of the plan are we
        # plan_out - plan from the task planner
        # tp_param - parameters of the task planner - they define the end point of the different rover trajectories
        # time_out_trying - maximum time trying something before triggering exit condition

    def execute(self, userdata):
	global cnt
	cnt = Controller()

        # mdp initialitation index - start from zero --> initial position of the system
        userdata.index = 0
        # Read plan from txt
        # Define an empty list
        plan_out = []
        # take the path of the txt file that you need - because we like to make things super complex in ros
        basic_dir = rospy.get_param("~directory_with_plan", '/root/catkin_igluna_ws/src/state_machines/plan/')
        with open(basic_dir + 'plan_drone_24_06_2021.txt', 'r') as f:
            for item in f:
                currentLine = item
                # add item to list
                plan_out.append(currentLine)

        userdata.plan_out = plan_out
        # parameters of the task planner re-mapping dx,dy,resolution
        tp_param = [0] * 5
        tp_param[0] = float(rospy.get_param('~dx_map', 15))  # m
        tp_param[1] = float(rospy.get_param('~dy_map', 11))  # m
        tp_param[2] = float(rospy.get_param('~x_origin', -5.97)) # m
        tp_param[3] = float(rospy.get_param('~y_origin', -2.76))  # m
        tp_param[4] = float(rospy.get_param('~resolution_map', 1))  # m

        userdata.tp_param = tp_param
        # Time out trying for the art_tag reading
        time_out_trying = 0  # [s]
        userdata.time_out_trying = time_out_trying
        userdata.failed_instance = 0

        rospy.loginfo("arTag detect server")
        cmd=["roslaunch", "my_pcl_tutorial", "detect_tag.launch"]

        global arTagServer

        arTagServer = subprocess.Popen(cmd)

        time.sleep(3)
        return 'start'


# Identify initial position with Artag
#  For now it just subscribe to topic initial position - to update with the arTag code

class InitialLocalization(smach.State):
    def __init__(self):
        # go_to_the_next_instance - Estimate initial position and go to the next point
        # userdata - plan, index, task planning parameters, previous position to evaluate orientation of the goal pose
        smach.State.__init__(self, outcomes=['go_to_the_next_instance', 'aborted'])

    def execute(self, userdata):  # Userdata = plan_out
        # Wait 2s (so we can see we go through this state in the GUI)

        # controller object
        global cnt

        # Set Lift-off altitude
        cnt.change_alt(ALT)
        rospy.loginfo("TAKEOFF")
        # Publish desired position at Lift-off
        t = time.time()

        while not (cnt.is_arrived()):
            if time.time() - t > 900:
                rospy.logerr("Too much time to get to the altitude desired ( > 900s)")
                return 'aborted'
            cnt.rate.sleep()

	rospy.loginfo("Takeoff success")
        return 'go_to_the_next_instance'


# Where to go
class ChooseWP(smach.State):
    def __init__(self):
        # go_to --> Choose waypoint
        # follow  --> reach end of the simulation
        # follow_up --> keep_on with the navigation part
        # task-execution --> You have reached the point and you have stuff to do
        # index - at which step of the plan are we
        # plan_out - plan from the task planner
        # tp_param - parameters of the task planner - they define the end point of the different rover trajectories
        # time_out_trying - maximum time trying something before triggering exit condition
        # waypoint - x,y,orientation of the next waypoint
        # goal -

        smach.State.__init__(self, outcomes=['go_to', 'follow_up', 'task_execution', 'follow'],
                             input_keys=['index', 'plan_out', 'tp_param', 'prev_pos_vector'],
                             output_keys=['index', 'waypoint', 'goal', 'plan_out', 'tp_param', 'prev_pos_vector'])

    def execute(self, userdata):  # Userdata = plan_out
        if userdata.index < len(userdata.plan_out):
            instance = str(userdata.plan_out[userdata.index])[1:-2].split()[0]
            try:
                next_instance = str(userdata.plan_out[userdata.index + 1])[1:-2].split()[0]
            except:
                return 'follow' 

            # If navigate is followed by un unvisit - that is the last waypoint that we have to reach
            if instance == 'navigate' and next_instance == 'unvisit':
                rospy.loginfo("<Navigate to wp> before an <unvisit waypoint> = objective waypoint")
                objective_instance = str(userdata.plan_out[userdata.index + 2])[1:-1].split()[3]
                id_objective_instance = int(objective_instance[9::])

                wp = str(userdata.plan_out[userdata.index])[1:-2].split()[-1]
                print("----------------------------------------")
                rospy.loginfo("going to waypoint: %s", wp)
                print("----------------------------------------")
                # create a string with numeric index -->  a = 'waypoint'+str(index) --> gives --> 'waypoint3'
                # take out last number from waypoint to use it as index a[-1] --> 3
                wp_index = int(wp[8::])
                print("----------------------------------------")
                rospy.loginfo("waypoint index: %s", wp_index)
                print("----------------------------------------")

                # controller object
                global cnt

                maxi=0
                name="error"
                for file in os.listdir("/root/database"):
                    tempo=file.split("_")
                    if tempo[0]=="test" and tempo[1].isdigit() and int(tempo[1])>maxi:
                        maxi=int(tempo[1])
                        name=file

                if name=="error":
                    rospy.logerr("No folder name where found in /root/database/")
                else:
                    rospy.loginfo("Folder name: {}".format(name))


                with open("/root/database/"+name+"/dronedatabase.txt", 'r') as fstream:
                    for line in fstream:
                        if line != '':
                            line_cleaned = line.replace(',', '')
                            line_id = str(line_cleaned)[0:-1].split()[0]

                            print(id_objective_instance)
                            if str(line_id) == str(id_objective_instance):
                                x_position = str(line_cleaned)[0:-1].split()[2]
                                y_position = str(line_cleaned)[0:-1].split()[3]
                                print("----------------------------------------")
                                print("ArTag ID:")
                                print(line_id)
                                drone_position_x = float(x_position)
                                print("----------------------------------------")
                                print("x_position:",  x_position)
                                drone_position_y = float(y_position)
                                print("----------------------------------------")
                                print("y_position:", y_position)
                                print("----------------------------------------")
                        else:
                            rospy.loginfo('End of the file')

                # Remap from the index to the x-y position
                # drone_position_x, drone_position_y = ind2xy(wp_index,
                #                                             userdata.tp_param[0],
                #                                             userdata.tp_param[1],
                #                                             userdata.tp_param[2],
                #                                             userdata.tp_param[3])

                # Give new target position
                cnt.change_target([drone_position_x, drone_position_y])

                print("----------------------------------------")
                rospy.loginfo("coordinate waypoint: x:%s, y:%s ", drone_position_x, drone_position_y)
                print("----------------------------------------")

                userdata.waypoint = wp
                userdata.index = userdata.index + 2
                print("----------------------------------------")
                print("Userdata index")
                rospy.loginfo(userdata.index)
                print("----------------------------------------")
                userdata.plan_out = userdata.plan_out

                while not cnt.is_arrived():
                    cnt.rate.sleep()

                return 'go_to'


            elif instance == 'navigate' and next_instance == 'visit':
                rospy.loginfo("<Navigate to wp> before an <visit waypoint> = not yet reached objective waypoint")
                wp = str(userdata.plan_out[userdata.index])[1:-2].split()[-1]
                userdata.waypoint = wp
                userdata.index = userdata.index + 1
                print("Userdata index")
                rospy.loginfo(userdata.index)
                print("----------------------------------------")
                rospy.loginfo('passing by: %s', wp)
                return 'follow_up'

            elif str(userdata.plan_out[userdata.index])[1:-2].split()[0] == 'visit':
                system = str(userdata.plan_out[1])[1:-1].split()[-1]
                wp = str(userdata.plan_out[userdata.index])[1:-2].split()[-1]
                # The system is now at that waypoint
                rospy.loginfo("the %s is at a waypoint %s", system, wp)
                userdata.waypoint = wp
                userdata.index = userdata.index + 1
                print("Userdata index")
                rospy.loginfo(userdata.index)
                print("----------------------------------------")
                userdata.plan_out = userdata.plan_out

                return 'follow_up'

            elif str(userdata.plan_out[userdata.index])[1:-2].split()[0] == 'unvisit':
                system = str(userdata.plan_out[1])[1:-1].split()[-1]
                wp = str(userdata.plan_out[userdata.index])[1:-2].split()[-1]
                userdata.waypoint = wp
                userdata.index = userdata.index + 1
                print("Userdata index")
                rospy.loginfo(userdata.index)
                print("----------------------------------------")
                userdata.plan_out = userdata.plan_out
                rospy.loginfo("the %s is leaving waypoint %s", system, wp)
                return 'follow_up'

            else:
                return 'task_execution'

        else:
            return 'follow'


class ExecuteTask(smach.State):
    def __init__(self):
        # go_to --> Choose waypoint
        # follow --> reach end of the simulation
        # task-execution --> You have reached the point and you have stuff to do
        # userdata - plan and index
        smach.State.__init__(self, outcomes=['follow', 'read_arTag', 'communicate_arTag_data', 'take_image',
                                             'communicate_image_data', 'get_data_from_sensors',
                                             'send_system_state'], input_keys=['index', 'plan_out'])

    def execute(self, userdata):  # Userdata = plan_out
        if userdata.index < len(userdata.plan_out):
            rospy.loginfo(str(userdata.plan_out[userdata.index])[1:-1].split()[0])
            rospy.loginfo(str(userdata.plan_out[userdata.index])[1:-1].split())
            if str(userdata.plan_out[userdata.index])[1:-1].split()[0] == 'read_arTag':
                rospy.loginfo("Reading arTag Data")
                return 'read_arTag'
            elif str(userdata.plan_out[userdata.index])[1:-1].split()[0] == 'communicate_arTag_data':
                rospy.loginfo("Communicate arTag Data")
                return 'communicate_arTag_data'
            elif str(userdata.plan_out[userdata.index])[1:-1].split()[0] == 'take_image':
                rospy.loginfo("Take Image")
                return 'take_image'
            elif str(userdata.plan_out[userdata.index])[1:-1].split()[0] == 'communicate_image_data':
                rospy.loginfo("Communicate Image Data")
                return 'communicate_image_data'
            elif str(userdata.plan_out[userdata.index])[1:-1].split()[0] == 'get_data_from_sensors':
                rospy.loginfo("get_data_from_sensors")
                return 'get_data_from_sensors'
            elif str(userdata.plan_out[userdata.index])[1:-1].split()[0] == 'send_system_state':
                rospy.loginfo("Send System State")
                return 'send_system_state'
            else:
                rospy.logerr("UNKOWN STATE")
                return 'follow'
        else:
            return 'follow'


class ReadArTag(smach.State):
    def __init__(self):
        # go_to_the_next_instance - Estimate initial position and go to the next point
        # userdata - plan, index, task planning parameters, previous position to evaluate orientation of the goal pose
        smach.State.__init__(self, outcomes=['arTag_read', 'waiting_for_arTag', 'no_arTag_detected'],
                             input_keys=['index', 'goal', 'time_out_trying', 'failed_instance'],
                             output_keys=['index', 'goal', 'time_out_trying', 'no_artag', 'failed_instance'])

    def execute(self, userdata):
        # arTag message --- add callback function for arTag when the code is ready!!!!!!!!
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.arTag_callback_fun, queue_size=1)
        rospy.sleep(1)
        # if we cannot find the Artag we rotate on ourself we rotate for 360?
        # or do we wait

        # If we do not read it try turn a bit the system in the opposite direction that it is it facing
        try:
            print("----------------------------------------")
            print('artag read')
            rospy.loginfo(self.arTag_callback.markers[0])
            print("----------------------------------------")
            # reset the time-out!!!
            time_out_trying = 0
            userdata.time_out_trying = time_out_trying
            # Go to the next instance
            userdata.index = userdata.index + 1
            # Artag detected!!!!
            userdata.no_artag = 0
            return 'arTag_read'

        except:

            print("----------------------------------------")
            print('we are waiting to read artag')
            print("----------------------------------------")
            time_out_trying = userdata.time_out_trying + 1
            userdata.time_out_trying = time_out_trying
            # maybe we will detect the arTag
            userdata.no_artag = 0

            if time_out_trying > 3:
                print("----------------------------------------")
                print('artag abort')
                rospy.loginfo(self.arTag_callback)
                print("----------------------------------------")
                # Abort mission? or take a photo and move to the next instance?
                time_out_trying = 0
                userdata.time_out_trying = time_out_trying
                # Go on with the plan
                userdata.index = userdata.index + 1
                # no artag detected - just take a photo and move on with the mission
                userdata.no_artag = 0
                # But record that you failed to send the message
                userdata.failed_instance = userdata.failed_instance + 1
                # return 'no_arTag_detected'
                rospy.logerr("ERROR WHILE READING ARTAG")
                return 'arTag_read'
            else:
                return 'waiting_for_arTag'

    def arTag_callback_fun(self, msg):
        self.arTag_callback = msg


class Communicate_arTag(smach.State):
    def __init__(self):
        # go_to_the_next_instance - Estimate initial position and go to the next point
        # userdata - plan, index, task planning parameters, previous position to evaluate orientation of the goal pose
        smach.State.__init__(self, outcomes=['arTag_data_sent', 'no_connection'],
                             input_keys=['index', 'failed_instance', 'no_artag'],
                             output_keys=['index', 'failed_instance', 'time_out_trying'])

    def execute(self, userdata):
        # # arTag message --- add callback function for arTag when the code is ready!!!!!!!!
        # rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.arTag_callback_fun, queue_size=1)
        # rospy.sleep(3)
        #
        # # If we read the artag we go to the next instance
        # rospy.sleep(3)

        if userdata.no_artag == 1:
            # just go to the next instance - it is a failed game
            userdata.failed_instance = userdata.failed_instance + 1
            return 'no_connection'

        else:
            time_out_trying = 0
            userdata.time_out_trying = time_out_trying

            print("----------------------------------------")
            rospy.loginfo("Artag Saved on file in /root/database!")
            print("----------------------------------------")
            # rospy.loginfo(self.arTag_callback)
            # rospy.loginfo("------------------")

            # self.arTag_callback.markers[0].id self.arTag_callback.markers[i].id

            # save the detected artag to file ... part of the code of Maximilien
            # with open('/root/database/read_arTag_list.txt', 'w') as f:
            # f.write("id:"+ self.arTag_callback.markers[0].id + ", x:" + self.arTag_callback.markers[0].pose.pose.point.x + ", y:" + self.arTag_callback.markers[0].pose.pose.point.y)

            # Advance the plan
            userdata.index = userdata.index + 1
            return 'arTag_data_sent'

    # def arTag_callback_fun(self, msg):
    #     self.arTag_callback = msg


class TakeImage(smach.State):
    def __init__(self):
        # go_to_the_next_instance - Estimate initial position and go to the next point
        # userdata - plan, index, task planning parameters, previous position to evaluate orientation of the goal pose
        smach.State.__init__(self, outcomes=['Image_read', 'no_Image_taken'],
                             input_keys=['index', 'failed_instance'],
                             output_keys=['index', 'image_message', 'failed_instance'])

    def execute(self, userdata):  # Userdata = plan_out
        # Subscribe to the image of the depth camera - check if a message is published
        rospy.Subscriber("/D435/color/image_raw", Image, self.image_callback, queue_size=1)
        # wait for the message to be read
        rospy.sleep(1)

        try:
            # take picture
            rospy.loginfo("message received: %s", self.message_received)
            userdata.image_message = self.message_received
            cmd = ["rosrun", "image_view", "image_saver", "image:=/D435/color/image_raw"]
            image_taking = subprocess.Popen(cmd)
            # wait for the image to be saved - or use https://gist.github.com/rethink-imcmahon/77a1a4d5506258f3dc1f#file-ros_image_saver-py
            # rospy.sleep(3)
            print("----------------------------------------")
            rospy.loginfo("got image")
            print("----------------------------------------")
            rospy.sleep(1.5)
            image_taking.terminate()
            # Advance the plan
            userdata.index = userdata.index + 1
            return 'Image_read'

        except NameError:
            print("check if message is published")
            print("aborted")
            self.message_received = 'no'
            userdata.failed_instance = userdata.failed_instance + 1
            return 'no_Image_taken'

    def image_callback(self, msg):
        self.image_msg = msg
        self.message_received = 'yes'


class Communicate_Image(smach.State):
    def __init__(self):
        # go_to_the_next_instance - Estimate initial position and go to the next point
        # userdata - plan, index, task planning parameters, previous position to evaluate orientation of the goal pose
        smach.State.__init__(self, outcomes=['Image_sent', 'no_connection'],
                             input_keys=['index', 'image_message', 'failed_instance'],
                             output_keys=['index', 'failed_instance'])

    def execute(self, userdata):
        if userdata.image_message == 'yes':
            userdata.index = userdata.index + 1
            print("----------------------------------------")
            print("Image sent")
            print("----------------------------------------")
            return 'Image_sent'

        else:
            print("----------------------------------------")
            print("no image to send")
            print("----------------------------------------")
            userdata.failed_instance = userdata.failed_instance + 1
            return 'no_connection'


class GetDataFromSensors(smach.State):
    def __init__(self):
        # go_to_the_next_instance - Estimate initial position and go to the next point
        # userdata - plan, index, task planning parameters, previous position to evaluate orientation of the goal pose
        smach.State.__init__(self, outcomes=['state_monitored', 're-evaluate plan', 'no_state'],
                             input_keys=['index', 'failed_instance'],
                             output_keys=['index', 'failed_instance', 'image_message', 'laser_message', 'T265_message',
                                          'joint_state_message', 'got_sensor_data'])

    def execute(self, userdata):
        # Initialize the variable to check
        self.image_message_received = 'no'
        self.laser_message_received = 'no'
        self.T265_callback = 'no'
        self.joint_state_message_received = 'no'

        # Subscribe to the image of the depth camera - check if a message is published
        rospy.Subscriber("/D435/color/image_raw", Image, self.image_callback, queue_size=1)
        # Subscribe to the T265_scan
        rospy.Subscriber("/T265/odom/sample", Odometry, self.T265_callback, queue_size=1)
        # wait for the messages to be read
        rospy.sleep(1.5)
        #  here to add the effective action to take if we don't have any of this messages
        # still to finish coding!!!!!!!

        if self.image_message_received == 'yes':
            rospy.loginfo('Depth camera working')
            userdata.image_message = self.image_message
        else:
            rospy.loginfo('No Depth camera')
            # remove depth camera instance for the rover in the domain file

        if self.T265_callback == 'yes':
            rospy.loginfo('Tracking camera working')
            userdata.T265_message = self.T265_message
        else:
            rospy.loginfo('No T265 camera')
            # remove T265 instance for the rover in the domain file


        # Check if the topic you need are published or not
        #  Still to code
        # Subscribe to battery, tracking camera, depth camera, joint state and Lidar

        #if self.image_message_received == 'yes' and self.T265_callback == 'yes' and self.joint_state_message_received == 'yes':
            # Go to the next instance
        #    userdata.index = userdata.index + 1
        #    userdata.got_sensor_data = 1
        #    return 'state_monitored'
        #elif self.image_message_received == 'no' or self.T265_callback == 'no' or self.joint_state_message_received == 'no':
            # Go to the next instance
        #    userdata.index = userdata.index + 1
        #    return 're-evaluate plan'
        #else:
        #    userdata.failed_instance = userdata.failed_instance + 1
        #    return 'no_state'
        return 'state_monitored'

    def image_callback(self, msg):
        self.image_message = msg
        self.image_message_received = 'yes'

    def T265_callback(self, msg):
        self.T265_message = msg
        self.T265_message_received = 'yes'


class Send_system_state(smach.State):
    def __init__(self):
        # go_to_the_next_instance - Estimate initial position and go to the next point
        # userdata - plan, index, task planning parameters, previous position to evaluate orientation of the goal pose
        smach.State.__init__(self, outcomes=['System_state_sent', 'no_connection'],
                             input_keys=['index', 'failed_instance', 'image_message', 'laser_message', 'T265_message',
                                         'joint_state_message', 'got_sensor_data'],
                             output_keys=['index', 'failed_instance'])

    def execute(self, userdata):  # Userdata = plan_out
        # Still to finish coding - for now print messages

        if userdata.got_sensor_data == 1:
            userdata.index = userdata.index + 1
            rospy.loginfo('--- Depth Camera ---')
            rospy.loginfo(userdata.image_message)
            rospy.loginfo('--- ------------ ---')
            rospy.loginfo('--- Tracking Camera ---')
            rospy.loginfo(userdata.T265_message)
            rospy.loginfo('--- ------------ ---')
            rospy.loginfo('--- Joint State ---')
            rospy.loginfo(userdata.joint_state_message)
            rospy.loginfo('--- ------------ ---')
            return 'System_state_sent'
        else:
            userdata.failed_instance = userdata.failed_instance + 1
            return 'no_connection'


if __name__ == '__main__':
    # State Machine Visualize the steps of the plan
    rospy.init_node('smach_rover_state_machine')

    # Rate
    rate = rospy.get_param("~rate", 1)

    ''' All paramaters used in the State Machines '''

    time_up = rospy.get_param("~max_simulation_time", 100 * 60)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    with sm:
        smach.StateMachine.add('INIT', InitState(),
                               transitions={'start': 'INITIAL_LOCALIZATION'})

        smach.StateMachine.add('INITIAL_LOCALIZATION', InitialLocalization(),
                               transitions={'go_to_the_next_instance': 'CHOOSE_ACTION',
                                            'aborted': 'aborted'},
                               remapping={})

        smach.StateMachine.add('CHOOSE_ACTION', ChooseWP(),
                               transitions={'go_to': 'EXECUTE_TASK',
                                            'follow_up': 'CHOOSE_ACTION',
                                            'task_execution': 'EXECUTE_TASK',
                                            'follow': 'succeeded'},
                               remapping={})

        smach.StateMachine.add('EXECUTE_TASK', ExecuteTask(),
                               transitions={'follow': 'succeeded',
                                            'read_arTag': 'READ_ARTAG',
                                            'communicate_arTag_data': 'COMMUNICATE_ARTAG',
                                            'take_image': 'TAKE_IMAGE',
                                            'communicate_image_data': 'COMMUNICATE_IMAGE',
                                            'get_data_from_sensors': 'GET_DATA_FROM_SENSORS',
                                            'send_system_state': 'SEND_SYSTEM_STATE'})

        smach.StateMachine.add('READ_ARTAG', ReadArTag(),
                               transitions={'arTag_read': 'CHOOSE_ACTION',
                                            'waiting_for_arTag': 'READ_ARTAG',
                                            'no_arTag_detected': 'aborted'},
                               remapping={})

        smach.StateMachine.add('COMMUNICATE_ARTAG', Communicate_arTag(),
                               transitions={'arTag_data_sent': 'CHOOSE_ACTION',
                                            'no_connection': 'aborted'},
                               remapping={})

        smach.StateMachine.add('TAKE_IMAGE', TakeImage(),
                               transitions={'Image_read': 'CHOOSE_ACTION',
                                            'no_Image_taken': 'aborted'},
                               remapping={})

        smach.StateMachine.add('COMMUNICATE_IMAGE', Communicate_Image(),
                               transitions={'Image_sent': 'CHOOSE_ACTION',
                                            'no_connection': 'aborted'},
                               remapping={})

        smach.StateMachine.add('GET_DATA_FROM_SENSORS', GetDataFromSensors(),
                               transitions={'state_monitored': 'CHOOSE_ACTION',
                                            're-evaluate plan': 'preempted',
                                            'no_state': 'aborted'},
                               remapping={})

        smach.StateMachine.add('SEND_SYSTEM_STATE', Send_system_state(),
                               transitions={'System_state_sent': 'CHOOSE_ACTION',
                                            'no_connection': 'aborted'},
                               remapping={})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.loginfo("Starting landing actions")

    global cnt

    t = time.time()
    cnt.change_target([0, 0])
    while not (cnt.is_arrived()):
        if time.time() - t > 15:
            rospy.logerr("Timeout while trying to go back in 0 0 before landing ( > 15s )")
            break
        cnt.rate.sleep()

    t = time.time()
    cnt.change_alt(0)
    while not (cnt.is_arrived()):
        if time.time() - t > 10:
            rospy.logerr("Timeout while trying to land ( > 10s )")
            break
        cnt.rate.sleep()

    global arTagServer

    arTagServer.terminate()

    rospy.sleep(1)

    sis.stop()

    rospy.loginfo("Drone landed")

    rospy.signal_shutdown('All done.')
