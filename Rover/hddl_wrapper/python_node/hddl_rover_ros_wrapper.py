#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Jan  4 16:46:45 2021

@author: Jasmine Rimani
"""

import math
import time
import actionlib
from mux import ActivateWP, ActivateStop, ActivateFollow
from util import ind2xy, euler_to_quaternion, quaternion_to_euler
# Clarification -  mux: multiplex between multiple topics
# utils.mux relays on pkg="topic_tools"
# import mdp_robot.modified_movement_model as model
import numpy as np
import rospy
import smach
import smach_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from ar_track_alvar_msgs.msg import AlvarMarkers
from sensor_msgs.msg import Image, LaserScan, JointState
from nav_msgs.msg import Odometry
import subprocess
import os
# import pandas as pd

# Start The State Machine
class InitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start'], output_keys=['index', 'plan_out', 'tp_param', 'time_out_trying', 'failed_instance'])
        # index - at which step of the plan are we
        # plan_out - plan from the task planner
        # tp_param - parameters of the task planner - they define the end point of the different rover trajectories
        # time_out_trying - maximum time trying something before triggering exit condition

    def execute(self, userdata):
        # mdp initialitation index - start from zero --> initial position of the system
        userdata.index = 0
        # Read plan from txt
        # Define an empty list
        plan_out = []
        # take the path of the txt file that you need - because we like to make things super complex in ros
        basic_dir = rospy.get_param("~directory_with_plan", '/root/igluna_ws/src/hddl_wrapper/plan/')
        with open(basic_dir + 'plan_friday.txt', 'r') as f:
            for item in f:
                currentLine = item
                # add item to list
                plan_out.append(currentLine)

        userdata.plan_out = plan_out
        # parameters of the task planner re-mapping dx,dy,resolution
        tp_param = [0, 0, 0, 0, 0]
        dx_map = rospy.get_param('~dx_map', 8) # m
        print(dx_map)
        dy_map= rospy.get_param('~dy_map', 11) # m
        x_origin = rospy.get_param('~x_origin', -5.97) # m
        y_origin= rospy.get_param('~y_origin', -2.76) # m
        resolution = rospy.get_param('~resolution_map', 1)  # m

        tp_param = [float(dx_map), float(dy_map), float(x_origin), float(y_origin), float(resolution)]

        userdata.tp_param = tp_param

         # Time out trying for the art_tag reading
        time_out_trying = 0  # [s]
        userdata.time_out_trying = time_out_trying
        userdata.failed_instance = 0

        return 'start'


# Identify initial position with Artag
#  For now it just subscribe to topic initial position - to update with the arTag code

class InitialLocalization(smach.State):
    def __init__(self):
        # go_to_the_next_instance - Estimate initial position and go to the next point
        # userdata - plan, index, task planning parameters, previous position to evaluate orientation of the goal pose
        smach.State.__init__(self, outcomes=['go_to_the_next_instance', 'aborted'],
                             output_keys=['prev_pos_vector'])

    def execute(self, userdata):  # Userdata = plan_out
        # Subscribe to amcl initial pose - given from rviz - this code will change soon
        rospy.Subscriber("/T265/odom/sample", Odometry, self.position_callback)
        time.sleep(2)

        if self.read_message == 'yes':
            rospy.loginfo("initial pose acquired")
            rospy.loginfo(self.position_message)
            initial_x = self.position_message.pose.pose.position.x
            # rospy.loginfo(self.position_message.pose.pose.position.x)
            initial_y = self.position_message.pose.pose.position.y
            prev_pos_vector = [initial_x, initial_y]
            rospy.loginfo('initial position, x: %s, y: %s', initial_x, initial_y)
            userdata.prev_pos_vector = prev_pos_vector
            return 'go_to_the_next_instance'
        else:
            print("check if message is published")
            print("aborted")
            return 'aborted'

    def position_callback(self, msg):
        self.position_message = msg
        self.read_message = 'yes'


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
                             output_keys=['index', 'waypoint', 'goal', 'plan_out', 'tp_param', 'prev_pos_vector', 'artag_index'])

    def execute(self, userdata):  # Userdata = plan_out
        # Wait 2s (so we can see we go through this state in the GUI)
        time.sleep(1)

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
                id_objective_instance = int(objective_instance[-1])
                goal = MoveBaseGoal()
                # geometry_msgs/PoseStamped target_pose
                #      std_msgs/Header header
                #      geometry_msgs/Pose pose
                #          geometry_msgs/Point position
                #          geometry_msgs/Quaternion orientation
                with open("/root/database/test_27_2021-06-25_10-02-29/dronedatabase.txt", 'r') as fstream:
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
                                userdata.artag_index = line_id
                                goal.target_pose.pose.position.x = float(x_position)
                                print("----------------------------------------")
                                print("x_position: %s", x_position)
                                goal.target_pose.pose.position.y = float(y_position)
                                print("----------------------------------------")
                                print("y_position: %s", y_position)
                                print("----------------------------------------")
                        else:
                            rospy.loginfo('End of the file')


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
                # Remap from the index to the x-y position
                # goal.target_pose.pose.position.x, goal.target_pose.pose.position.y = ind2xy(wp_index,
                #                                                                            userdata.tp_param[0],
                #                                                                            userdata.tp_param[1],
                #                                                                            userdata.tp_param[2],
                #                                                                            userdata.tp_param[3],
                #                                                                            userdata.tp_param[4])
                # Define the angle between final position and initial position
                prev_wp = str(userdata.plan_out[userdata.index])[1:-2].split()[-2]
                prev_wp_index = int(prev_wp[-1])
                print("----------------------------------------")
                rospy.loginfo("leaving waypoint: %s", prev_wp)
                print("----------------------------------------")

                prev_wp_x, prev_wp_y = userdata.prev_pos_vector[0], userdata.prev_pos_vector[1]
                # give new starting position
                userdata.prev_pos_vector[0] = goal.target_pose.pose.position.x
                userdata.prev_pos_vector[1] = goal.target_pose.pose.position.y

                # # Which waypoint am I leaving
                # rospy.loginfo("leaving waypoint coordinate: x:%s, y:%s ", prev_wp_x,
                #               prev_wp_y)
                # # Evaluate orientation of the body
                angle_z = math.atan2((goal.target_pose.pose.position.y - prev_wp_y),
                                     (goal.target_pose.pose.position.x - prev_wp_x))
                quaternion = euler_to_quaternion(0, 0, angle_z)
                rospy.loginfo("orientation around z-axis: %s", angle_z*180/np.pi)
                # Define the quaternion
                goal.target_pose.pose.orientation.x = quaternion[0]
                goal.target_pose.pose.orientation.y = quaternion[1]
                goal.target_pose.pose.orientation.z = quaternion[2]
                goal.target_pose.pose.orientation.w = quaternion[3]

                # Define the boundaries of 50 cm on the position
                # Take a point around 0.5m of distance from the tag
                u_abs_x = abs(goal.target_pose.pose.position.x-prev_wp_x)
                u_prop_x = (0.50)/u_abs_x

                if u_prop_x < 1:
                    goal.target_pose.pose.position.x = prev_wp_x * u_prop_x + (1-u_prop_x) * goal.target_pose.pose.position.x

                if u_prop_x > 1:
                    goal.target_pose.pose.position.x = prev_wp_x * (0.9) + 0.1* goal.target_pose.pose.position.x

                u_abs_y = abs(goal.target_pose.pose.position.y-prev_wp_y)
                u_prop_y = (0.50)/u_abs_y

                if u_prop_y < 1:
                    goal.target_pose.pose.position.y = prev_wp_y * u_prop_y + (1-u_prop_y) * goal.target_pose.pose.position.y

                if u_prop_y > 1:
                    goal.target_pose.pose.position.y = prev_wp_y * (0.9) + 0.1* goal.target_pose.pose.position.y


                print("----------------------------------------")
                rospy.loginfo("coordinate waypoint: x:%s, y:%s ", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
                print("----------------------------------------")

                # Define the frame id
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.Time.now()
                print("----------------------------------------")
                rospy.loginfo('Goal message')
                rospy.loginfo(goal)
                print("----------------------------------------")

                # userdata assign
                userdata.goal = goal
                userdata.waypoint = wp

                userdata.index = userdata.index + 1
                print("----------------------------------------")
                print("Userdata index")
                rospy.loginfo(userdata.index)
                print("----------------------------------------")
                userdata.plan_out = userdata.plan_out
                rospy.loginfo("going to waypoint: %s", wp)
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


# how to Move
# Use MoveBase Action to go
class MoveToWP(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found_path', 'waiting', 'aborted'],
                             input_keys=['goal', 'index'],
                             output_keys=['index'])

    def execute(self, userdata):
        rospy.loginfo("going to waypoint: ")
        rospy.loginfo(userdata.goal)
        rospy.loginfo("------------------")

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        print("server on")
        rospy.sleep(2)

#        ActivateWP()

        client.send_goal(userdata.goal)
        print("goal sent")
        wait = client.wait_for_result()
        print("Result Acquired")

        if not wait:
            rospy.loginfo("Action server not available!")
            return 'waiting'

        status = client.get_state()

        if status == 3:
            print("we are in status: %s", status)
            print("--------------------------------")
            return 'found_path'
        elif status == 1:
            print("we are in status: %s", status)
            print("--------------------------------")
            return 'waiting'
        else:
            print("we are in status: %s", status)
            print("--------------------------------")
            return 'aborted'

        #        ActivateStop()

        outcome = smach_ros.SimpleActionState.execute(self, userdata)
        rospy.loginfo("Outcome move base: %s", outcome)
        rospy.loginfo("------------------")
        # ActivateStop()
        userdata.index = userdata.index+1
        print("Userdata index", userdata.index)
        print("----------------------------------------")

        return outcome


class ExecuteTask(smach.State):
    def __init__(self):
        # go_to --> Choose waypoint
        # follow --> reach end of the simulation
        # task-execution --> You have reached the point and you have stuff to do
        # userdata - plan and index
        smach.State.__init__(self, outcomes=['follow', 'read_arTag', 'communicate_arTag_data', 'take_image',
                                             'communicate_image_data', 'make_available', 'get_data_from_sensors',
                                             'send_system_state'], input_keys=['index', 'plan_out'])

    def execute(self, userdata):  # Userdata = plan_out

        time.sleep(1)

        if userdata.index < len(userdata.plan_out):
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
            elif str(userdata.plan_out[userdata.index])[1:-1].split()[0] == 'make_available':
                rospy.loginfo("Make Available Second System")
                return 'make_available'
            elif str(userdata.plan_out[userdata.index])[1:-1].split()[0] == 'get_data_from_sensors':
                rospy.loginfo("get_data_from_sensors")
                return 'get_data_from_sensors'
            elif str(userdata.plan_out[userdata.index])[1:-1].split()[0] == 'send_system_state':
                rospy.loginfo("Send System State")
                return 'send_system_state'
        else:
            return 'follow'


class ReadArTag(smach.State):
    def __init__(self):
        # go_to_the_next_instance - Estimate initial position and go to the next point
        # userdata - plan, index, task planning parameters, previous position to evaluate orientation of the goal pose
        smach.State.__init__(self, outcomes=['arTag_read', 'waiting_for_arTag', 'no_arTag_detected'],
                             input_keys=['index', 'goal', 'time_out_trying', 'failed_instance','artag_index'],
                             output_keys=['index', 'goal', 'time_out_trying', 'no_artag','failed_instance'])

    def execute(self, userdata):

        # arTag message --- add callback function for arTag when the code is ready!!!!!!!!
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.arTag_callback_fun, queue_size=1)
        rospy.sleep(2)

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
            # Evaluate orientation of the body
            x = userdata.goal.target_pose.pose.orientation.x
            y = userdata.goal.target_pose.pose.orientation.y
            z = userdata.goal.target_pose.pose.orientation.z
            w = userdata.goal.target_pose.pose.orientation.w
            # go back to euler angle
            angle_x, angle_y, angle_z = quaternion_to_euler(x, y, z, w)
            # add 10° to the goal orientation
            angle_z = angle_z + (30*np.pi/180)
            quaternion = euler_to_quaternion(0, 0, angle_z)
            rospy.loginfo("orientation around z-axis: %s", angle_z * 180 / np.pi)
            # Define the new pose quaternion
            userdata.goal.target_pose.pose.orientation.x = quaternion[0]
            userdata.goal.target_pose.pose.orientation.y = quaternion[1]
            userdata.goal.target_pose.pose.orientation.z = quaternion[2]
            userdata.goal.target_pose.pose.orientation.w = quaternion[3]
            # maybe we will detect the arTag
            userdata.no_artag = 0

            if time_out_trying > 10:
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
                userdata.no_artag = 1
                # But record that you failed to send the message
                userdata.failed_instance = userdata.failed_instance + 1
                return 'no_arTag_detected'
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
        rospy.sleep(2)
        # Wait 2s (so we can see we go through this state in the GUI)
        # rospy.sleep(2)
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
            rospy.sleep(3)

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
        # Wait 2s (so we can see we go through this state in the GUI)
        time.sleep(2)
        # Subscribe to the image of the depth camera - check if a message is published
        rospy.Subscriber("/D435/color/image_raw", Image, self.image_callback, queue_size=1)
        # wait for the message to be read
        time.sleep(2)

        try:
            # take picture
            rospy.loginfo("message received: %s", self.message_received)
            userdata.image_message = self.message_received
            rospy.sleep(2)
            cmd = ["rosrun", "image_view", "image_saver", "image:=/D435/color/image_raw"]
            image_taking = subprocess.Popen(cmd)
            # wait for the image to be saved - or use https://gist.github.com/rethink-imcmahon/77a1a4d5506258f3dc1f#file-ros_image_saver-py
            # rospy.sleep(3)
            print("----------------------------------------")
            rospy.loginfo("got image")
            print("----------------------------------------")
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
        # Wait 2s (so we can see we go through this state in the GUI)
        rospy.sleep(2)

        if userdata.image_message == 'yes':
            rospy.sleep(3)
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


class make_available(smach.State):
    def __init__(self):
        # go_to_the_next_instance - Estimate initial position and go to the next point
        # userdata - plan, index, task planning parameters, previous position to evaluate orientation of the goal pose
        smach.State.__init__(self, outcomes=['Second_system_available', 'Second_system_not_available'],
                             input_keys=['index', 'failed_instance'],
                             output_keys=['index', 'failed_instance'])

    def execute(self, userdata):  # Userdata = plan_out
        # Wait 2s (so we can see we go through this state in the GUI)
        time.sleep(2)
        # Ping the drone to see if it is up!
        # hostname = ''
        # response = os.system('ping -c 1 ' + hostname)
        response = 0  # not tested part of the code - for now just accept that the system is available.
        time.sleep(2)

        if response == 0:
            userdata.index = userdata.index + 1
            print(hostname, 'is up')
            return 'Second_system_available'
        else:
            print(hostname, 'is down')
            userdata.failed_instance = userdata.failed_instance + 1
            return 'Second_system_not_available'

class GetDataFromSensors(smach.State):
    def __init__(self):
        # go_to_the_next_instance - Estimate initial position and go to the next point
        # userdata - plan, index, task planning parameters, previous position to evaluate orientation of the goal pose
        smach.State.__init__(self, outcomes=['state_monitored', 're-evaluate plan', 'no_state'],
                             input_keys=['index', 'failed_instance'],
                             output_keys=['index', 'failed_instance', 'image_message','laser_message', 'T265_message', 'joint_state_message', 'got_sensor_data'])

    def execute(self, userdata):

        time.sleep(2)
        # Initialize the variable to check
        self.image_message_received = 'no'
        self.laser_message_received = 'no'
        self.T265_callback = 'no'
        self.joint_state_message_received = 'no'

        # Subscribe to the image of the depth camera - check if a message is published
        rospy.Subscriber("/D435/color/image_raw", Image, self.image_callback, queue_size=1)
        # wait for the message to be read
        time.sleep(1)
        # Subscribe to the laser_scan
        rospy.Subscriber("/scan", LaserScan, self.laser_callback, queue_size=1)
        # wait for the message to be read
        time.sleep(1)
        # Subscribe to the T265_scan
        rospy.Subscriber("/T265/odom/sample", Odometry, self.T265_callback, queue_size=1)
        # wait for the message to be read
        time.sleep(1)
        # Subscribe to joint state
        rospy.Subscriber("/joint_state", JointState, self.joint_state_callback, queue_size=1)
        # wait for the message to be read
        time.sleep(1)
        #  here to add the effective action to take if we don't have any of this messages
        # still to finish coding!!!!!!!

        if self.image_message_received == 'yes':
            rospy.loginfo('Depth camera working')
            userdata.image_message = self.image_message
        else:
            rospy.loginfo('No Depth camera')
            # remove depth camera instance for the rover in the domain file

        if self.laser_message_received == 'yes':
            rospy.loginfo('LiDAR working')
            userdata.laser_message = self.laser_message
        else:
            rospy.loginfo('No LiDAR')
            # remove LiDAR instance for the rover in the domain file

        if self.T265_callback == 'yes':
            rospy.loginfo('Tracking camera working')
            userdata.T265_message = self.T265_message
        else:
            rospy.loginfo('No T265 camera')
            # remove T265 instance for the rover in the domain file

        if self.joint_state_message_received == 'yes':
            rospy.loginfo('Joint State working')
            userdata.joint_state_message= self.joint_state_message
        else:
            rospy.loginfo('Joint State no received')
            # remove Joint State instance for the rover in the domain file

        # Check if the topic you need are published or not
        #  Still to code
        # Subscribe to battery, tracking camera, depth camera, joint state and Lidar
        # just say that you have all the message for the first dry run - still to code the remove the sensors from the plan
        all_message_received = 0

        if all_message_received == 0:
        # if self.image_message_received == 'yes' and self.laser_message_received == 'yes' and self.T265_callback == 'yes' and self.joint_state_message_received == 'yes':
            # Go to the next instance
            userdata.index = userdata.index + 1
            userdata.got_sensor_data = 1
            return 'state_monitored'
        elif self.image_message_received == 'no' and self.laser_message_received == 'no' and self.T265_callback == 'no' and self.joint_state_message_received == 'no':
            # Go to the next instance
            userdata.index = userdata.index + 1
            return 're-evaluate plan'
        else:
            userdata.failed_instance = userdata.failed_instance + 1
            return 'no_state'


    def image_callback(self, msg):
            self.image_message = msg
            self.image_message_received = 'yes'

    def laser_callback(self, msg):
            self.laser_message = msg
            self.laser_message_received = 'yes'

    def T265_callback(self, msg):
            self.T265_message = msg
            self.T265_message_received = 'yes'

    def joint_state_callback(self, msg):
            self.joint_state_message = msg
            self.joint_state_message_received = 'yes'


class Send_system_state(smach.State):
    def __init__(self):
        # go_to_the_next_instance - Estimate initial position and go to the next point
        # userdata - plan, index, task planning parameters, previous position to evaluate orientation of the goal pose
        smach.State.__init__(self, outcomes=['System_state_sent', 'no_connection'],
                             input_keys=['index', 'failed_instance', 'image_message','laser_message', 'T265_message', 'joint_state_message', 'got_sensor_data'],
                             output_keys=['index', 'failed_instance'])

    def execute(self, userdata):  # Userdata = plan_out
        # Wait 2s (so we can see we go through this state in the GUI)
        time.sleep(2)

        # Still to finish coding - for now print messages


        if userdata.got_sensor_data == 1:
            userdata.index = userdata.index + 1
            rospy.loginfo('--- Depth Camera ---')
            rospy.loginfo(userdata.image_message)
            rospy.loginfo('--- ------------ ---')
            rospy.loginfo('--- LiDAR ---')
            rospy.loginfo(userdata.laser_message)
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


class FollowingState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished'])

    def execute(self, userdata):
        # ActivateFollow()
        # Follow during 10s
        time.sleep(2)
        # ActivateStop()
        return 'finished'


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
                               transitions={'go_to': 'MOVE_TO',
                                            'follow_up': 'CHOOSE_ACTION',
                                            'task_execution': 'EXECUTE_TASK',
                                            'follow': 'FOLLOWING'},
                               remapping={})

        smach.StateMachine.add('EXECUTE_TASK', ExecuteTask(),
                               transitions={'follow': 'FOLLOWING',
                                            'read_arTag': 'READ_ARTAG',
                                            'communicate_arTag_data': 'COMMUNICATE_ARTAG',
                                            'take_image': 'TAKE_IMAGE',
                                            'communicate_image_data': 'COMMUNICATE_IMAGE',
                                            'make_available': 'MAKE_AVAILABLE',
                                            'get_data_from_sensors': 'GET_DATA_FROM_SENSORS',
                                            'send_system_state': 'SEND_SYSTEM_STATE'})

        smach.StateMachine.add('READ_ARTAG', ReadArTag(),
                               transitions={'arTag_read': 'CHOOSE_ACTION',
                                            'waiting_for_arTag':  'MOVE_TO',
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

        smach.StateMachine.add('MAKE_AVAILABLE', make_available(),
                               transitions={'Second_system_available': 'CHOOSE_ACTION',
                                            'Second_system_not_available': 'aborted'},
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

        smach.StateMachine.add('MOVE_TO', MoveToWP(),
                               {'found_path': 'CHOOSE_ACTION', 'waiting': 'MOVE_TO', 'aborted': 'preempted'})



        smach.StateMachine.add('FOLLOWING', FollowingState(),
                               transitions={'finished': 'succeeded'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    sis.stop()

    rospy.signal_shutdown('All done.')


