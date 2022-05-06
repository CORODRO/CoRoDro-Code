#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 28 08:23:19 2021
@author: j.rimani
"""

# Import your directories
import sys
# Call the directory where the hipop package are
# sys.path.append('/home/dcas/j.rimani/hipop-python-master')
# from hddl_wrapper import hddl_wrapper

import time
import logging
import rospy
import json
import smach
import smach_ros
import roslaunch
import subprocess
import glob
import os
from os.path import exists

from datetime import datetime


from math import radians, pi, sin, cos, atan2, sqrt, fabs, degrees

LOGGER = logging.getLogger(__name__)

# from move_to.msg import MoveToAction, MoveToGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from std_msgs.msg import Empty
from std_msgs.msg import Bool
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import Header
from nav_msgs.msg import Odometry


###
frequence=10
ALT=rospy.get_param("altitude", 2) # Altitude
EPSILON=[0.2, 0.2, 0.2] #epsilon for position precision
NUMBER_OF_TIME_ON_POSE=10 #nombre de fois qu'il doit etre "arrived" pour le croire (pour rappel voir frequence pour le temps)
###


class Controller:
	# initialization method
	def __init__(self):
		# Instantiate a setpoints message
		self.sp = PoseStamped()
		self.sp.header=Header()
		
		self.EPSILON=EPSILON

                self.ALT=ALT
		# ROS loop rate
		self.rate = rospy.Rate(frequence)

		self.number_of_time_on_pose = 0

		# Real position
		#msg = rospy.wait_for_message("T265/odom/sample", Odometry)
		self.real_zero_alt = 0 #msg.pose.pose.position.z - 0.05
		self.linear = Point(0,0,self.real_zero_alt)
		self.angular = Point(0,0,0)

		# Topic
		self.sp_pub = rospy.Publisher("/SM/orders", PoseStamped, queue_size=1)

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
		#self.angular.w = msg.pose.pose.orientation.w

	def is_arrived_x(self):
		return abs(self.linear.x - self.sp.pose.position.x) <= self.EPSILON[0]
	
	def is_arrived_y(self):
		return abs(self.linear.y - self.sp.pose.position.y) <= self.EPSILON[1]
	
	def is_arrived_z(self):
		return abs(self.linear.z - self.sp.pose.position.z) <= self.EPSILON[2]
	
	def is_arrived(self):
		if self.number_of_time_on_pose == NUMBER_OF_TIME_ON_POSE:
			return True
		elif self.is_arrived_x() and self.is_arrived_y() and self.is_arrived_z():
			self.number_of_time_on_pose += 1
		return False
			
		
	def change_target(self, couple):
		self.sp.pose.position.x = couple[0]
		self.sp.pose.position.y = couple[1]
		self.number_of_time_on_pose = 0 # Reinit because target is changing
		self.send_order() # sending new target
	
	def change_alt(self, z):
		self.sp.pose.position.z = z
		self.number_of_time_on_pose = 0 # Reinit because target is changing
		self.send_order() # sending new target

	def send_order(self):
		self.sp_pub.publish(self.sp)
		self.sp_pub.publish(self.sp)
		rospy.loginfo("New target sent: {} {} {}".format(self.sp.pose.position.x, 
			self.sp.pose.position.y, self.sp.pose.position.z))
		



# Start The State Machine
class InitState(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['start'], input_keys=[], output_keys=['launch_package', 'folder_name', 'drone', 'sp_pub','id_pointcloud_processing'])

	def execute(self, userdata):
		global drone
                userdata.id_pointcloud_processing=0
		# In Which package are my launch files? - in the general state because is a general entry
		launch_package = rospy.get_param("launch_package",'my_pcl_tutorial')
		userdata.launch_package = launch_package
		rospy.loginfo(launch_package)


		# Name of the folder where everything is stocked
		maxi=0
		for f in os.listdir("/root/database"):
			tempo=f.split("_")
			if tempo[0]=="test" and tempo[1].isdigit():
				maxi=max(maxi, int(tempo[1]))

		folder_name = "test_"+str(maxi+1)+"_"+str(datetime.today()).replace(" ","_").replace(":","-")[:-7]+"/"
		userdata.folder_name = folder_name
		try:
			os.mkdir('/root/database/'+folder_name)
			rospy.loginfo("Folder created with name: " + folder_name)
		except:
			rospy.logwarn("Folder with name: "+ folder_name + " already exists")

		# Drone controller 
		drone = Controller()

		## Topics
		# Subscribe to drone's local position
		rospy.Subscriber('/mavros/local_position/odom', Odometry, drone.posCb)

		rospy.sleep(2)
		return 'start'


class DroneTakeOff(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['mapping_flying'], input_keys=['launch_package', 'folder_name', 'sp_pub'],output_keys=['launch_package', 'folder_name'])

	def execute(self, userdata):
                global drone
		#cmd=["rosrun","state_machines","setpoint_node.py"]
		#run_setpoint_node = subprocess.Popen(cmd)

		drone.change_alt(drone.ALT)

		# Waiting during ascent
		rospy.loginfo("TAKEOFF")
		while not(drone.is_arrived()):	   
			drone.rate.sleep()


		rospy.loginfo("Drone flying")

		return 'mapping_flying'

class MappingPath(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['path_finished_and_landed'], input_keys=['sp_pub', 'launch_package', 'pointcloud_saver_file_out', 'folder_name','rate', 'id_pointcloud_processing'],output_keys=['launch_package', 'folder_name', 'pointcloud_saver_file_out', 'id_pointcloud_processing'])

	def execute(self, userdata):
                global drone
		userdata.id_pointcloud_processing = -1 #Init before proccessing

		global HumanMapVisualizationClient, HumanMapVisualizationServer

		cmd_serv = ["rosrun", userdata.launch_package, "pointcloud_saver_server"]
		HumanMapVisualizationServer = subprocess.Popen(cmd_serv)

		topic_in_pointcloud_saver = rospy.get_param("topic_in_pointcloud_saver",'/octomap_point_cloud_centers')
		file_out_pointcloud_saver = rospy.get_param("file_out_pointcloud_saver",'/root/database/'+userdata.folder_name)
		userdata.pointcloud_saver_file_out=[]
		cmd = ["rosrun", userdata.launch_package, "pointcloud_saver_client", topic_in_pointcloud_saver,file_out_pointcloud_saver]
		HumanMapVisualizationClient=[]

		#dict={7:'25p_', 11:'50p_', 13:'75p_', 16:'100p_'}
		dict={2:'25p_', 5:'50p_', 7:'75p_', 9:'100p_'}

		#cross=[[5,0],[0,0],[-5,0],[0,0],[0,5],[0,0],[0,-5],[0,0]]
		#path=[[4.25,-4],[-4.25,-4],[-4.25,-2],[4.25,-2],[4.25,2],[-4.25,2],[-4.25,4],[4.25,4], [0,0]]
		#targets=cross + path
		targets=[[4.25,-4],[-4.25,-4],[-4.25,-2],[4.25,-2],[4.25,0],[-4.25,0],[-4.25,2],[4.25,2],[4.25,4],[-4.25,4], [0,0]]
	# 	for i in range(len(targets)):
	#		targets[i][0]*=2.5/5.0
	#		targets[i][1]*=2.5/5.0

		# Path
		for id_target in range(len(targets)):
			drone.change_target(targets[id_target])
			while not(drone.is_arrived()):
				drone.rate.sleep()

			if id_target==0:
				# Starting mapping process
				rospy.loginfo('Launch Mapping')
				launch_file_mapping = rospy.get_param("launch_file_name", 'dronemappingphase.launch')
				cmd_map = ["roslaunch", userdata.launch_package, launch_file_mapping, "filename_tag:=/root/database/"+userdata.folder_name+"dronedatabase.txt"]
				launch_RealTimeMappingProcess = subprocess.Popen(cmd_map)
				# out, err = launch_RealTimeMappingProcess.communicate()
				rospy.sleep(10)

			# Saving at 25% 50% 75% 100%
			if id_target in [2, 5, 7, 9]:
				if id_target==9:
					rospy.sleep(4)
				rospy.loginfo("Creating datas for the {}%".format(dict[id_target][:-2]))
				cmd = ["rosrun", userdata.launch_package, "pointcloud_saver_client", topic_in_pointcloud_saver,file_out_pointcloud_saver + dict[id_target]+'3DdemMap.pcd']
				HumanMapVisualizationClient.append(subprocess.Popen(cmd))
				userdata.pointcloud_saver_file_out.append(file_out_pointcloud_saver + dict[id_target]+'3DdemMap.pcd')
                                userdata.id_pointcloud_processing = userdata.id_pointcloud_processing + 1


		# Landing

		rospy.loginfo("LANDING")
		t = time.time()

		drone.change_alt(drone.real_zero_alt)
		while not(drone.is_arrived()):
			if time.time() - t > 8 :
				# drone.change_alt(-2) # ending signal
				rospy.logerr("Timeout while trying to land ( > 8s )")
				break
			drone.rate.sleep()


		rospy.loginfo('Terminate Launch Mapping')
		launch_RealTimeMappingProcess.terminate()

		global OutlineRemovalServer,CallDenosingServer,ExtractionServer,GridMapGeneratorService

		rospy.loginfo('Start outline removal server')
		cmd = ["rosrun", userdata.launch_package, "outliers_removal_server"]
		OutlineRemovalServer = subprocess.Popen(cmd)

		rospy.loginfo('Start denoising server')
		cmd = ["rosrun", userdata.launch_package, "denoising_server"]
		CallDenosingServer = subprocess.Popen(cmd)


                # Should the launch file be running when we launch the service or can I close it?
		rospy.loginfo('Start Conversion Extraction Server')
		cmd = ["rosrun", userdata.launch_package, "conversion_extraction_server"]
		ExtractionServer = subprocess.Popen(cmd)

		rospy.loginfo('Start occupancymap_generator_server')
		cmd = ["rosrun", userdata.launch_package, "occupancymap_generator_server"]
		GridMapGeneratorService = subprocess.Popen(cmd)

		rospy.sleep(5)

		return 'path_finished_and_landed'


class outlineRemoval(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['cleaned_map'], input_keys=['launch_package','pointcloud_saver_file_out', 'id_pointcloud_processing', 'folder_name'],
							 output_keys=['launch_package','denosing_out_file','pointcloud_saver_file_out'])

	def execute(self, userdata):
		prefix=['25p_', '50p_', '75p_', '100p_']

		global OutlineRemovalClient, CallDenosingClient

		rospy.loginfo('Start Outline Removal Client')
		file_in_OutlineRemoval = rospy.get_param("file_in_OutlineRemoval", userdata.pointcloud_saver_file_out[userdata.id_pointcloud_processing])
		file_out_OutlineRemoval = rospy.get_param("file_out_OutlineRemoval", '/root/database/'+userdata.folder_name+prefix[userdata.id_pointcloud_processing]+'test_outliersx1.pcd')
		radius_search_OutlineRemoval = rospy.get_param("radius_search_OutlineRemoval", '0.06')
		min_neighbors_in_radius_OutlineRemoval = rospy.get_param("min_neighbors_in_radius_OutlineRemoval", '4')

		cmd = ["rosrun", userdata.launch_package, "outliers_removal_client", file_in_OutlineRemoval,
			   file_out_OutlineRemoval, radius_search_OutlineRemoval, min_neighbors_in_radius_OutlineRemoval]
		OutlineRemovalClient = subprocess.Popen(cmd)



		file_in_Denosing = rospy.get_param("file_in_Denosing", '/root/database/'+userdata.folder_name+prefix[userdata.id_pointcloud_processing]+'test_outliersx1.pcd')
		file_out_Denosing = rospy.get_param("file_out_Denosing", '/root/database/'+userdata.folder_name+prefix[userdata.id_pointcloud_processing]+'test_outliersx1_denoisedx1.pcd')

		# file input to the grid map
		userdata.denosing_out_file = prefix[userdata.id_pointcloud_processing]+'test_outliersx1_denoisedx1.pcd'


                while not exists(file_in_Denosing):
			rospy.sleep(1)
		rospy.sleep(3)


		rospy.loginfo('Start Denoising Service')

		cmd = ["rosrun", userdata.launch_package, "denoising_client", file_in_Denosing, file_out_Denosing]
		CallDenosingClient = subprocess.Popen(cmd)

		return 'cleaned_map'


class GridMapGeneration(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['obtained_grid_map', 'next_pointcloud_process'], input_keys=['launch_package','denosing_out_file', 'pointcloud_saver_file_out','id_pointcloud_processing', 'folder_name'],
							 output_keys=['launch_package', 'id_pointcloud_processing'])

	def execute(self, userdata):
		prefix=['25p_', '50p_', '75p_', '100p_']

		while not exists('/root/database/'+userdata.folder_name+prefix[userdata.id_pointcloud_processing]+'test_outliersx1_denoisedx1.pcd'):
			rospy.sleep(1)
		rospy.sleep(3)

		rospy.loginfo('Start GridMapLoader')

		launch_GridMapLoader = rospy.get_param("launch_GridMapLoader_file", 'grid_map_pcl_loader_node.launch')
                launch_MapfromPointCloud = subprocess.Popen(['roslaunch', userdata.launch_package, launch_GridMapLoader, 'folder_path:='+'/root/database/'+userdata.folder_name,
                                                 'pcd_filename:='+prefix[userdata.id_pointcloud_processing]+'test_outliersx1_denoisedx1.pcd','output_grid_map:='+prefix[userdata.id_pointcloud_processing]+'elevation_map.bag'])


		# is there a parameter that we have to take or the name of the file is always the same?
		file_in_ExtractionServer = rospy.get_param("file_in_ExtractionServer", '/root/database/'+userdata.folder_name+prefix[userdata.id_pointcloud_processing]+'elevation_map.bag')
		file_out_ExtractionServer = rospy.get_param("file_out_ExtractionServer", '/root/database/'+userdata.folder_name+prefix[userdata.id_pointcloud_processing]+'extracted_map.pcd')

		while not exists(file_in_ExtractionServer):
			rospy.sleep(1)
		rospy.sleep(3)

		rospy.loginfo('Start Conversion Extraction Client')
		cmd = ["rosrun", userdata.launch_package, "conversion_extraction_client", file_in_ExtractionServer,
			   file_out_ExtractionServer]
		ExtractionClient = subprocess.Popen(cmd)




		rospy.loginfo('Start Occupancymap_generator_client')
		# is there a parameter that we have to take or the name of the file is always the same?
		file_in_GridMapGenerator = rospy.get_param("file_in_GridMapGenerator", '/root/database/'+userdata.folder_name+prefix[userdata.id_pointcloud_processing]+'extracted_map.pcd')
		file_out_GridMapGenerator = rospy.get_param("file_out_GridMapGenerator", '/root/database/'+userdata.folder_name+prefix[userdata.id_pointcloud_processing]+'2d_final_map')
		frame_id_GridMapGenerator=rospy.get_param("frame_id_GridMapGenerator",'map')
		resolution_GridMapGenerator = rospy.get_param("resolution_GridMapGenerator",'0.08')
		resolution_discretized_GridMapGenerator=rospy.get_param("resolution_discretized_GridMapGenerator",'0.3')
		resolution_grid_map_pcl_node_Grid=rospy.get_param("resolution_grid_map_pcl_node",'0.035')


		while not exists(file_in_GridMapGenerator):
			rospy.sleep(1)
		rospy.sleep(3)

		cmd = ["rosrun", userdata.launch_package, "occupancymap_generator_client", file_in_GridMapGenerator,
			   file_out_GridMapGenerator,frame_id_GridMapGenerator,resolution_GridMapGenerator,resolution_discretized_GridMapGenerator,resolution_grid_map_pcl_node_Grid]
		GridMapGeneratorClient = subprocess.Popen(cmd)

		while not exists(file_out_GridMapGenerator+'.yaml'):
			rospy.sleep(1)
		rospy.sleep(3)

		userdata.id_pointcloud_processing += 1

		global OutlineRemovalClient, CallDenosingClient 
		rospy.loginfo("Terminate subprocesses")
		OutlineRemovalClient.terminate()
		CallDenosingClient.terminate()
		ExtractionClient.terminate()
		GridMapGeneratorClient.terminate()

		#if userdata.id_pointcloud_processing < len(userdata.pointcloud_saver_file_out) :
		#	return 'next_pointcloud_process'

		print("-------- FINISHED --------")
		return 'obtained_grid_map'


if __name__ == '__main__':
	# State Machine Visualize the steps of the plan
	rospy.init_node('Mapping_Smach')

	# Rate0
	rate = rospy.get_param("~rate", 1)

	''' All paramaters used in the State Machines '''

	# Head visibility limits
	# head_height = rospy.get_param("~head_height", 0.116)
	# head_boarder_distance = rospy.get_param("~head_boarder_distance ", 0.03)

        # Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['succeeded'])

	with sm:
		smach.StateMachine.add('INIT', InitState(),
							   transitions={'start': 'DroneTakeOff'})

		smach.StateMachine.add('DroneTakeOff', DroneTakeOff(),
							   transitions={'mapping_flying': 'MappingPath'},
							   remapping={})

		smach.StateMachine.add('MappingPath', MappingPath(),
							   transitions={'path_finished_and_landed': 'outlineRemoval'},
							   remapping={})

		smach.StateMachine.add('outlineRemoval', outlineRemoval(),
							   transitions={'cleaned_map': 'GridMapGeneration'},
							   remapping={})

		smach.StateMachine.add('GridMapGeneration', GridMapGeneration(),
							   transitions={'next_pointcloud_process': 'outlineRemoval', 'obtained_grid_map': 'succeeded'},
							   remapping={})

	# Create and start the introspection server
	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()

	# Execute SMACH plan
	outcome = sm.execute()

	global HumanMapVisualizationClient, HumanMapVisualizationServer
	HumanMapVisualizationServer.terminate()
	for proc in HumanMapVisualizationClient:
		proc.terminate()

	global OutlineRemovalServer,CallDenosingServer,ExtractionServer,GridMapGeneratorService
	OutlineRemovalServer.terminate()
	CallDenosingServer.terminate()
	ExtractionServer.terminate()
	GridMapGeneratorService.terminate()

	sis.stop()

	rospy.loginfo("THE END !")

	rospy.signal_shutdown('All done.')
