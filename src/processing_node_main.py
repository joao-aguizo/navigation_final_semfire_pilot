#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import print_function, division
import rospy
import ros_numpy
import numpy as np
from laser_interface2 import LaserInterface2 
from height_mapper import HeightMapper
from ros_map_converter import RosMapConverter
import sys
import time
from sensor_msgs.msg import PointCloud2

# !!! COPYRIGHT NOTICE !!!:
# This script is based on the "nav_2d_voxels" package by Dora Lourenço
# The original package can be found here: https://gitlab.ingeniarius.pt/semfire_ing_uc/navigation/nav_2d_voxels


def run_once(dimension, resolution, meters, bool_plot, choose_map, source):
	"""
	Obtain a pointcloud from LIDARs. Build, design and publish a map.
	"""
	

	# Object build
	laser_interface = LaserInterface2()
	mapper = HeightMapper() 
	converter = RosMapConverter()

	# Variable to safe the value of the threshold
	threshold = 0.5
	
	# Obtain pointcloud
	if source == 'l':
		xyz_cloud, roll, pitch = laser_interface.get_point_cloud_lidar()
	
	if source == 'o':
		xyz_cloud = laser_interface.get_point_cloud_octomap()


	# Build the map
	map_ = mapper.build_map(xyz_cloud, dimension, resolution, meters, threshold, bool_plot, choose_map, roll, pitch)
   
	# Publish in ROS
	converter.publish_map(map_, dimension, resolution, threshold, choose_map)

	


def run_loop(dimension, resolution, meters, bool_plot, choose_map, source):
	"""
	Arranca um ROS node que está continuamente a construir e publicar mapas
	"""

	# Object build
	laser_interface = LaserInterface2()
	mapper = HeightMapper() 
	converter = RosMapConverter()

	last_xyz_cloud = []
	last_points = None
	last_translation = []
	last_tf_time = 0

	gradient_threshold = rospy.get_param('~gradient_threshold', 0.35)
	# Variable to save the value of the threshold
	threshold = gradient_threshold

	rate = rospy.Rate(1)

	while not rospy.is_shutdown():

		# Obtain pointcloud
		if source == 'l':
			xyz_cloud, roll, pitch = laser_interface.get_point_cloud_lidar()
	
		if source == 'octo':
			t = time.perf_counter()

			xyz_cloud, points, translation, tf_time, use_last_one = laser_interface.get_point_cloud_octomap()

			if points != None:
				last_xyz_cloud = xyz_cloud
				last_points = points
				last_translation = translation
				last_tf_time = tf_time

			if points == None and use_last_one == False:
				continue
			elif use_last_one == True and last_points == None:
				continue
			elif use_last_one == True:
				roll, pitch = laser_interface.listen_map_tf()[1:]	# The slice from a tuple has the first index inclusive and the second exclusive!
				translation, rotation, tf_time = laser_interface.listen_octomap_tf()
				map_ = mapper.build_map(last_xyz_cloud, dimension, resolution, meters, threshold, bool_plot, choose_map, roll, pitch, translation, mapper.combine_cell_values)
				converter.publish_map(map_, dimension, resolution, threshold, choose_map, translation, tf_time, 'new_local_costmap')
			else:
				roll, pitch = laser_interface.listen_map_tf()[1:]	# The slice from a tuple has the first index inclusive and the second exclusive!
			
				# Build the map
				map_ = mapper.build_map(xyz_cloud, dimension, resolution, meters, threshold, bool_plot, choose_map, roll, pitch, translation, mapper.combine_cell_values)

				# Publish in ROS
				converter.publish_map(map_, dimension, resolution, threshold, choose_map, translation, tf_time, 'new_local_costmap')

			elapsed = time.perf_counter() - t

			# print("The elapsed time was: " + str(format(elapsed, '.2f')) + " with " + str(points) + "points.") # classic
			if use_last_one == False:
				print(f"The elapsed time was: {elapsed:0.2f} with {points} points.") # with f-string
			else:
				print(f"The elapsed time was: {elapsed:0.2f} with {last_points} points.") # with f-string

			rate.sleep()


if __name__ == "__main__":

	rospy.init_node('processing_node', anonymous=True)

	choose_map = rospy.get_param('~map', 'b')
	source = rospy.get_param('~source', 'octo')
	frequency = rospy.get_param('~frequency', 'loop')
	dimension = rospy.get_param('~dimension', 50)
	resolution = rospy.get_param('~resolution', 1)
	meters = rospy.get_param('~meters', 9)

	if frequency == "loop" and not frequency == "once":
		run_loop(dimension, resolution, meters, frequency, choose_map, source)

	if not frequency == "loop" and not frequency == "once":
		once = True
		run_once(dimension, resolution, meters, frequency, choose_map)
	
	if frequency == "loop" and frequency == "once":
		sys.exit('You cannot choose the flags --loop and --once at the same time')

	