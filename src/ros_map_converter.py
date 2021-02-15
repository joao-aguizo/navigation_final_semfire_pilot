#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import print_function, division
import rospy
import ros_numpy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
import copy
import time
import tf

# !!! COPYRIGHT NOTICE !!!:
# This script is based on the "nav_2d_voxels" package by Dora Lourenço
# The original package can be found here: https://gitlab.ingeniarius.pt/semfire_ing_uc/navigation/nav_2d_voxels


class RosMapConverter:
	"""
	Class that converts a 2D matrix of interpolated heights into an OccupancyGrid map and tries to publish in ROS    
	"""

	def __init__(self):
		self.listener = tf.TransformListener()

	def nan_percentage(self, array):

		number_cells = array.shape[0]*array.shape[1]

		number_cells_nan = 0

		for line in range(array.shape[0]):
			for col in range(array.shape[1]):
				if np.isnan(array[line,col]):
					number_cells_nan += 1

		percentage = (number_cells_nan*100)/number_cells

		return percentage
	
	def normalize_matrix_gradient(self, map, threshold):
		""" 
		This function receive a matrix and normalize, returning the matrix normalized
		"""
		matrix = np.array(copy.deepcopy(map))
	
		# Find maximum and minimum
		# min = np.nanmin(matrix)
		# max = np.nanmax(matrix)
		min = 0.0
		max = threshold

		matrix_normalized = (matrix - min)/(max - min)

		return matrix_normalized

	def normalize_matrix_effort(self, map):

		matrix = np.array(copy.deepcopy(map))

		# Maximum and minimum
		min = -1
		max = 1

		matrix_normalized = (matrix - min)/(max - min)

		return matrix_normalized

	def normalize_matrix_effort_and_gradient(self, map, threshold):

		matrix = np.array(copy.deepcopy(map))

		# Maximum and minimum
		min = -threshold
		max = threshold

		# for i in range(matrix.shape[0]):
		# 	for j in range(matrix.shape[1]):
		# 		if not np.isnan(matrix[i,j]):
		# 			print (matrix[i,j])
		matrix_normalized = (matrix - min)/(max - min)
		# print(f"after: {matrix_normalized}")

		return matrix_normalized

	def grid_data_from_map(self, map):
		"""
		Function that receive the map and places the map information into 
		the data type of an OccupancyGrid map (Probabilities are in the range [0,100].  Unknown is -1)
		"""
		grid_data = []
		
		for col in range(map.shape[1]):
			for line in range(map.shape[0]):
				if np.isnan(map[line,col]):
					data = -1
				else:
					data = map[line,col]*100

				grid_data.append(data)

		grid_data = np.array(grid_data).astype('int8')

		return grid_data

	def grid_i_want(self, map):

		for line in range(map.shape[0]):
			for col in range(map.shape[1]):
				if line == 8 and col == 5:
					map[line,col] = 1
				else:
					map[line,col] = 0
		
		grid_data = []
			
		for col in range(map.shape[1]):
			for line in range(map.shape[0]):
				if np.isnan(map[line,col]):
					data = -1
				else:
					data = map[line,col]*100

				grid_data.append(data)

		grid_data = np.array(grid_data).astype('int8')

		return grid_data

	def get_bobcat_pose(self):
		"""
		Listen the yaw rotation and the translation from map to bobcat_base
		"""

		# listener = tf.TransformListener()

		self.listener.waitForTransform("map", "base_link", rospy.Time(0),rospy.Duration(4.0))

		translation = self.listener.lookupTransform("map", "base_link", rospy.Time(0))[0]

		return translation

		# return bobcat_x, bobcat_y


	def publish_map(self, map_matrix, dimension, resolution, threshold, choose_map, bobcat2map_translation, time, costmap_name):
		"""
		This function receive a matrix with a 2D map is going to create a OccupacyGrid type message message
		TODO: Discribe better 
		"""
		# Normalize gradient map information depending on the flag
		if choose_map == 'b':
			map_normalized = self.normalize_matrix_effort_and_gradient(map_matrix, threshold)
		if choose_map == 'e':
			map_normalized = self.normalize_matrix_effort(map_matrix)
		if choose_map == 'g':
			map_normalized = self.normalize_matrix_gradient(map_matrix, threshold)

		# Create an OccupancyGrid variable
		grid = OccupancyGrid()

		# bobcat2map_translation = self.get_bobcat_pose()

		# Complete the information to publish in the message
		grid.header.stamp = time
		grid.header.frame_id = "map"

		grid.info.resolution = resolution
		grid.info.width = int(dimension/resolution)
		grid.info.height = int(dimension/resolution)
		grid.info.origin.position.x = -dimension/2 + bobcat2map_translation[0]
		grid.info.origin.position.y = -dimension/2 + bobcat2map_translation[1]
		grid.info.origin.position.z = 0.0
		grid.info.origin.orientation.x = 0.0
		grid.info.origin.orientation.y = 0.0
		grid.info.origin.orientation.z = 0.0
		grid.info.origin.orientation.w = 1.0

		# percentage = self.nan_percentage(map_matrix)

		grid.data = self.grid_data_from_map(map_normalized)
		# grid.data = self.grid_i_want(map_normalized)

		pub = rospy.Publisher(costmap_name, OccupancyGrid, queue_size=10)
		# pub = rospy.Publisher('/move_base/global_costmap/costmap', OccupancyGrid, queue_size=10)
		

		pub.publish(grid)