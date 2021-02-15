#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import print_function, division
import rospy
import ros_numpy
import numpy as np
# from interpolation import Interpolator
from math import sqrt, pow, pi, tan, degrees
import copy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from ros_map_converter import RosMapConverter
import matplotlib.pyplot as plt

# !!! COPYRIGHT NOTICE !!!:
# This script is based on the "nav_2d_voxels" package by Dora Lourenço
# The original package can be found here: https://gitlab.ingeniarius.pt/semfire_ing_uc/navigation/nav_2d_voxels


class HeightMapper:
	"""
	Class that receives 3D numpy matrices with points, dimension and resolution of the map. Builds an interpolated height map and its gradient
	"""

	def separate_xyz_from_3d_matrix(self,xyz_array):
		"""
		This function receive a 3D matrix and separate the coordinates x,y and z in different arrays
		"""
		x = np.array(xyz_array[:,0])
		y = np.array(xyz_array[:,1])
		z = np.array(xyz_array[:,2])

		return x, y, z

	def create_2d_grid(self,x,y,z,dimension,n_cells,translation):
		"""
		Given the x an y cordinates of the points, and the dimension and the resolution of the grid returns
		2 arrays (x and y indexes) telling in which cell the points are and another array with the respective height.
		Delete the points out of the grid.
		"""
		# Save the values of height (z)
		height = z

		# bins array
		bins_x = np.array(np.linspace(translation[0] - dimension/2, translation[0] + dimension/2,n_cells+1))
		bins_y = np.array(np.linspace(translation[1] - dimension/2, translation[1] + dimension/2,n_cells+1))
			
		# See in which cell the x and y coordinates are
		x_cell = np.digitize(x,bins_x)
		y_cell = np.digitize(y,bins_y)

		# List to save the index of the points out of map
		index = []

		# If x or y = 0 or x or y = cells+1 means that the points are out of the map 
		for out_map in range(x_cell.size): # The size o x_cells and y_cells are the same
			if x_cell[out_map] < 1 or y_cell[out_map] < 1  or x_cell[out_map] > n_cells or y_cell[out_map] > n_cells:
				index.append(out_map)         
				
		x_cell = np.delete(x_cell,index)
		y_cell = np.delete(y_cell,index)
		height = np.delete(height,index)

		x_cell[:] = x_cell-1
		y_cell[:] = y_cell-1

		return x_cell, y_cell, height 

	def combine_cell_values(self,values,z_bobcat):
		"""
		Assume that we receive an iterable with the possible values for a cell and return the median.
		"""

		values = values[~np.isnan(values)]

		if values.size > 0:
			min_ = np.min(values)
			values = [elem for elem in values if elem < min_ + 2.5]
			values = [elem - z_bobcat for elem in values]
			# print("new = " + str(values))
			return np.median(values) 
		else:
			return np.NaN


	def nan_percentage(self, array):

		number_cells = array.shape[0]*array.shape[1]

		number_cells_nan = 0

		for line in range(array.shape[0]):
			for col in range(array.shape[1]):
				if np.isnan(array[line,col]):
					number_cells_nan += 1

		percentage = (number_cells_nan*100)/number_cells

		return percentage

	def create_map_with_lidar_points(self,x,y,z,cells,z_bobcat,func):
		""" 
		Function to create the map with the LIDAR points, and leave the cells without information with NaN and
		treat cells with more than one LIDAR point and put the values in the cells.
		The cells with more than one point are treated with the function combine_cell_values()
		"""
		# Inicialize the array the type array[x_cell,y_cell] = z with NaN
		array = np.empty((cells,cells,500))
		array[:] = np.NaN
		final_array = np.empty((cells,cells))
		final_array[:] = np.NaN
		times = np.zeros((cells,cells)).astype('int')
		
		for idx, (i, j) in enumerate(zip(x, y)):
			array[i, j, times[i,j]] = z[idx]
			if times[i,j] < 499:
				times[i,j] += 1

		for i in range(cells):
			for j in range(cells):
				final_array[i, j] = func(array[i, j], z_bobcat)

		return final_array

	def calculate_parcial_derivatives_gradient(self, map, resolution):
		"""
		Receive the map after the interpolation and calculate the gradient in each cell. This function returns the partial
		derivatives (dx, dy)
		"""
		dx, dy = np.gradient(map,resolution)
		return dx, dy


	def if_upward_or_downward_slope(self, dx, dy, n_cells, dimension, threshold, translation, gradient_norm):
		""" 
		This function receive the parcial derivatives and see point to point if it is 
		a upward slope or a downward slope
		See the effort between the vector that joins the position of the robot 
		with the position of the point we are analyzing and the gradient vector at 
		the point that we're looking at
		"""

		bins_x = np.array(np.linspace(translation[0] - dimension/2, translation[0] + dimension/2,n_cells+1))
		bins_y = np.array(np.linspace(translation[1] - dimension/2, translation[1] + dimension/2,n_cells+1))

		# Robot is in the (0,0) pose in the map, see in what cell this coordinates are
		x = np.digitize(translation[0],bins_x)-1
		y = np.digitize(translation[1],bins_y)-1

		# dx and dy have the same dimension
		n_lines = dx.shape[0]
		n_cols = dx.shape[1]

		# Matrix to save the values of the effort of each point
		effort = np.empty((n_lines, n_cols))

		# Go through all the points to calculate the effort
		for line in range(n_lines):
			for col in range(n_cols):
				if (line == x and col == y): # Isto vão ser para mudar
					effort[line, col] = 0.0

				else:
					# If the gradient norm is bigger than 
					if gradient_norm[line, col] >= threshold:
						effort[line,col] = 1.0
						# print(f"gradient_norm[{line},{col}] = {gradient_norm[line,col]}")

					else:
						# Obtain the vector that joins the position of the robot with the position of the point
						# we are analyzing
						vector_point = [line - x, col - y]
						vector_gradient = [dx[line,col], dy[line, col]]
						# print(f"vector_gradient = {vector_gradient}")

						# Obtain the unit vector of each vector
						unit_vector_point = vector_point/np.linalg.norm(vector_point)
						if dx[line,col] != 0.0 and dy[line,col] != 0.0 and not np.isnan(dx[line,col]) and not np.isnan(dy[line,col]):
							unit_vector_gradient = vector_gradient/np.linalg.norm(vector_gradient)
						elif np.isnan(dx[line,col]) and np.isnan(dy[line,col]):
							unit_vector_gradient = [np.NaN,np.NaN]
						else:
							unit_vector_gradient = [0,0]
						# print(f"unit_vector_gradient = {unit_vector_gradient}")

						# Do the dot product between the 2 unit vectors
						dot_product = np.dot(unit_vector_point, unit_vector_gradient)
						# print(f"dot product = {dot_product}")

						# Obtain the angle between the 2 vectors
						angle = np.arccos(dot_product)
						# angle_degrees = degrees(angle)

						effort[line, col] = np.cos(angle)
						# print(f"effort = {effort[line,col]}")

		return effort

	def calculate_gradient_and_apply_threshold_to_gradient(self, dx, dy, threshold):
		"""
		This function receive the partial derivatives and calculate the gradient norm and apply a certain threshold
		"""
		n_lines = dx.shape[0]
		n_col = dy.shape[1]

		gradient_limited_map = np.empty((n_lines,n_col))
		# Matrix to save the norm of the gradient
		gradient = np.empty((n_lines,n_col))


		for line in range(n_lines):
			for col in range(n_col):

				if np.isnan(dx[line, col]) or np.isnan(dy[line, col]):
					gradient[line, col] = 0.0

				else:
					vector = [dx[line, col], dy[line, col]]
					gradient[line, col] = np.linalg.norm(vector)


				if gradient[line,col] >= threshold:
					gradient_limited_map[line,col] = threshold

				else:
					gradient_limited_map[line,col] = gradient[line, col]

		return gradient, gradient_limited_map

	def map_with_gradient_and_effort(self, effort, gradient, n_cells):
		"""
		Build a map taking into account the effort (if it is upward or downward slope) and the gradient norm
		limited with the threshold
		"""

		map = np.empty((n_cells, n_cells))

		for line in range(n_cells):
			for col in range(n_cells):
				map[line, col] = effort[line,col] * gradient[line,col]

		return map
				

	def build_map(self, xyz_array, dimension, resolution, meters, threshold, plot_bool, choose_map, roll, pitch, translation, func):
		"""
		Build the map from a numpy XYZ pointcloud where each line is a point.
		TODO: descrever melhor
		"""

		# converter = RosMapConverter()

		# translation = converter.get_bobcat_pose()


		# Number of cells
		n_cells = int(dimension/resolution)
		
		
		# Separate x,y and z coordinates of each point
		x, y, z = self.separate_xyz_from_3d_matrix(xyz_array)


		# Create a 2d grid. receive the indexes (x and y) of each point
		# telling in which cell the point are and repective height
		x_index, y_index, z_height = self.create_2d_grid(x, y, z, dimension, n_cells, translation)

		
		# Create a matrix to be like map_before_interpolation[x_index,y_index] = height with the point from LIDARs
		map_ = self.create_map_with_lidar_points(x_index,y_index,z_height,n_cells, translation[2], func)


		# With this function we obtain the parcial derivatives of the gradient
		dx, dy = self.calculate_parcial_derivatives_gradient(map_, resolution)


		# Calculate the norm of the gradient and apply a certain threshold to the gradient values and receive the 2D map
		gradient_norm_map, gradient_limited_map = self.calculate_gradient_and_apply_threshold_to_gradient(dx, dy, threshold)

		# Calculate the effort of each point to see if is upward or downward
		effort = self.if_upward_or_downward_slope(dx, dy, n_cells, dimension, threshold, translation, gradient_norm_map)

		# Obtain the map that relates the effort with the gradient norm
		map_effort_and_gradient = self.map_with_gradient_and_effort(effort, gradient_limited_map, n_cells)
		# plt.figure()
		# # plt.contour(range(200), range(200), map_effort_and_gradient)
		# plt.quiver(range(n_cells), range(n_cells),dy,dx)
		# # plt.colorbar()
		# plt.axis([0, n_cells, n_cells, 0])
		# plt.show()

		if choose_map == 'e':
			return effort

		if choose_map == 'g':  
			return gradient_limited_map
		
		if choose_map == 'b':
			return map_effort_and_gradient