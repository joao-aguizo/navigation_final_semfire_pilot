#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import ros_numpy
import numpy as np
from laser_interface2 import LaserInterface2
from height_mapper import HeightMapper
from ros_map_converter import RosMapConverter
import time
from statistics import variance, mean


def main():
	"""
	Arranca um ROS node que está continuamente a construir e publicar mapas
	"""

	last_xyz_cloud = []
	last_points = None
	last_translation = []
	last_tf_time = 0

	# Object build
	laser_interface = LaserInterface2()
	mapper = HeightMapper()
	converter = RosMapConverter()

	choose_map = rospy.get_param('~map', 'b')
	source = rospy.get_param('~source', 'octo')
	frequency = rospy.get_param('~frequency', 'loop')
	dimension = rospy.get_param('~dimension', 50)
	resolution = rospy.get_param('~resolution', 1)
	meters = rospy.get_param('~meters', 9)
	threshold = 2.5
	bool_plot = False
	main.variance_threshold = rospy.get_param('~variance_threshold', 0.5)
	main.mean_threshold = rospy.get_param('~mean_threshold', 1.0)
	main.variation_threshold = rospy.get_param('~variation_threshold', 5.0)
	main.evident_threshold = rospy.get_param('~evident_threshold', 5.5)

	rate = rospy.Rate(1)

	while not rospy.is_shutdown():

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
			map_ = mapper.build_map(last_xyz_cloud, dimension, resolution, meters, threshold, bool_plot, choose_map, roll, pitch, last_translation, evident_obstacle)
			converter.publish_map(map_, dimension, resolution, threshold, choose_map, translation, last_tf_time, 'evident_obstacles_map')
		else:
			roll, pitch = laser_interface.listen_map_tf()[1:]	# The slice from a tuple has the first index inclusive and the second exclusive!
		
			# Build the map
			map_ = mapper.build_map(xyz_cloud, dimension, resolution, meters, threshold, bool_plot, choose_map, roll, pitch, translation, evident_obstacle)

			# Publish in ROS
			converter.publish_map(map_, dimension, resolution, threshold, choose_map, translation, tf_time, 'evident_obstacles_map')

		elapsed = time.perf_counter() - t

		# print("The elapsed time was: " + str(format(elapsed, '.2f')) + " with " + str(points) + "points.") # classic
		if use_last_one == False:
			print(f"[EVIDENT] The elapsed time was: {elapsed:0.2f} with {points} points.") # with f-string
		else:
			print(f"[EVIDENT] The elapsed time was: {elapsed:0.2f} with {last_points} points.") # with f-string


		rate.sleep()


def evident_obstacle(values, z_bobcat):
	"""
	This function receives the different values within a cell and returns a bool that holds whether it is an obstacle or not
	"""
	values = values[~np.isnan(values)]
	if values.size > 0:
		min_ = np.min(values)
		values = [elem for elem in values if elem < min_ + main.evident_threshold]
		values = [elem - z_bobcat for elem in values]

		if len(values) > 1:

			variance_ = variance(values)
			mean_ = mean(values)
			min_ = np.amin(values)
			max_ = np.amax(values)
			variation = max_ - min_

			if variance_ > main.variance_threshold and mean_ > main.mean_threshold and variation > main.variation_threshold:
				return 255
			else:
				return 0
		else:
			return 0
	else:
		return np.NaN

if __name__ == "__main__":

	rospy.init_node('evident_node', anonymous=True)
	main()
	