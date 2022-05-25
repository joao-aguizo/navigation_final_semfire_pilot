#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import ros_numpy
import numpy as np
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt
from time import perf_counter

class CostmapFusion:

	def __init__(self):

		self.roughness = OccupancyGrid()
		self.obstacles = OccupancyGrid()

	def get_costmaps(self, choose_map):
		
		if choose_map == "both":
			self.roughness = rospy.wait_for_message('new_local_costmap', OccupancyGrid)
			self.obstacles = rospy.wait_for_message('evident_obstacles_map', OccupancyGrid)
			return "both"
		elif choose_map == "roughness":
			self.roughness = rospy.wait_for_message('new_local_costmap', OccupancyGrid)
			return "roughness"
		elif choose_map == "evident":
			self.obstacles = rospy.wait_for_message('evident_obstacles_map', OccupancyGrid)
			return "evident"
		else:
			rospy.logerr("Data Fusion Node: Unknown map name received inside 'get_costmaps'!")
			return None


	def fuse_layers(self, map_type):

		if map_type == "both":

			fused_costmap = []

			i = 0
			for elem in self.obstacles.data:
				if elem == 100:
					fused_costmap.append(100)
				else:
					fused_costmap.append(self.roughness.data[i])
				i += 1
			self.publish_map(fused_costmap)

		elif map_type == "roughness":
			self.roughness.data = [-1 if elem == 0 else elem for elem in self.roughness.data]
			self.publish_map(self.roughness.data)

		elif map_type == "evident":
			self.publish_map(self.obstacles.data)

		else:
			rospy.logerr("Data Fusion Node: Unknown map name received inside 'fuse_layers'!")


	def publish_map(self, fused_costmap):

		# Create an OccupancyGrid variable
		grid = OccupancyGrid()

		# Complete the information to publish in the message
		grid.header.stamp = self.roughness.header.stamp
		grid.header.frame_id = self.roughness.header.frame_id

		grid.info.resolution = self.roughness.info.resolution
		grid.info.width = self.roughness.info.width
		grid.info.height = self.roughness.info.height
		grid.info.origin.position.x = self.roughness.info.origin.position.x
		grid.info.origin.position.y = self.roughness.info.origin.position.y
		grid.info.origin.position.z = self.roughness.info.origin.position.z
		grid.info.origin.orientation.x = self.roughness.info.origin.orientation.x
		grid.info.origin.orientation.y = self.roughness.info.origin.orientation.y
		grid.info.origin.orientation.z = self.roughness.info.origin.orientation.z
		grid.info.origin.orientation.w = self.roughness.info.origin.orientation.w

		grid.data = np.array(fused_costmap).astype('int8')

		pub = rospy.Publisher("fused_costmap", OccupancyGrid, queue_size=10)

		pub.publish(grid)


	def main(self, choose_map):
		rate = rospy.Rate(1)
		while not rospy.is_shutdown():
			t = perf_counter()
			map_type = self.get_costmaps(choose_map)
			self.fuse_layers(map_type)
			elapsed = perf_counter() - t
			print(f"[FUSION] The elapsed time was: {elapsed:0.2f}.") # with f-string
			rate.sleep()




if __name__ == "__main__":

	rospy.init_node('costmap_fusion')

	choose_map = rospy.get_param('~fusion_type', "both")

	fusion = CostmapFusion()
	fusion.main(choose_map)