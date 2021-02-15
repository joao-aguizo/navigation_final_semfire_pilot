#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
import rospy

class timestampSwitcher:

	def __init__(self):
		self.time = Clock()
		self.pointcloud = PointCloud2()
		self.pub = rospy.Publisher("octomap_point_cloud_centers", PointCloud2, queue_size=10)

	def read_time_and_pointcloud(self):
		self.time = rospy.wait_for_message("clock", Clock)
		self.pointcloud = rospy.wait_for_message("test_pc2", PointCloud2)

	def change_timestamp(self):
		# self.pointcloud.header.stamp.secs = self.time.clock.secs
		# self.pointcloud.header.stamp.nsecs = self.time.clock.nsecs
		self.pointcloud.header.stamp = rospy.Time.now()

	def publish_changed_pointcloud(self):
		self.pub.publish(self.pointcloud)

	def main(self):
		rate = rospy.Rate(10)

		while not rospy.is_shutdown():
			self.read_time_and_pointcloud()
			self.change_timestamp()
			self.publish_changed_pointcloud()
			rate.sleep()


if __name__ == "__main__":
	rospy.init_node('timestamp_changer')
	
	switcher = timestampSwitcher()
	switcher.main()