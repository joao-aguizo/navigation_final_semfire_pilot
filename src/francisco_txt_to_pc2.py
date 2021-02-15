#!/usr/bin/env python3

from matplotlib.colors import PowerNorm
from sensor_msgs.msg import PointCloud2, PointField
import struct
import rospy
import numpy as np
import csv

class txtToPC2:

	def __init__(self):

		self.data = []
		self.pc2 = PointCloud2()
		self.pub = rospy.Publisher("test_pc2", PointCloud2, queue_size=10)

	def read_txt(self):

		csv_reader = csv.reader(open("/media/data/Final Pilot/45_facedown_merged.txt", 'r'), delimiter=',')
		for row in csv_reader:
			x = np.float32(row[0:3])
			# print(x)
			self.data.extend(list(x.view(np.uint8)))
			# print(x.view(np.uint8))

	def build_pointcloud2(self):

		self.pc2.data = self.data
		self.pc2.point_step = 12
		self.pc2.height = 1
		self.pc2.width = int(len(self.data)/12)
		self.pc2.is_bigendian = False
		self.pc2.is_dense = True
		self.pc2.row_step = 8*self.pc2.width
		self.pc2.header.stamp = rospy.Time.now()
		self.pc2.header.frame_id = "map"
		self.pc2.fields = [PointField("x", 0, 7, 1), PointField("y", 4, 7, 1), PointField("z", 8, 7, 1)]


	def publish_pointcloud2(self):

		self.pub.publish(self.pc2)
			

	def main(self):

		rate = rospy.Rate(1)

		self.read_txt()
		self.build_pointcloud2()
		while not rospy.is_shutdown():
			print("publishing")
			self.publish_pointcloud2()
			rate.sleep()
		# rospy.loginfo("Received pointcloud.")
			# self.process_pointcloud2(pointcloud)




if __name__ == "__main__":

	rospy.init_node('pointcloud_modifier')

	pointcloud = txtToPC2()
	pointcloud.main()
	# print("hey")
	# print(pointcloud.data)