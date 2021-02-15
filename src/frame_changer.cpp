#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <tf/transform_listener.h>
#include "pcl_ros/transforms.h"


class frameChanger
{
private:

	ros::NodeHandle n;

	ros::Publisher pointcloud_pub;
	ros::Subscriber pointcloud_sub;

	sensor_msgs::PointCloud2 inputPointCloud;
	sensor_msgs::PointCloud2 outputPointCloud;

	tf::TransformListener* tfListener;

	bool newPointCloudReceived = false;


	void LIDARCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
	{
		newPointCloudReceived = true;
		inputPointCloud = *msg;
	}


public:

	frameChanger()
	{
		// Initialize ROS
		n = ros::NodeHandle();

		tfListener = new tf::TransformListener;

		// Create a publisher object, able to push messages
		pointcloud_pub = n.advertise<sensor_msgs::PointCloud2>("decimated_pointclouds_bobcat_base", 5);

		// Create a subscriber for laser scans 
		pointcloud_sub = n.subscribe("decimated_pointclouds", 10, &frameChanger::LIDARCallback, this); // decimated_pointclouds

		// wait to start receiving valid tf transforms:
		// lookupTransform will go through errors until a valid chain has been found from target to source frames

		bool tf_error = true;
		ROS_INFO("waiting for tf between the required frames...");

		tf::StampedTransform bobcatBaseToMap;
		ros::Rate tryRate(2);

		while (tf_error) 
		{
			tf_error = false;

			try 
			{
				//try to lookup the map -> bobcat_base transform; this will test if
				// a valid transform chain has been published from bobcat_base to map
				tfListener->lookupTransform("bobcat_base", "map", ros::Time(0), bobcatBaseToMap);
			}

			catch(tf::TransformException &exception) 
			{
				ROS_WARN("%s Retrying...", exception.what());

				tf_error = true;

				ros::spinOnce();
				tryRate.sleep();
			}   
		}

		ROS_INFO("tf is good");
		// from now on, tfListener will keep track of transforms; do NOT need ros::spin(), since
		// tf_listener gets spawned as a separate thread
	}


	void run()
	{
		ros::Rate loop_rate(10);

		while (ros::ok())
		{
			if (newPointCloudReceived)
			{
				pcl_ros::transformPointCloud("base_footprint", inputPointCloud, outputPointCloud, *tfListener);

				pointcloud_pub.publish(outputPointCloud);

				newPointCloudReceived = false;
			}

			ros::spinOnce();

			// And throttle the loop
			loop_rate.sleep();
		}
	}
};


int main(int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "frame_changer");

	// Create our frameChanger object and run it
	auto frame_changer = frameChanger();
	frame_changer.run();

	return 0;
}