#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


class bobcatFootprintBroadcaster
{
private:

	ros::NodeHandle n;

	tf::TransformListener* tfListener;
	tf::TransformBroadcaster* tfBroadcaster;
	tf::StampedTransform mapToBobcatBase;

public:

	bobcatFootprintBroadcaster()
	{
		// Initialize ROS
		n = ros::NodeHandle();

		tfListener = new tf::TransformListener;
		tfBroadcaster = new tf::TransformBroadcaster;

		// wait to start receiving valid tf transforms:
		// lookupTransform will go through errors until a valid chain has been found from target to source frames

		bool tf_error = true;
		ROS_INFO("waiting for tf between the required frames...");

		ros::Rate tryRate(2);

		while (tf_error) 
		{
			tf_error = false;

			try 
			{
				tfListener->lookupTransform("map", "bobcat_base", ros::Time(0), mapToBobcatBase);
			}

			catch(tf::TransformException &exception) 
			{
				ROS_WARN("%s Retrying...", exception.what());

				tf_error = true;

				ros::spinOnce();
				tryRate.sleep();
			}   
		}

		ROS_INFO("Received map->bobcat_base transform!");
	}


	void run()
	{
		ros::Rate loop_rate(10);

		while(ros::ok())
		{
			tfListener->lookupTransform("map", "bobcat_base", ros::Time(0), mapToBobcatBase);

			tf::Transform tf(mapToBobcatBase.getBasis(),mapToBobcatBase.getOrigin());

			tf::Quaternion tfQuat; // tf library object for quaternion
			tf::Vector3 tfVec;

		  	tfQuat = tf.getRotation();
		  	tfVec = tf.getOrigin();

		    tf::Matrix3x3 rotationMatrix(tfQuat);
		    double roll, pitch, yaw;
		    rotationMatrix.getRPY(roll, pitch, yaw);

		    tfQuat.setRPY(-roll, -pitch, yaw);
		    tf::Transform tf2(tfQuat, tf::Vector3(0,0,0));

		    tf::StampedTransform finalTransform(tf2, ros::Time::now(), "bobcat_base", "base_footprint");    
			tfBroadcaster->sendTransform(finalTransform);

			ros::spinOnce();
			loop_rate.sleep();
		}
	}
};


int main(int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "listens_bobcat_base_broadcasts_bobcat_footprint");

	// Create our bobcatFootprintBroadcaster() object and run it
	auto broadcaster = bobcatFootprintBroadcaster();
	broadcaster.run();

	return 0;
}