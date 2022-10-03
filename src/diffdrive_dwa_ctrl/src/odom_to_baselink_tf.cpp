
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class tf_sub_pub
{

private:
	ros::NodeHandle n;
	tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;
	ros::Subscriber sub;

public:
	tf_sub_pub()
	{
		sub = n.subscribe("/odom", 1000, &tf_sub_pub::callback, this);
	}

	void callback(const nav_msgs::Odometry::ConstPtr &msg)
	{
		transformStamped.header.stamp = ros::Time::now();
		transformStamped.header.frame_id = "odom";
		transformStamped.child_frame_id = "base_link";

		transformStamped.transform.translation.x = msg->pose.pose.position.x;
		transformStamped.transform.translation.y = msg->pose.pose.position.y;
		transformStamped.transform.translation.z = msg->pose.pose.position.z;

		transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
		transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
		transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
		transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;

		br.sendTransform(transformStamped);
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "odom_to_baselink_tf");
	tf_sub_pub my_tf_sub_bub;
	ros::spin();
	return 0;
}