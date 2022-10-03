#ifndef ODOM_TO_BASELINK_TF_H_
#define ODOM_TO_BASELINK_TF_H_

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#define NAME_OF_THIS_NODE "odom_to_baselink_tf"

class odom_to_baselink_tf
{
	private:
		ros::NodeHandle Handle;
		tf2_ros::TransformBroadcaster br;
		geometry_msgs::TransformStamped transformStamped;
		ros::Subscriber sub;
		
		void PeriodicTask();
		
	public:
		float RunPeriod;
		void Prepare(void);
		void RunPeriodically(float Period);
		void Shutdown(void);
		void Callback(const nav_msgs::Odometry::ConstPtr &msg);

};

#endif /* ODOM_TO_BASELINK_TF_H_ */