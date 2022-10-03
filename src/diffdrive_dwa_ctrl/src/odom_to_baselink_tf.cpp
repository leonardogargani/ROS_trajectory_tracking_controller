#include "diffdrive_dwa_ctrl/odom_to_baselink_tf.h"


void odom_to_baselink_tf::Prepare(void)
{
	
	std::string FullParamName;

    FullParamName = ros::this_node::getName() + "/run_period";
    if (false == Handle.getParam(FullParamName, RunPeriod))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    
	sub = Handle.subscribe("/odom", 1000, &odom_to_baselink_tf::Callback, this);	
	
}


void odom_to_baselink_tf::RunPeriodically(float Period)
{

    ros::Rate LoopRate(1.0 / Period);

    ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0 / Period);

    while (ros::ok()) {
    	PeriodicTask();
        ros::spinOnce();
        LoopRate.sleep();
    }

}


void odom_to_baselink_tf::Shutdown(void)
{
    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}


void odom_to_baselink_tf::Callback(const nav_msgs::Odometry::ConstPtr &msg)
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


void odom_to_baselink_tf::PeriodicTask()
{
	// do nothing
}

