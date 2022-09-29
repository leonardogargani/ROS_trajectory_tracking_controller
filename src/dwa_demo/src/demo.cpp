#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <dwa_local_planner/dwa_planner_ros.h>

#include <geometry_msgs/PoseStamped.h>
#include "diffdrive_kin_ctrl/GenerateDesiredPathService.h"

#include <std_msgs/Float64MultiArray.h>

geometry_msgs::PoseStamped gl_l_global_pose;

void odometry_MessageCallback(const nav_msgs::Odometry::ConstPtr& odometry_msg){
	
	::gl_l_global_pose.header = odometry_msg->header;
	::gl_l_global_pose.pose = odometry_msg->pose.pose;
	
	ROS_INFO("CALLBACK -- X: %.4f -- Y: %.4f", gl_l_global_pose.pose.position.x, gl_l_global_pose.pose.position.y);
	
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "dwa_demo");

    tf2_ros::Buffer tfBuffer(ros::Duration(10));
    tf2_ros::TransformListener tfListener(tfBuffer);
        
    //costmap_2d::Costmap2DROS my_local_costmap("my_local_costmap", tfBuffer);
    costmap_2d::Costmap2DROS my_global_costmap("my_global_costmap", tfBuffer);

    //my_local_costmap.start();
    my_global_costmap.start();

    dwa_local_planner::DWAPlannerROS dp;
    dp.initialize("my_dwa_planner", &tfBuffer, &my_global_costmap);
    
    ros::ServiceClient client;
    diffdrive_kin_ctrl::GenerateDesiredPathService srv;
    ros::NodeHandle Handle;
    
    ros::Publisher vehicleCommand_publisher;
    vehicleCommand_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/robot_input", 1);
    
    ros::Subscriber odometry_subscriber;
    odometry_subscriber = Handle.subscribe("/odom", 1, &odometry_MessageCallback);
    
    client = Handle.serviceClient<diffdrive_kin_ctrl::GenerateDesiredPathService>("generate_desired_path_service");
    
    while (!client.call(srv)) {
        ROS_INFO("Waiting for service");
    }
    
    ROS_INFO("DEMO.CPP -> Path has been generated and received.");

    std::vector<geometry_msgs::PoseStamped> orig_global_plan;
    geometry_msgs::PoseStamped tmp_pose_stamped;

    for (uint t = 0; t < srv.response.xref.size(); t+=100) {
    //for (uint t = 500; t < 1500; t++) {
        tmp_pose_stamped.pose.position.x = srv.response.xref[t];
        tmp_pose_stamped.pose.position.y = srv.response.yref[t];
        //tmp_pose_stamped.pose.position.x = -3.0;
        //tmp_pose_stamped.pose.position.y = 0.0;
        
        tmp_pose_stamped.header.frame_id = "map";  // base_link odom map
        orig_global_plan.push_back(tmp_pose_stamped);
    //}

		if (dp.setPlan(orig_global_plan)) {
		    ROS_INFO("DWA set plan: SUCCESS");
		} else {
		    ROS_ERROR("DWA set plan: FAILED");
		}

		// create twist message to be populated by the local planner
		geometry_msgs::Twist dwa_cmd_vel;
		geometry_msgs::PoseStamped l_global_pose;


        float xy_dist = 999.99;

		while(xy_dist > 0.15) {
		//while(!dp.isGoalReached()) {

		    ROS_INFO("--> GOAL #%d: %f, %f", t, tmp_pose_stamped.pose.position.x, tmp_pose_stamped.pose.position.y);

		    my_global_costmap.getRobotPose(l_global_pose);

            xy_dist = sqrt(pow((tmp_pose_stamped.pose.position.x - l_global_pose.pose.position.x), 2.0)
                            + pow((tmp_pose_stamped.pose.position.y - l_global_pose.pose.position.y), 2.0));
            
            ROS_INFO("xy_dist: %f", xy_dist);


		    // update global costmap
		    //my_local_costmap.updateMap();
		    my_global_costmap.updateMap();

		    // compute velocity commands using DWA
		    //if (dp.dwaComputeVelocityCommands(l_global_pose, dwa_cmd_vel) == true) {
		    if (dp.computeVelocityCommands(dwa_cmd_vel) == true) {
		        ROS_INFO("DWA compute cmd_vel: SUCCESS");
		    } else {
		        ROS_ERROR("DWA compute cmd_vel: FAILED");
		    }
		    
		    
		    ROS_INFO("DWA_CMD_X (lin_x): %.4f | DWA_CMD_Z (ang_z): %.4f | (v_y (lin_y) = %.4f, v_z (lin_z) = %.4f)",
		        dwa_cmd_vel.linear.x, dwa_cmd_vel.angular.z, dwa_cmd_vel.linear.y, dwa_cmd_vel.linear.z);
		   
		   	float d = 0.15;
		  	float r = 0.03;

		  	float omega_r = ((float)dwa_cmd_vel.linear.x + (float)dwa_cmd_vel.angular.z * d / 2.0) / r;
		  	float omega_l = ((float)dwa_cmd_vel.linear.x - (float)dwa_cmd_vel.angular.z * d / 2.0) / r;
			
			ROS_INFO("OMEGA_R: %.4f --- OMEGA_L: %.4f", omega_r, omega_l);

		  	std_msgs::Float64MultiArray vehicleCommandMsg;
		  	vehicleCommandMsg.data.push_back(ros::Time::now().toSec());
		  	vehicleCommandMsg.data.push_back(omega_r);
		  	vehicleCommandMsg.data.push_back(omega_l);
		  	vehicleCommand_publisher.publish(vehicleCommandMsg);
		  	
		  	//orig_global_plan.clear();

			//ros::spinOnce();
			//usleep(1000);
						
			//dp.setCurrentPose(gl_l_global_pose);
		}

        xy_dist = 999.99;

		ROS_WARN("\n");
		ROS_WARN("\n");
		ROS_WARN("---- NEW ITERATION ----");
		
		orig_global_plan.clear();
    }

    return (0);

}
