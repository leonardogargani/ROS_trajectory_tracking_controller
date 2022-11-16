#include "diffdrive_dwa_ctrl/diffdrive_dwa_trajctrl.h"



void diffdrive_dwa_trajctrl::Prepare(void)
{

	std::string FullParamName;

	FullParamName = ros::this_node::getName() + "/run_period";
	if (false == Handle.getParam(FullParamName, RunPeriod))
		ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
		
	FullParamName = ros::this_node::getName() + "/skipped_goals";
	if (false == Handle.getParam(FullParamName, skipped_goals))
		ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
	
	FullParamName = "/diffdrive_kin_sim/d";
	if (false == Handle.getParam(FullParamName, d))
		ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
		
	FullParamName = "/diffdrive_kin_sim/r";
	if (false == Handle.getParam(FullParamName, r))
		ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

	tf2_ros::Buffer tfBuffer(ros::Duration(10));
	tf2_ros::TransformListener tfListener(tfBuffer);

    	costmap_2d::Costmap2DROS my_global_costmap("my_global_costmap", tfBuffer);
	my_global_costmap.start();
    	dwa_local_planner::DWAPlannerROS dp;
	dp.initialize("my_dwa_planner", &tfBuffer, &my_global_costmap);

	vehicleCommand_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/robot_input", 1);
	controllerState_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/controller_state", 1);

	client = Handle.serviceClient<diffdrive_kin_ctrl::GenerateDesiredPathService>("generate_desired_path_service");

	while (!client.call(srv)) {
		ROS_INFO("Waiting for service");
	}

	ROS_INFO("Path has been generated and received.");

	std::vector<geometry_msgs::PoseStamped> orig_global_plan;
	geometry_msgs::PoseStamped tmp_pose_stamped;

	for (uint t = 0; t < srv.response.xref.size(); t+=skipped_goals) {

		tmp_pose_stamped.pose.position.x = srv.response.xref[t];
		tmp_pose_stamped.pose.position.y = srv.response.yref[t];

		// yaw goal set to 0 (a value is required by DWA)
		tmp_pose_stamped.pose.orientation.x = 0.0;
		tmp_pose_stamped.pose.orientation.y = 0.0;
		tmp_pose_stamped.pose.orientation.z = 0.0;
		tmp_pose_stamped.pose.orientation.w = 1.0;

		tmp_pose_stamped.header.frame_id = "map";
		orig_global_plan.push_back(tmp_pose_stamped);

		if (dp.setPlan(orig_global_plan)) {
			ROS_INFO("DWA set plan: SUCCESS");
		} else {
			ROS_ERROR("DWA set plan: FAILED");
		}
		
		while(!dp.isGoalReached()) {

			ROS_INFO("Current goal (#%d): x=%f, y=%f", t, tmp_pose_stamped.pose.position.x, tmp_pose_stamped.pose.position.y);

			my_global_costmap.getRobotPose(l_global_pose);
			my_global_costmap.updateMap();

			ROS_INFO("Current position: x=%f, y=%f", l_global_pose.pose.position.x, l_global_pose.pose.position.y);

			// compute velocity commands using DWA
			if (dp.computeVelocityCommands(dwa_cmd_vel) == true) {
				ROS_INFO("DWA result: v=%.4f, w=%.4f", dwa_cmd_vel.linear.x, dwa_cmd_vel.angular.z);

			} else {
				ROS_ERROR("DWA compute cmd_vel: FAILED");
			}
			
			// Valori originali: prima di introdurre i parametri
			//float d = 0.15;
			//float r = 0.03;

			float omega_r = ((float)dwa_cmd_vel.linear.x + (float)dwa_cmd_vel.angular.z * d / 2.0) / r;
			float omega_l = ((float)dwa_cmd_vel.linear.x - (float)dwa_cmd_vel.angular.z * d / 2.0) / r;

			ROS_INFO("Velocities of wheels: w_r=%.4f, w_l=%.4f\n", omega_r, omega_l);

    		std_msgs::Float64MultiArray vehicleCommandMsg;
			vehicleCommandMsg.data.push_back(ros::Time::now().toSec());
			vehicleCommandMsg.data.push_back(omega_r);
			vehicleCommandMsg.data.push_back(omega_l);
			vehicleCommand_publisher.publish(vehicleCommandMsg);

			double xref = tmp_pose_stamped.pose.position.x;
			double yref = tmp_pose_stamped.pose.position.y;

			double x = l_global_pose.pose.position.x;
			double y = l_global_pose.pose.position.y;

			// publish controller state
			std_msgs::Float64MultiArray controllerStateMsg;
			controllerStateMsg.data.push_back(ros::Time::now().toSec()); // 0
			controllerStateMsg.data.push_back(xref); // 1
			controllerStateMsg.data.push_back(yref); // 2
			controllerStateMsg.data.push_back(xref);	// xPref // 3
			controllerStateMsg.data.push_back(yref);	// yPref // 4
			controllerStateMsg.data.push_back(x);		// xP // 5
			controllerStateMsg.data.push_back(y);		// yP // 6
			controllerStateMsg.data.push_back(0);		// vPx // 7
			controllerStateMsg.data.push_back(0);		// vPy // 8
			controllerStateMsg.data.push_back(dwa_cmd_vel.linear.x); // 9
			controllerStateMsg.data.push_back(dwa_cmd_vel.angular.z); // 10
			controllerStateMsg.data.push_back(omega_r); // 11
			controllerStateMsg.data.push_back(omega_l); // 12
			controllerState_publisher.publish(controllerStateMsg);

		}

		orig_global_plan.clear();
	}

}


void diffdrive_dwa_trajctrl::RunPeriodically(float Period)
{
	ros::Rate LoopRate(1.0 / Period);

	ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0 / Period);

	while (ros::ok()) {
		PeriodicTask();
		ros::spinOnce();
		LoopRate.sleep();
	}
}


void diffdrive_dwa_trajctrl::Shutdown(void)
{
	ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}


void diffdrive_dwa_trajctrl::PeriodicTask(void)
{
	// do nothing
}

