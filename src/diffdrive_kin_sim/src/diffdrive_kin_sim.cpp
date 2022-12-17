#include "diffdrive_kin_sim/diffdrive_kin_sim.h"
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>

#include <unistd.h>


void diffdrive_kin_sim::Prepare(void)
{

    // fetch parameters from parameters server
    std::string FullParamName;

    FullParamName = ros::this_node::getName() + "/dt";
    if (false == Handle.getParam(FullParamName, dt))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/x0";
    if (false == Handle.getParam(FullParamName, x0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/y0";
    if (false == Handle.getParam(FullParamName, y0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/theta0";
    if (false == Handle.getParam(FullParamName, theta0))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
    
    FullParamName = ros::this_node::getName() + "/d";
	if (false == Handle.getParam(FullParamName, d))
		ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
		
	FullParamName = ros::this_node::getName() + "/r";
	if (false == Handle.getParam(FullParamName, r))
		ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // publishers
    vehicleState_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/robot_state", 1);
    clock_publisher = Handle.advertise<rosgraph_msgs::Clock>("/clock", 1);
    odom_publisher = Handle.advertise<nav_msgs::Odometry>("/odom", 1);

    // subscribers
    vehicleCommand_subscriber = Handle.subscribe("/robot_input", 1, &diffdrive_kin_sim::vehicleCommand_MessageCallback, this);
    
    // create and initialize the simulator
    simulator = new diffdrive_kin_ode(dt);
    simulator->setInitialState(x0, y0, theta0);
    simulator->setRobotDimensions(d, r);

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}


void diffdrive_kin_sim::RunPeriodically(void)
{
    ROS_INFO("Node %s running.", ros::this_node::getName().c_str());

    // wait other nodes to start
    sleep(1.0);

    while (ros::ok()) {
        PeriodicTask();
        ros::spinOnce();
        usleep(1000);
    }
}


void diffdrive_kin_sim::Shutdown(void)
{
    delete simulator;
    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}


void diffdrive_kin_sim::vehicleCommand_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    // input command: t, msg->data[0]; right angular velocity, msg->data[1]; left angular velocity, msg->data[2]
    simulator->setReferenceCommands(msg->data.at(1), msg->data.at(2));
}


// 
void diffdrive_kin_sim::PeriodicTask(void)
{

    // perform an integration step and fetch the new state
    simulator->integrate();
    double x, y, theta;
    simulator->getPose(x, y, theta);

    // fetch (w_r,w_l) of the robot
    double omega_r_act, omega_l_act;
    simulator->getCommands(omega_r_act, omega_l_act);

    double time;
    simulator->getTime(time);

    // print simulation time every 5 seconds
    if (std::fabs(std::fmod(time, 5.0)) < 1.0e-3) {
        ROS_INFO("Simulator time: %d seconds", (int) time);
    }

    // publish vehicle state as a message
    std_msgs::Float64MultiArray vehicleStateMsg;
    vehicleStateMsg.data.push_back(time);
    vehicleStateMsg.data.push_back(x);
    vehicleStateMsg.data.push_back(y);
    vehicleStateMsg.data.push_back(theta);
    vehicleStateMsg.data.push_back(omega_r_act);
    vehicleStateMsg.data.push_back(omega_l_act);
    vehicleState_publisher.publish(vehicleStateMsg);

    // compute (v,w) from (w_r,w_l)
    double v_r = omega_r_act * r;
    double v_l = omega_l_act * r;
    double v = (v_r + v_l) / 2.0;
    double w = (v_r - v_l) / d;

    // transform theta into a quaternion
    tf2::Quaternion quaternion; 
    quaternion.setRPY(0, 0, theta); 
    quaternion = quaternion.normalize();

    // publish an Odometry message on the odom topic (to accommodate DWA needs)
    nav_msgs::Odometry odometry_message;
    odometry_message.header.frame_id = "odom";
    odometry_message.child_frame_id = "base_link";
    odometry_message.header.stamp = ros::Time(time);
    odometry_message.pose.pose.position.x = x;
    odometry_message.pose.pose.position.y = y;
    odometry_message.pose.pose.orientation.x = quaternion.x();
    odometry_message.pose.pose.orientation.y = quaternion.y();
    odometry_message.pose.pose.orientation.z = quaternion.z();
    odometry_message.pose.pose.orientation.w = quaternion.w();
    odometry_message.twist.twist.linear.x = v;
    odometry_message.twist.twist.angular.z = w;
    odom_publisher.publish(odometry_message);

    // publish clock
    rosgraph_msgs::Clock clockMsg;
    clockMsg.clock = ros::Time(time);
    clock_publisher.publish(clockMsg);
}
