#include "diffdrive_kin_ctrl/diffdrive_kin_trajctrl.h"

#include <unistd.h>


void diffdrive_kin_trajctrl::Prepare(void)
{

    // fetch parameters from parameters server
    std::string FullParamName;

    FullParamName = ros::this_node::getName() + "/run_period";
    if (false == Handle.getParam(FullParamName, RunPeriod))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/P_dist";
    if (false == Handle.getParam(FullParamName, P_dist))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/Kp";
    if (false == Handle.getParam(FullParamName, Kp))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/Ki";
    if (false == Handle.getParam(FullParamName, Ki))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/Kd";
    if (false == Handle.getParam(FullParamName, Kd))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = "/diffdrive_kin_sim/d";
    if (false == Handle.getParam(FullParamName, d))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = "/diffdrive_kin_sim/r";
    if (false == Handle.getParam(FullParamName, r))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // publishers
    vehicleCommand_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/robot_input", 1);
    controllerState_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/controller_state", 1);
    
    // subscribers
    vehicleState_subscriber = Handle.subscribe("/robot_state", 1, &diffdrive_kin_trajctrl::vehicleState_MessageCallback, this);

    // create the trajectory controller
    controller = new diffdrive_kin_fblin(P_dist);

	// service for trajectory generation
    client = Handle.serviceClient<diffdrive_kin_ctrl::GenerateDesiredPathService>("generate_desired_path_service");

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());

    // query the service server until it gives a response (while waiting, do nothing)
    while (!client.call(srv)) {
        ROS_INFO("Waiting for service");
    }

	ROS_INFO("Size in kin_trajctrl: %lu", srv.response.xref.size());
	ROS_INFO("Starting point -- x: %.2f -- y: %.2f", srv.response.xref[0], srv.response.yref[0]);

    // populate the reference vectore with the trajectory from the service response
    for (uint t = 0; t < srv.response.xref.size(); t++) {
        xref_vector.push_back(srv.response.xref[t]);
        yref_vector.push_back(srv.response.yref[t]);            
    }

    ROS_INFO("Path has been generated and received.");

}


void diffdrive_kin_trajctrl::RunPeriodically(float Period)
{
    ros::Rate LoopRate(1.0 / Period);

    ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0 / Period);

    while (ros::ok()) {
        PeriodicTask();
        ros::spinOnce();
        LoopRate.sleep();
    }
}


void diffdrive_kin_trajctrl::Shutdown(void)
{
    delete controller;
    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}


// set (x,y,theta) of the robot in the controller 
void diffdrive_kin_trajctrl::vehicleState_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    // input command: t, msg->data[0]; x, msg->data[1]; y, msg->data[2]; theta, msg->data[3];
    //                  linear velocity, msg->data[4]; angular velocity, msg->data[5]
    controller->set_diffdriveState(msg->data.at(1), msg->data.at(2), msg->data.at(3));
}


// compute (v,w) in order to follow the trajectory;
// to do so, use a PID controller and a linearization law
void diffdrive_kin_trajctrl::PeriodicTask(void)
{
    long unsigned int t_ns = ros::Time::now().toNSec();
    int t = (int)(t_ns / 10000000);

    // handle the end of the desired trajectory: in such a case kill the simulator
    if (t + 1 > xref_vector.size()) {
        ROS_INFO("Path vector totally consumed");
        system("rosnode kill diffdrive_kin_sim");
        ros::shutdown();
        return;
    }

    double xref = xref_vector[t];
    double yref = yref_vector[t];
    double dxref = (xref_vector[t + 1] - xref_vector[t]) / 1;
    double dyref = (yref_vector[t + 1] - yref_vector[t]) / 1;
    
    // compute the control action: transform trajectory to point P
    controller->reference_transformation(xref, yref, xPref, yPref);
    controller->output_transformation(xP, yP);
    
    double error_x = xPref - xP;
    double derivative_x = (error_x - prev_error_x) / RunPeriod;
    prev_error_x = error_x;
    
    double error_y = yPref - yP;
    double derivative_y = (error_y - prev_error_y) / RunPeriod;
    prev_error_y = error_y;
    
    // PID controller for trajectory tracking
    vPx = dxref + Kp * (xPref - xP) + Ki * (xPref - xP) * RunPeriod + derivative_x * Kd;
    vPy = dyref + Kp * (yPref - yP) + Ki * (yPref - yP) * RunPeriod + derivative_y * Kd;

    // linearization law
    controller->control_transformation(vPx, vPy, v, omega);

    // pass from unicycle to differential drive
    omega_r = (v + omega * d / 2) / r;
    omega_l = (v - omega * d / 2) / r;
    
    // publish vehicle commands as a message
    std_msgs::Float64MultiArray vehicleCommandMsg;
    vehicleCommandMsg.data.push_back(ros::Time::now().toSec());
    vehicleCommandMsg.data.push_back(omega_r);
    vehicleCommandMsg.data.push_back(omega_l);
    vehicleCommand_publisher.publish(vehicleCommandMsg);

    // publish controller state as a message
    std_msgs::Float64MultiArray controllerStateMsg;
    controllerStateMsg.data.push_back(ros::Time::now().toSec());
    controllerStateMsg.data.push_back(xref);
    controllerStateMsg.data.push_back(yref);
    controllerStateMsg.data.push_back(xPref);
    controllerStateMsg.data.push_back(yPref);
    controllerStateMsg.data.push_back(xP);
    controllerStateMsg.data.push_back(yP);
    controllerStateMsg.data.push_back(vPx);
    controllerStateMsg.data.push_back(vPy);
	controllerStateMsg.data.push_back(v);
	controllerStateMsg.data.push_back(omega);
    controllerStateMsg.data.push_back(omega_r);
    controllerStateMsg.data.push_back(omega_l);
    controllerState_publisher.publish(controllerStateMsg);
}
