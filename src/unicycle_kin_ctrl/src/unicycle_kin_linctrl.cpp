#include "unicycle_kin_ctrl/unicycle_kin_linctrl.h"

#include <unistd.h>


void unicycle_kin_linctrl::Prepare(void)
{
    std::string FullParamName;

    FullParamName = ros::this_node::getName() + "/run_period";
    if (false == Handle.getParam(FullParamName, RunPeriod))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/P_dist";
    if (false == Handle.getParam(FullParamName, P_dist))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    vehicleState_subscriber = Handle.subscribe("/robot_state", 1, &unicycle_kin_linctrl::vehicleState_MessageCallback, this);
    vehicleCommand_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/robot_input", 1);
    controllerState_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/controller_state", 1);

    controller = new unicycle_kin_fblin(P_dist);

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void unicycle_kin_linctrl::RunPeriodically(float Period)
{
    ros::Rate LoopRate(1.0 / Period);

    ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0 / Period);

    while (ros::ok()) {
        PeriodicTask();
        ros::spinOnce();
        LoopRate.sleep();
    }
}

void unicycle_kin_linctrl::Shutdown(void)
{
    delete controller;
    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void unicycle_kin_linctrl::vehicleState_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    // input command: t, msg->data[0]; x, msg->data[1]; y, msg->data[2]; theta, msg->data[3];
    //                  linear velocity, msg->data[4]; angular velocity, msg->data[5]
    controller->set_unicycleState(msg->data.at(1), msg->data.at(2), msg->data.at(3));
}

void unicycle_kin_linctrl::PeriodicTask(void)
{
    // generate input commands
    double vPx, vPy;
    if (ros::Time::now().toSec() <= 5.0) {
        vPx = 1.0;
        vPy = 0.0;
    } else if (ros::Time::now().toSec() <= 10.0) {
        vPx = 2.0;
        vPy = 0.0;
    } else {
        vPx = 2.0;
        vPy = 2.0;
    }

    // compute the control action
    double v, omega;
    controller->control_transformation(vPx, vPy, v, omega);;

    // publish vehicle commands
    std_msgs::Float64MultiArray vehicleCommandMsg;
    vehicleCommandMsg.data.push_back(ros::Time::now().toSec());
    vehicleCommandMsg.data.push_back(v);
    vehicleCommandMsg.data.push_back(omega);
    vehicleCommand_publisher.publish(vehicleCommandMsg);

    // publish controller state
    std_msgs::Float64MultiArray controllerStateMsg;
    controllerStateMsg.data.push_back(ros::Time::now().toSec());
    controllerStateMsg.data.push_back(vPx);
    controllerStateMsg.data.push_back(vPy);
    controllerStateMsg.data.push_back(v);
    controllerStateMsg.data.push_back(omega);
    controllerState_publisher.publish(controllerStateMsg);
}
