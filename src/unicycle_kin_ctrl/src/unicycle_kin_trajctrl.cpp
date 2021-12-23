#include "unicycle_kin_ctrl/unicycle_kin_trajctrl.h"

#include <unistd.h>


void unicycle_kin_trajctrl::Prepare(void)
{
    std::string FullParamName;

    FullParamName = ros::this_node::getName() + "/run_period";
    if (false == Handle.getParam(FullParamName, RunPeriod))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/P_dist";
    if (false == Handle.getParam(FullParamName, P_dist))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName() + "/K";
    if (false == Handle.getParam(FullParamName, K))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    vehicleState_subscriber = Handle.subscribe("/robot_state", 1, &unicycle_kin_trajctrl::vehicleState_MessageCallback, this);
    vehicleCommand_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/robot_input", 1);
    controllerState_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/controller_state", 1);

    controller = new unicycle_kin_fblin(P_dist);

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void unicycle_kin_trajctrl::RunPeriodically(float Period)
{
    ros::Rate LoopRate(1.0 / Period);

    ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0 / Period);

    while (ros::ok()) {
        PeriodicTask();
        ros::spinOnce();
        LoopRate.sleep();
    }
}

void unicycle_kin_trajctrl::Shutdown(void)
{
    delete controller;
    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void unicycle_kin_trajctrl::vehicleState_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    // input command: t, msg->data[0]; x, msg->data[1]; y, msg->data[2]; theta, msg->data[3];
    //                  linear velocity, msg->data[4]; angular velocity, msg->data[5]
    controller->set_unicycleState(msg->data.at(1), msg->data.at(2), msg->data.at(3));
}

void unicycle_kin_trajctrl::PeriodicTask(void)
{
    // 8-shaped trajectory generation
    // trajectory parameters (these parameters should be moved to the parameter server)
    const double a = 1.0;
    const double w = 1.0;

    // trajectory computation
    xref = a * std::sin(w * ros::Time::now().toSec());
    dxref = w * a * std::cos(w * ros::Time::now().toSec());
    yref = a * std::sin(w * ros::Time::now().toSec()) * std::cos(w * ros::Time::now().toSec());
    dyref = w * a * (std::pow(std::cos(w * ros::Time::now().toSec()), 2.0) 
                        - std::pow(std::sin(w * ros::Time::now().toSec()), 2.0));

    // compute the control action
    // transform trajectory to point P
    controller->reference_transformation(xref, yref, xPref, yPref);
    controller->output_transformation(xP, yP);

    // trajectory tracking law
    vPx = dxref + K * (xPref - xP);
    vPy = dyref + K * (yPref - yP);

    // linearization law
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
    controllerState_publisher.publish(controllerStateMsg);
}
