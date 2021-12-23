#include "bicycle_kin_ctrl/bicycle_kin_linctrl.h"

#include <unistd.h>


void bicycle_kin_linctrl::Prepare(void)
{
    /* Retrieve parameters from ROS parameter server */
    std::string FullParamName;

    // run_period
    FullParamName = ros::this_node::getName()+"/run_period";
    if (false == Handle.getParam(FullParamName, RunPeriod))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    // Controller parameters
    FullParamName = ros::this_node::getName()+"/P_dist";
    if (false == Handle.getParam(FullParamName, P_dist))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    FullParamName = ros::this_node::getName()+"/l";
    if (false == Handle.getParam(FullParamName, l))
        ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

    /* ROS topics */
    vehicleState_subscriber = Handle.subscribe("/car_state", 1, &bicycle_kin_linctrl::vehicleState_MessageCallback, this);
    vehicleCommand_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/car_input", 1);
    controllerState_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/controller_state", 1);

    /* Create controller class */
    controller = new bicycle_kin_fblin(P_dist);

    // Initialize controller parameters
    controller->set_bicycleParam(l);

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void bicycle_kin_linctrl::RunPeriodically(float Period)
{
    ros::Rate LoopRate(1.0/Period);

    ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);

    while (ros::ok())
    {
        PeriodicTask();

        ros::spinOnce();

        LoopRate.sleep();
    }
}

void bicycle_kin_linctrl::Shutdown(void)
{
    // Delete controller object
    delete controller;

    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void bicycle_kin_linctrl::vehicleState_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    // Input command: t, msg->data[0]; x, msg->data[1]; y, msg->data[2]; theta, msg->data[3];
    // linear velocity, msg->data[4]; angular velocity, msg->data[5]
    /*  Set vehicle state */
    controller->set_bicycleState(msg->data.at(1), msg->data.at(2), msg->data.at(3));
}

void bicycle_kin_linctrl::PeriodicTask(void)
{
    /*  Generate input commands */
    double vPx, vPy;
    if (ros::Time::now().toSec()<=5.0)
    {
        vPx = 1.0;
        vPy = 0.0;
    }
    else if (ros::Time::now().toSec()<=10.0)
    {
        vPx = 2.0;
        vPy = 0.0;
    }
    else
    {
        vPx = 2.0;
        vPy = 2.0;
    }

    /*  Compute the control action */
    double v, phi;
    controller->control_transformation(vPx, vPy, v, phi);;

    /*  Publish vehicle commands */
    std_msgs::Float64MultiArray vehicleCommandMsg;
    vehicleCommandMsg.data.push_back(ros::Time::now().toSec());
    vehicleCommandMsg.data.push_back(v);
    vehicleCommandMsg.data.push_back(phi);
    vehicleCommand_publisher.publish(vehicleCommandMsg);

    /*  Publish controller state */
    std_msgs::Float64MultiArray controllerStateMsg;
    controllerStateMsg.data.push_back(ros::Time::now().toSec());
    controllerStateMsg.data.push_back(vPx);
    controllerStateMsg.data.push_back(vPy);
    controllerStateMsg.data.push_back(v);
    controllerStateMsg.data.push_back(phi);
    controllerState_publisher.publish(controllerStateMsg);
}
