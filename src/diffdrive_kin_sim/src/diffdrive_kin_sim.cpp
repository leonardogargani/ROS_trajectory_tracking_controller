#include "diffdrive_kin_sim/diffdrive_kin_sim.h"

#include <unistd.h>


void diffdrive_kin_sim::Prepare(void)
{
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

    vehicleCommand_subscriber = Handle.subscribe("/robot_input", 1, &diffdrive_kin_sim::vehicleCommand_MessageCallback, this);
    vehicleState_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/robot_state", 1);
    clock_publisher = Handle.advertise<rosgraph_msgs::Clock>("/clock", 1);

    simulator = new diffdrive_kin_ode(dt);
    simulator->setInitialState(x0, y0, theta0);

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void diffdrive_kin_sim::RunPeriodically(void)
{
    ROS_INFO("Node %s running.", ros::this_node::getName().c_str());

    // wait other nodes start
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

void diffdrive_kin_sim::PeriodicTask(void)
{
    simulator->integrate();

    double x, y, theta;
    simulator->getPose(x, y, theta);

    double omega_r_act, omega_l_act;
    simulator->getCommands(omega_r_act, omega_l_act);

    double time;
    simulator->getTime(time);

    // print simulation time every 5 sec
    if (std::fabs(std::fmod(time, 5.0)) < 1.0e-3)
    {
        ROS_INFO("Simulator time: %d seconds", (int) time);
    }

    // publish vehicle state
    std_msgs::Float64MultiArray vehicleStateMsg;
    vehicleStateMsg.data.push_back(time);
    vehicleStateMsg.data.push_back(x);
    vehicleStateMsg.data.push_back(y);
    vehicleStateMsg.data.push_back(theta);
    vehicleStateMsg.data.push_back(omega_r_act);
    vehicleStateMsg.data.push_back(omega_l_act);
    vehicleState_publisher.publish(vehicleStateMsg);

    // publish clock
    rosgraph_msgs::Clock clockMsg;
    clockMsg.clock = ros::Time(time);
    clock_publisher.publish(clockMsg);
}
