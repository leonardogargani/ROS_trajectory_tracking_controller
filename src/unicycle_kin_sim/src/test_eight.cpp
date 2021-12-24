#include "unicycle_kin_sim/test_eight.h"

void test_eight::Prepare(void)
{
    vehicleCommand_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/robot_input", 1);
    RunPeriod = RUN_PERIOD_DEFAULT;
    linear_velocity = angular_velocity = 0.0;
    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void test_eight::RunPeriodically(float Period)
{
    ros::Rate LoopRate(1.0 / Period);

    ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0 / Period);

    while (ros::ok()) {
        PeriodicTask();
        ros::spinOnce();
        LoopRate.sleep();
    }
}

void test_eight::Shutdown(void)
{
    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void test_eight::PeriodicTask(void)
{
    // 8-shaped trajectory generation
    // trajectory parameters (these parameters should be moved to the parameter server)
    const double a = 1.0;
    const double w = 1.0;

    // trajectory computation
    xref    = a * std::sin(w * ros::Time::now().toSec());
    dxref   = w * a * std::cos(w * ros::Time::now().toSec());
    ddxref  = - std::pow(w, 2.0) * a * std::sin(w * ros::Time::now().toSec());
    yref    = a * std::sin(w * ros::Time::now().toSec()) * std::cos(w * ros::Time::now().toSec());
    dyref   = w * a * (std::pow(std::cos(w * ros::Time::now().toSec()), 2.0) 
                        - std::pow(std::sin(w * ros::Time::now().toSec()), 2.0));
    ddyref  = - 4.0 * std::pow(w, 2.0) * a * std::sin(w * ros::Time::now().toSec()) * std::cos(w * ros::Time::now().toSec());

    // flatness transformation
    linear_velocity  = std::sqrt(std::pow(dxref,2.0)+std::pow(dyref,2.0));
    angular_velocity = (dxref*ddyref-dyref*ddxref)/(std::pow(dxref,2.0)+std::pow(dyref,2.0));

    // publishing vehicle commands (t, msg->data[0]; velocity, msg->data[1]; steer, msg->data[2])
    std_msgs::Float64MultiArray msg;
    msg.data.push_back(ros::Time::now().toSec());
    msg.data.push_back(linear_velocity);
    msg.data.push_back(angular_velocity);
    vehicleCommand_publisher.publish(msg);
}
