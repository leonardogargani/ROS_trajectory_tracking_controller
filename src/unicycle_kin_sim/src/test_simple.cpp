#include "unicycle_kin_sim/test_simple.h"

void test_simple::Prepare(void)
{
    vehicleCommand_publisher = Handle.advertise<std_msgs::Float64MultiArray>("/robot_input", 1);
    RunPeriod = RUN_PERIOD_DEFAULT;
    linear_velocity = angular_velocity = 0.0;
    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void test_simple::RunPeriodically(float Period)
{
    ros::Rate LoopRate(1.0 / Period);

    ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0 / Period);

    while (ros::ok()) {
        PeriodicTask();
        ros::spinOnce();
        LoopRate.sleep();
    }
}

void test_simple::Shutdown(void)
{
    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void test_simple::PeriodicTask(void)
{
    if (ros::Time::now().toSec() <= 5.0) {
        linear_velocity = 1.0;
        angular_velocity = 0.0;
    } else {
        linear_velocity = 0.0;
        angular_velocity = 0.5;
    }

    // publishing vehicle commands (t, msg->data[0]; velocity, msg->data[1]; steer, msg->data[2])
    std_msgs::Float64MultiArray msg;
    msg.data.push_back(ros::Time::now().toSec());
    msg.data.push_back(linear_velocity);
    msg.data.push_back(angular_velocity);
    vehicleCommand_publisher.publish(msg);
}
