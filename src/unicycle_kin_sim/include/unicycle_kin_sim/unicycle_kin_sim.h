#ifndef UNICYCLE_SIM_H_
#define UNICYCLE_SIM_H_

#include "ros/ros.h"
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Float64MultiArray.h>

#include "unicycle_kin_ode.h"

#define NAME_OF_THIS_NODE "unicycle_kin_sim"

class unicycle_kin_sim
{
private:
    ros::NodeHandle Handle;

    ros::Subscriber vehicleCommand_subscriber;
    ros::Publisher vehicleState_publisher;
    ros::Publisher clock_publisher;

    /// parameters from ROS parameter server
    double dt;
    double x0, y0, theta0;

    void vehicleCommand_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);

    void PeriodicTask(void);

    unicycle_kin_ode *simulator;

public:
    void Prepare(void);
    void RunPeriodically(void);
    void Shutdown(void);
};

#endif /* UNICYCLE_SIM_H_ */
