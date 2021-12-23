#ifndef UNICYCLE_KIN_LINCTRL_H_
#define UNICYCLE_KIN_LINCTRL_H_

#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include <unicycle_kin_fblin.h>

#define NAME_OF_THIS_NODE "unicycle_kin_linctrl"


class unicycle_kin_linctrl
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Subscriber vehicleState_subscriber;
    ros::Publisher vehicleCommand_publisher, controllerState_publisher;

    /* Parameters from ROS parameter server */
    double P_dist;

    /* ROS topic callbacks */
    void vehicleState_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

    /* Node periodic task */
    void PeriodicTask(void);
    
    /* Node state variables */
    unicycle_kin_fblin* controller;

  public:
    float RunPeriod;

    void Prepare(void);
    
    void RunPeriodically(float Period);
    
    void Shutdown(void);

};

#endif /* UNICYCLE_KIN_LINCTRL_H_ */
