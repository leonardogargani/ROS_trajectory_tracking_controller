#ifndef DIFFDRIVE_KIN_TRAJCTRL_H_
#define DIFFDRIVE_KIN_TRAJCTRL_H_

#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include <diffdrive_kin_fblin.h>
#include "diffdrive_kin_ctrl/GenerateDesiredPathService.h"
#include <vector>

#define NAME_OF_THIS_NODE "diffdrive_kin_trajctrl"


class diffdrive_kin_trajctrl
{
private: 
    ros::NodeHandle Handle;

    ros::Subscriber vehicleState_subscriber;
    ros::Publisher vehicleCommand_publisher, controllerState_publisher;

    ros::ServiceClient client;
    diffdrive_kin_ctrl::GenerateDesiredPathService srv;

    // parameters from ROS parameter server
    double P_dist;
    double Kp;
    double Ki;
    double Kd;
    double d;
    double r;

    diffdrive_kin_fblin* controller;
    double xP, yP, xPref, yPref;
    double vPx, vPy, v, omega, omega_r, omega_l;

    std::vector<double> xref_vector, yref_vector, dxref_vector, dyref_vector; 
    double prev_error_x = 0;
    double prev_error_y = 0;

    void vehicleState_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void PeriodicTask(void);

public:

    float RunPeriod;

    void Prepare(void);
    void RunPeriodically(float Period);
    void Shutdown(void);

};

#endif /* DIFFDRIVE_KIN_TRAJCTRL_H_ */
