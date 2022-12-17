#ifndef DIFFDRIVE_DWA_TRAJCTRL_H_
#define DIFFDRIVE_DWA_TRAJCTRL_H_

#include "ros/ros.h"
#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <dwa_local_planner/dwa_planner_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "diffdrive_kin_ctrl/GenerateDesiredPathService.h"
#include <std_msgs/Float64MultiArray.h>


#define NAME_OF_THIS_NODE "diffdrive_dwa_trajctrl"


class diffdrive_dwa_trajctrl
{
private: 
    ros::NodeHandle Handle;

    ros::Subscriber vehicleState_subscriber;
    ros::Publisher vehicleCommand_publisher;
    ros::Publisher controllerState_publisher;

    ros::ServiceClient client;
    diffdrive_kin_ctrl::GenerateDesiredPathService srv;

    // create twist message to be populated by the local planner
    geometry_msgs::Twist dwa_cmd_vel;
    
    geometry_msgs::PoseStamped l_global_pose;

    void PeriodicTask(void);

public:

    // parameters fetched from yaml
    float RunPeriod;
    int skipped_goals;
    float d, r;

    void Prepare(void);
    void RunPeriodically(float Period);
    void Shutdown(void);
};

#endif /* DIFFDRIVE_KIN_TRAJCTRL_H_ */
