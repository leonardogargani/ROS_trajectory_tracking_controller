#ifndef TEST_EIGHT_H_
#define TEST_EIGHT_H_

#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>

// used only if the actual value of the period is not retrieved from the ROS parameter server
#define RUN_PERIOD_DEFAULT 0.01

#define NAME_OF_THIS_NODE "test_eight"

class test_eight
{
private:
    ros::NodeHandle Handle;

    ros::Publisher vehicleCommand_publisher;

    void PeriodicTask(void);

    double xref, dxref, ddxref, yref, dyref, ddyref;
    double linear_velocity, angular_velocity;

public:
    double RunPeriod;
    void Prepare(void);
    void RunPeriodically(float Period);
    void Shutdown(void);
};

#endif /* TEST_EIGHT_H_ */
