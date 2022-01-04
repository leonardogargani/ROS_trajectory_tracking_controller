#ifndef EIGHT_TRAJ_GEN_H_
#define EIGHT_TRAJ_GEN_H_

#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include "trajectory_generator/GenerateDesiredPathService.h"

#define NAME_OF_THIS_NODE "eight_traj_gen"


class eight_traj_gen
{
  private:
    bool GenerateDesiredPath(trajectory_generator::GenerateDesiredPathService::Request &req,
                              trajectory_generator::GenerateDesiredPathService::Response &res);

  public:
    
};

#endif /* EIGHT_TRAJ_GEN_H_ */
