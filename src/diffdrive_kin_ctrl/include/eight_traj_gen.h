#ifndef EIGHT_TRAJ_GEN_H_
#define EIGHT_TRAJ_GEN_H_

#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include "diffdrive_kin_ctrl/GenerateDesiredPathService.h"

#define NAME_OF_THIS_NODE "eight_traj_gen"


class eight_traj_gen
{
  private:
    bool GenerateDesiredPath(diffdrive_kin_ctrl::GenerateDesiredPathService::Request &req,
                              diffdrive_kin_ctrl::GenerateDesiredPathService::Response &res);

  public:
    
};

#endif /* EIGHT_TRAJ_GEN_H_ */
