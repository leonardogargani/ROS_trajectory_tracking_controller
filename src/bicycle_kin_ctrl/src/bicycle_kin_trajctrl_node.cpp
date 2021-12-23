#include "bicycle_kin_ctrl/bicycle_kin_trajctrl.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  bicycle_kin_trajctrl bicycle_kin_trajctrl_node;
   
  bicycle_kin_trajctrl_node.Prepare();
  
  bicycle_kin_trajctrl_node.RunPeriodically(bicycle_kin_trajctrl_node.RunPeriod);
  
  bicycle_kin_trajctrl_node.Shutdown();
  
  return (0);
}

