#include "unicycle_kin_ctrl/unicycle_kin_trajctrl.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  unicycle_kin_trajctrl unicycle_kin_trajctrl_node;
   
  unicycle_kin_trajctrl_node.Prepare();
  
  unicycle_kin_trajctrl_node.RunPeriodically(unicycle_kin_trajctrl_node.RunPeriod);
  
  unicycle_kin_trajctrl_node.Shutdown();
  
  return (0);
}

