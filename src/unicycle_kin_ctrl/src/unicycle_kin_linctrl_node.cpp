#include "unicycle_kin_ctrl/unicycle_kin_linctrl.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  unicycle_kin_linctrl unicycle_kin_linctrl_node;
   
  unicycle_kin_linctrl_node.Prepare();
  
  unicycle_kin_linctrl_node.RunPeriodically(unicycle_kin_linctrl_node.RunPeriod);
  
  unicycle_kin_linctrl_node.Shutdown();
  
  return (0);
}

