#include "bicycle_kin_ctrl/bicycle_kin_linctrl.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  bicycle_kin_linctrl bicycle_kin_linctrl_node;
   
  bicycle_kin_linctrl_node.Prepare();
  
  bicycle_kin_linctrl_node.RunPeriodically(bicycle_kin_linctrl_node.RunPeriod);
  
  bicycle_kin_linctrl_node.Shutdown();
  
  return (0);
}

