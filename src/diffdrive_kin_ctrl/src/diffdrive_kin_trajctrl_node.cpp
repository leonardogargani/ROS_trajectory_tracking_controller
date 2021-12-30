#include "diffdrive_kin_ctrl/diffdrive_kin_trajctrl.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  diffdrive_kin_trajctrl diffdrive_kin_trajctrl_node;
   
  diffdrive_kin_trajctrl_node.Prepare();
  diffdrive_kin_trajctrl_node.RunPeriodically(diffdrive_kin_trajctrl_node.RunPeriod);
  diffdrive_kin_trajctrl_node.Shutdown();
  
  return (0);
}

