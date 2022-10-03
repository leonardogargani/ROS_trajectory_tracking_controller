#include "diffdrive_dwa_ctrl/diffdrive_dwa_trajctrl.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, NAME_OF_THIS_NODE);
    diffdrive_dwa_trajctrl diffdrive_dwa_trajctrl_node;

    diffdrive_dwa_trajctrl_node.Prepare();
    diffdrive_dwa_trajctrl_node.RunPeriodically(diffdrive_dwa_trajctrl_node.RunPeriod);
    diffdrive_dwa_trajctrl_node.Shutdown();

    return (0);
}

