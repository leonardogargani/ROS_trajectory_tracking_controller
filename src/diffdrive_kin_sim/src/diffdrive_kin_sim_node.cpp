#include "diffdrive_kin_sim/diffdrive_kin_sim.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, NAME_OF_THIS_NODE);
    diffdrive_kin_sim diffdrive_kin_sim_node;

    diffdrive_kin_sim_node.Prepare();
    diffdrive_kin_sim_node.RunPeriodically();
    diffdrive_kin_sim_node.Shutdown();

    return (0);
}
