#include "unicycle_kin_sim/unicycle_kin_sim.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, NAME_OF_THIS_NODE);
    unicycle_kin_sim unicycle_kin_sim_node;

    unicycle_kin_sim_node.Prepare();
    unicycle_kin_sim_node.RunPeriodically();
    unicycle_kin_sim_node.Shutdown();

    return (0);
}
