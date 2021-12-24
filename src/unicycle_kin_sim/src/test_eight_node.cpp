#include "unicycle_kin_sim/test_eight.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, NAME_OF_THIS_NODE);
    test_eight test_eight_node;

    test_eight_node.Prepare();
    test_eight_node.RunPeriodically(test_eight_node.RunPeriod);
    test_eight_node.Shutdown();

    return (0);
}
