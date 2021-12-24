#include "unicycle_kin_sim/test_simple.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  test_simple test_simple_node;
   
  test_simple_node.Prepare();
  test_simple_node.RunPeriodically(test_simple_node.RunPeriod);
  test_simple_node.Shutdown();
  
  return (0);
}

