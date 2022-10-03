#include "diffdrive_dwa_ctrl/odom_to_baselink_tf.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, NAME_OF_THIS_NODE);
    odom_to_baselink_tf odom_to_baselink_tf_node;

    odom_to_baselink_tf_node.Prepare();
    odom_to_baselink_tf_node.RunPeriodically(odom_to_baselink_tf_node.RunPeriod);
    odom_to_baselink_tf_node.Shutdown();

    return (0);
}
