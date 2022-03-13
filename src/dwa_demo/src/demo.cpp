#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <dwa_local_planner/dwa_planner_ros.h>


int main(int argc, char **argv)
{

    ros::init(argc, argv, "dwa_demo");

    std::cout << "hello" << std::endl;

    // ...

    tf2_ros::Buffer tfBuffer(ros::Duration(10));
    tf2_ros::TransformListener tfListener(tfBuffer);
    costmap_2d::Costmap2DROS costmap("my_costmap", tfBuffer);

    dwa_local_planner::DWAPlannerROS dp;
    dp.initialize("my_dwa_planner", &tfBuffer, &costmap);

    return (0);

}
