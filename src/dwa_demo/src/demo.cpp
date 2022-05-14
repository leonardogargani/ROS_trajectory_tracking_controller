#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <dwa_local_planner/dwa_planner_ros.h>

#include <geometry_msgs/PoseStamped.h>
#include "diffdrive_kin_ctrl/GenerateDesiredPathService.h"


int main(int argc, char **argv)
{

    ros::init(argc, argv, "dwa_demo");

    tf2_ros::Buffer tfBuffer(ros::Duration(10));
    tf2_ros::TransformListener tfListener(tfBuffer);
        
    costmap_2d::Costmap2DROS my_local_costmap("my_local_costmap", tfBuffer);
    costmap_2d::Costmap2DROS my_global_costmap("my_global_costmap", tfBuffer);

    my_local_costmap.start();
    my_global_costmap.start();

    dwa_local_planner::DWAPlannerROS dp;
    dp.initialize("my_dwa_planner", &tfBuffer, &my_local_costmap);
    
    ros::ServiceClient client;
    diffdrive_kin_ctrl::GenerateDesiredPathService srv;
    ros::NodeHandle Handle;
    
    client = Handle.serviceClient<diffdrive_kin_ctrl::GenerateDesiredPathService>("generate_desired_path_service");
    
    while (!client.call(srv)) {
        ROS_INFO("Waiting for service");
    }
    
    ROS_INFO("DEMO.CPP -> Path has been generated and received.");

    std::vector<geometry_msgs::PoseStamped> orig_global_plan;
    geometry_msgs::PoseStamped tmp_pose_stamped;

    for (uint t = 0; t < srv.response.xref.size(); t++) {
        tmp_pose_stamped.pose.position.x = srv.response.xref[t];
        tmp_pose_stamped.pose.position.y = srv.response.yref[t];

        tmp_pose_stamped.header.frame_id = "base_link";
        orig_global_plan.push_back(tmp_pose_stamped);
    }

    if (dp.setPlan(orig_global_plan)) {
        ROS_INFO("DWA set plan: SUCCESS");
    } else {
        ROS_ERROR("DWA set plan: FAILED");
    }

    // create twist message to be populated by the local planner
    geometry_msgs::Twist dwa_cmd_vel;
    dwa_cmd_vel.angular.x = -1;
    dwa_cmd_vel.angular.y = -1;
    dwa_cmd_vel.angular.z = -1;
    dwa_cmd_vel.linear.x = -1;
    dwa_cmd_vel.linear.y = -1;
    dwa_cmd_vel.linear.z = -1;
    geometry_msgs::PoseStamped l_global_pose;

    while(!dp.isGoalReached()) {

        ROS_INFO("pt.2a");
        my_global_costmap.getRobotPose(l_global_pose);

        // update global costmap
        my_local_costmap.updateMap();
        my_global_costmap.updateMap();

        ROS_INFO("pt.2b");

        // compute velocity commands using DWA
        //if (dp.dwaComputeVelocityCommands(l_global_pose, dwa_cmd_vel) == true) {
        if (dp.computeVelocityCommands(dwa_cmd_vel) == true) {
            ROS_INFO("DWA compute cmd_vel: SUCCESS");
        } else {
            ROS_ERROR("DWA compute cmd_vel: FAILED");
        }
        

        // https://answers.ros.org/question/376753/dwa-local-planner-as-a-stand-alone-c-library/
        // rosrun rqt_logger_level rqt_logger_level


        /*
            linear: 
            x: 0
            y: 0.1
            z: 0
            angular: 
            x: 0
            y: 0
            z: 0.126316
        */
        //std::cout << dwa_cmd_vel << std::endl;
        /*
            pose: 
            position: 
                x: 0
                y: 0
                z: 0
            orientation: 
                x: 0
                y: 0
                z: 0
                w: 1
        */
        //std::cout << l_global_pose << std::endl;

        // it seems to work => we just need to send dwa_cmd_vel to the simulator to actually make the robot move

    }

    ROS_INFO("pt.3");

    // dwaComputeVelocityCommands -> odom_helper_.getRobotVel (line 196)

    return (0);

}
