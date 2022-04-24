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
        
    costmap_2d::Costmap2DROS my_costmap("my_costmap", tfBuffer);

    my_costmap.start();

    dwa_local_planner::DWAPlannerROS dp;
    dp.initialize("my_dwa_planner", &tfBuffer, &my_costmap);
    
    
    /*
     * Come usare DWA con un nostra path come reference (dal service).
     * 
    1) Ottieni path di riferimento chiamando servizio eight_trajgen e trasfomalo in un vettore di PoseStamped.
    	1.a) Passa il path al planner usando setPlan().
    		 setPlan() prende come argomento un vettore di PoseStamped.
    
    2) Crea variabile di tipo PoseStamped e Twist.
       PoseStamped va riempito con position, orientation e velocity correnti.
       Twist sarà riempito con le nuove velocità --> sia lineare che angolare.
       2.a) (forse) dividi la omega in omega_dx e omega_sx.
    
    3) Chiama dwaComputeVelocityCommands() passandogli i parametri appena dichiarati.
    
    4) Usa velocità ottenute per calcolare il nuovo stato [x, y, theta] come diffdrive_kin_ode.
    
    5) Ripeti da punto 2 fino a fine path.
       La fine del path forse la si trova con isGoalReached().
    */
    

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

    geometry_msgs::PoseStamped l_global_pose;
    my_costmap.getRobotPose(l_global_pose);

    while(!dp.isGoalReached()) {

        // update global costmap
        my_costmap.updateMap();

        // compute velocity commands using DWA
        if (dp.dwaComputeVelocityCommands(l_global_pose, dwa_cmd_vel)) {
            ROS_INFO("DWA compute cmd_vel: SUCCESS");
        } else {
            ROS_ERROR("DWA compute cmd_vel: FAILED");
        }

    }


    // dwaComputeVelocityCommands -> odom_helper_.getRobotVel (line 196)
 




    return (0);

}
