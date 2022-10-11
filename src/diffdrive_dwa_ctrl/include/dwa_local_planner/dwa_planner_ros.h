/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *********************************************************************/
 #ifndef DWA_LOCAL_PLANNER_DWA_PLANNER_ROS_H_
 #define DWA_LOCAL_PLANNER_DWA_PLANNER_ROS_H_
 
 #include <boost/shared_ptr.hpp>
 #include <boost/thread.hpp>
 
 #include <tf2_ros/buffer.h>
 
 #include <dynamic_reconfigure/server.h>
 #include <dwa_local_planner/DWAPlannerConfig.h>
 
 #include <angles/angles.h>
 
 #include <nav_msgs/Odometry.h>
 
 #include <costmap_2d/costmap_2d_ros.h>
 #include <nav_core/base_local_planner.h>
 #include <base_local_planner/latched_stop_rotate_controller.h>
 
 #include <base_local_planner/odometry_helper_ros.h>
 
 #include <dwa_local_planner/dwa_planner.h>
 
 namespace dwa_local_planner {
   class DWAPlannerROS : public nav_core::BaseLocalPlanner {
     public:
       DWAPlannerROS();
 
       void initialize(std::string name, tf2_ros::Buffer* tf,
           costmap_2d::Costmap2DROS* costmap_ros);
 
       ~DWAPlannerROS();
 
       bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
 
 
       bool dwaComputeVelocityCommands(geometry_msgs::PoseStamped& global_pose, geometry_msgs::Twist& cmd_vel);
 
       bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
 
       bool isGoalReached();
       
       bool isInitialized() {
         return initialized_;
       }
 
     private:
       void reconfigureCB(DWAPlannerConfig &config, uint32_t level);
 
       void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);
 
       void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);
 
       tf2_ros::Buffer* tf_; 
 
       // for visualisation, publishers of global and local plan
       ros::Publisher g_plan_pub_, l_plan_pub_;
 
       base_local_planner::LocalPlannerUtil planner_util_;
 
       boost::shared_ptr<DWAPlanner> dp_; 
 
       costmap_2d::Costmap2DROS* costmap_ros_;
 
       dynamic_reconfigure::Server<DWAPlannerConfig> *dsrv_;
       dwa_local_planner::DWAPlannerConfig default_config_;
       bool setup_;
       geometry_msgs::PoseStamped current_pose_;
 
       base_local_planner::LatchedStopRotateController latchedStopRotateController_;
 
 
       bool initialized_;
 
 
       base_local_planner::OdometryHelperRos odom_helper_;
       std::string odom_topic_;
   };
 };
 #endif
