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
 #ifndef DWA_LOCAL_PLANNER_DWA_PLANNER_H_
 #define DWA_LOCAL_PLANNER_DWA_PLANNER_H_
 
 #include <vector>
 #include <Eigen/Core>
 
 
 #include <dwa_local_planner/DWAPlannerConfig.h>
 
 //for creating a local cost grid
 #include <base_local_planner/map_grid_visualizer.h>
 
 //for obstacle data access
 #include <costmap_2d/costmap_2d.h>
 
 #include <base_local_planner/trajectory.h>
 #include <base_local_planner/local_planner_limits.h>
 #include <base_local_planner/local_planner_util.h>
 #include <base_local_planner/simple_trajectory_generator.h>
 
 #include <base_local_planner/oscillation_cost_function.h>
 #include <base_local_planner/map_grid_cost_function.h>
 #include <base_local_planner/obstacle_cost_function.h>
 #include <base_local_planner/twirling_cost_function.h>
 #include <base_local_planner/simple_scored_sampling_planner.h>
 
 #include <nav_msgs/Path.h>
 
 namespace dwa_local_planner {
   class DWAPlanner {
     public:
       DWAPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util);
 
       void reconfigure(DWAPlannerConfig &cfg);
 
       bool checkTrajectory(
           const Eigen::Vector3f pos,
           const Eigen::Vector3f vel,
           const Eigen::Vector3f vel_samples);
 
       base_local_planner::Trajectory findBestPath(
           const geometry_msgs::PoseStamped& global_pose,
           const geometry_msgs::PoseStamped& global_vel,
           geometry_msgs::PoseStamped& drive_velocities);
 
       void updatePlanAndLocalCosts(const geometry_msgs::PoseStamped& global_pose,
           const std::vector<geometry_msgs::PoseStamped>& new_plan,
           const std::vector<geometry_msgs::Point>& footprint_spec);
 
       double getSimPeriod() { return sim_period_; }
 
       bool getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost);
 
       bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
 
     private:
 
       base_local_planner::LocalPlannerUtil *planner_util_;
 
       double stop_time_buffer_; 
       double path_distance_bias_, goal_distance_bias_, occdist_scale_;
       Eigen::Vector3f vsamples_;
 
       double sim_period_;
       base_local_planner::Trajectory result_traj_;
 
       double forward_point_distance_;
 
       std::vector<geometry_msgs::PoseStamped> global_plan_;
 
       boost::mutex configuration_mutex_;
       std::string frame_id_;
       ros::Publisher traj_cloud_pub_;
       bool publish_cost_grid_pc_; 
       bool publish_traj_pc_;
 
       double cheat_factor_;
 
       base_local_planner::MapGridVisualizer map_viz_; 
 
       // see constructor body for explanations
       base_local_planner::SimpleTrajectoryGenerator generator_;
       base_local_planner::OscillationCostFunction oscillation_costs_;
       base_local_planner::ObstacleCostFunction obstacle_costs_;
       base_local_planner::MapGridCostFunction path_costs_;
       base_local_planner::MapGridCostFunction goal_costs_;
       base_local_planner::MapGridCostFunction goal_front_costs_;
       base_local_planner::MapGridCostFunction alignment_costs_;
       base_local_planner::TwirlingCostFunction twirling_costs_;
 
       base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_;
   };
 };
 #endif
