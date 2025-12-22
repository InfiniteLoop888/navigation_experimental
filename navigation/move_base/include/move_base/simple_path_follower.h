/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2024
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
*   * Neither the name of the copyright holder nor the names of its
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
*********************************************************************/
#ifndef SIMPLE_PATH_FOLLOWER_H_
#define SIMPLE_PATH_FOLLOWER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <angles/angles.h>
#include <vector>
#include <dynamic_reconfigure/server.h>
#include <move_base/SimplePathFollowerConfig.h>

namespace move_base {

/**
 * @class SimplePathFollower
 * @brief A simple local planner that follows the global path by tracking waypoints
 */
class SimplePathFollower : public nav_core::BaseLocalPlanner {
public:
  /**
   * @brief Constructor
   */
  SimplePathFollower();

  /**
   * @brief Destructor
   */
  ~SimplePathFollower();

  /**
   * @brief Initializes the planner
   * @param name The name of this planner
   * @param tf A pointer to a transform listener
   * @param costmap_ros The cost map to use for assigning costs to local plans
   */
  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Set the plan that the local planner is following
   * @param plan The plan to pass to the local planner
   * @return True if the plan was updated successfully, false otherwise
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

  /**
   * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
   * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
   * @return True if a valid velocity command was found, false otherwise
   */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  /**
   * @brief Check if the goal pose has been achieved by the local planner
   * @return True if achieved, false otherwise
   */
  bool isGoalReached();

private:
  /**
   * @brief Get the current pose of the robot in the global frame
   * @param global_pose Will be filled with the current pose
   * @return True if successful, false otherwise
   */
  bool getRobotPose(geometry_msgs::PoseStamped& global_pose);

  /**
   * @brief Find the closest point on the path to the robot
   * @param robot_pose The current robot pose
   * @param plan The plan to search in
   * @return Index of the closest point, or -1 if path is empty
   */
  int findClosestPoint(const geometry_msgs::PoseStamped& robot_pose,
                       const std::vector<geometry_msgs::PoseStamped>& plan);

  /**
   * @brief Find the target point on the path ahead of the robot
   * @param robot_pose The current robot pose
   * @param closest_idx Index of the closest point
   * @param plan The plan to search in
   * @param target_pose Will be filled with the target pose
   * @return True if a valid target was found, false otherwise
   */
  bool findTargetPoint(const geometry_msgs::PoseStamped& robot_pose, int closest_idx,
                       const std::vector<geometry_msgs::PoseStamped>& plan,
                       geometry_msgs::PoseStamped& target_pose);

  /**
   * @brief Calculate distance between two poses
   */
  double distance(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2);

  /**
   * @brief Get yaw angle from a pose
   */
  double getYaw(const geometry_msgs::Pose& pose);

  /**
   * @brief Callback for dynamic reconfigure
   */
  void reconfigureCB(SimplePathFollowerConfig &config, uint32_t level);

  // Member variables
  bool initialized_;
  unsigned int last_closest_index_;
  bool rotating_to_goal_; // State variable for hysteresis
  std::string name_;
  tf2_ros::Buffer* tf_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  geometry_msgs::Twist last_cmd_vel_;
  
  // Dynamic reconfigure
  dynamic_reconfigure::Server<SimplePathFollowerConfig> *dsrv_;

  // Error handling
  int tf_failure_count_;
  
  // Parameters
  double max_vel_x_;           // Maximum linear velocity
  double max_vel_theta_;        // Maximum angular velocity
  double acc_lim_x_;            // Maximum linear acceleration
  double acc_lim_theta_;        // Maximum angular acceleration
  double lookahead_dist_;       // Lookahead distance for target point
  double goal_tolerance_;       // Goal tolerance distance
  double goal_tolerance_theta_; // Goal tolerance angle
  double kp_linear_;            // Linear velocity proportional gain
  double kp_angular_;           // Angular velocity proportional gain
  double min_vel_x_;            // Minimum linear velocity
};

} // namespace move_base

#endif // SIMPLE_PATH_FOLLOWER_H_

