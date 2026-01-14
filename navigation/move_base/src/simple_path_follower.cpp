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
#include <move_base/simple_path_follower.h>
#include <pluginlib/class_list_macros.hpp>
#include <cmath>
#include <algorithm>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// Register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(move_base::SimplePathFollower, nav_core::BaseLocalPlanner)

namespace move_base {

SimplePathFollower::SimplePathFollower()
  : initialized_(false), last_closest_index_(0), rotating_to_goal_(false), locked_goal_yaw_(0.0), tf_(NULL), costmap_ros_(NULL),
    max_vel_x_(0.5), max_vel_theta_(1.0), acc_lim_x_(0.5), acc_lim_theta_(1.0), lookahead_dist_(0.5),
    goal_tolerance_(0.1), goal_tolerance_theta_(0.1),
    kp_linear_(1.0), kp_angular_(2.0), min_vel_x_(0.05),
    dsrv_(NULL), tf_failure_count_(0)
{
  last_cmd_vel_.linear.x = 0.0;
  last_cmd_vel_.angular.z = 0.0;
  last_cmd_time_ = ros::Time::now();
  rotation_start_time_ = ros::Time::now();
}

SimplePathFollower::~SimplePathFollower()
{
  if (dsrv_)
    delete dsrv_;
}

void SimplePathFollower::reconfigureCB(SimplePathFollowerConfig &config, uint32_t level)
{
  max_vel_x_ = config.max_vel_x;
  min_vel_x_ = config.min_vel_x;
  max_vel_theta_ = config.max_vel_theta;
  acc_lim_x_ = config.acc_lim_x;
  acc_lim_theta_ = config.acc_lim_theta;
  lookahead_dist_ = config.lookahead_dist;
  goal_tolerance_ = config.goal_tolerance;
  goal_tolerance_theta_ = config.goal_tolerance_theta;
  kp_linear_ = config.kp_linear;
  kp_angular_ = config.kp_angular;
  
  ROS_INFO("SimplePathFollower: Reconfigured parameters");
}

void SimplePathFollower::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (initialized_)
  {
    ROS_WARN("SimplePathFollower already initialized, doing nothing.");
    return;
  }

  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  
  // Load parameters
  ros::NodeHandle private_nh("~/" + name);
  private_nh.param("max_vel_x", max_vel_x_, max_vel_x_);
  private_nh.param("max_vel_theta", max_vel_theta_, max_vel_theta_);
  private_nh.param("lookahead_dist", lookahead_dist_, lookahead_dist_);
  private_nh.param("goal_tolerance", goal_tolerance_, goal_tolerance_);
  private_nh.param("goal_tolerance_theta", goal_tolerance_theta_, goal_tolerance_theta_);
  private_nh.param("kp_linear", kp_linear_, kp_linear_);
  private_nh.param("kp_angular", kp_angular_, kp_angular_);
  private_nh.param("min_vel_x", min_vel_x_, min_vel_x_);
  private_nh.param("acc_lim_x", acc_lim_x_, acc_lim_x_);
  private_nh.param("acc_lim_theta", acc_lim_theta_, acc_lim_theta_);

  // Initialize dynamic reconfigure
  dsrv_ = new dynamic_reconfigure::Server<SimplePathFollowerConfig>(private_nh);
  dynamic_reconfigure::Server<SimplePathFollowerConfig>::CallbackType cb = boost::bind(&SimplePathFollower::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  ROS_INFO("SimplePathFollower initialized with parameters:");
  ROS_INFO("  max_vel_x: %.2f", max_vel_x_);
  ROS_INFO("  max_vel_theta: %.2f", max_vel_theta_);
  ROS_INFO("  acc_lim_x: %.2f", acc_lim_x_);
  ROS_INFO("  acc_lim_theta: %.2f", acc_lim_theta_);
  ROS_INFO("  lookahead_dist: %.2f", lookahead_dist_);
  ROS_INFO("  goal_tolerance: %.2f", goal_tolerance_);
  ROS_INFO("  kp_linear: %.2f", kp_linear_);
  ROS_INFO("  kp_angular: %.2f", kp_angular_);

  initialized_ = true;
}

bool SimplePathFollower::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_)
  {
    ROS_ERROR("SimplePathFollower has not been initialized");
    return false;
  }

  // If we're in rotation mode and the plan is for the same goal area, 
  // keep rotation mode but update the goal orientation
  bool was_rotating = rotating_to_goal_;
  geometry_msgs::PoseStamped old_goal;
  if (was_rotating && !global_plan_.empty())
  {
    old_goal = global_plan_.back();
  }

  global_plan_ = plan;
  last_closest_index_ = 0;
  
  // If we were rotating and new plan has a goal, check if it's the same area
  if (was_rotating && !plan.empty())
  {
    geometry_msgs::PoseStamped new_goal = plan.back();
    double goal_dist = std::sqrt(
      std::pow(old_goal.pose.position.x - new_goal.pose.position.x, 2) +
      std::pow(old_goal.pose.position.y - new_goal.pose.position.y, 2)
    );
    
    // If goal moved significantly, reset rotation mode
    if (goal_dist > goal_tolerance_ * 2.0)
    {
      rotating_to_goal_ = false;
      ROS_INFO("SimplePathFollower: Goal moved significantly (%.3f m), resetting rotation mode", goal_dist);
    }
    else
    {
      // Update locked goal yaw to new goal orientation
      locked_goal_yaw_ = getYaw(new_goal.pose);
      ROS_INFO("SimplePathFollower: Goal position similar, updating goal orientation in rotation mode");
    }
  }
  else
  {
    rotating_to_goal_ = false;
  }
  
  ROS_INFO("SimplePathFollower: Received new plan with %zu waypoints", plan.size());
  return true;
}

bool SimplePathFollower::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_)
  {
    ROS_ERROR("SimplePathFollower has not been initialized");
    return false;
  }

  if (global_plan_.empty())
  {
    ROS_WARN_THROTTLE(1.0, "SimplePathFollower: Global plan is empty");
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    return false;
  }

  // Get current robot pose in global frame
  geometry_msgs::PoseStamped robot_pose;
  if (!getRobotPose(robot_pose))
  {
    tf_failure_count_++;
    if (tf_failure_count_ > 5)
    {
      ROS_WARN_THROTTLE(1.0, "SimplePathFollower: Could not get robot pose for %d consecutive cycles", tf_failure_count_);
    }
    
    // Safety: stop if we can't localize for too long
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    return false;
  }
  tf_failure_count_ = 0; // Reset counter on success

  // Transform global plan to the global frame if needed
  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  std::string global_frame = costmap_ros_->getGlobalFrameID();
  
  // Optimization: Start from the last known closest point
  unsigned int start_idx = last_closest_index_;
  if (start_idx >= global_plan_.size())
  {
    start_idx = 0;
  }

  for (size_t i = start_idx; i < global_plan_.size(); ++i)
  {
    const auto& pose = global_plan_[i];
    geometry_msgs::PoseStamped transformed_pose;
    try
    {
      if (pose.header.frame_id != global_frame)
      {
        tf_->transform(pose, transformed_pose, global_frame);
      }
      else
      {
        transformed_pose = pose;
      }
      transformed_plan.push_back(transformed_pose);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE(1.0, "SimplePathFollower: Failed to transform pose: %s", ex.what());
      continue;
    }
  }

  if (transformed_plan.empty())
  {
    ROS_WARN_THROTTLE(1.0, "SimplePathFollower: Transformed plan is empty");
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    return false;
  }

  // Find closest point on path
  int closest_idx = findClosestPoint(robot_pose, transformed_plan);
  if (closest_idx < 0 || closest_idx >= (int)transformed_plan.size())
  {
    ROS_WARN_THROTTLE(1.0, "SimplePathFollower: Could not find closest point on path");
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    return false;
  }

  // Update last_closest_index_
  unsigned int current_global_idx = start_idx + closest_idx;
  if (current_global_idx > last_closest_index_)
  {
    last_closest_index_ = current_global_idx;
  }

  // Get goal pose
  geometry_msgs::PoseStamped goal_pose = transformed_plan.back();
  double dist_to_goal = distance(robot_pose, goal_pose);
  
  // Find target point ahead
  geometry_msgs::PoseStamped target_pose;
  bool is_goal = false;
  if (!findTargetPoint(robot_pose, closest_idx, transformed_plan, target_pose))
  {
    // If we can't find a target ahead, use the goal
    target_pose = goal_pose;
    is_goal = true;
  }

  // Calculate distance and direction to target
  double dx = target_pose.pose.position.x - robot_pose.pose.position.x;
  double dy = target_pose.pose.position.y - robot_pose.pose.position.y;
  double dist = std::sqrt(dx * dx + dy * dy);
  
  double robot_yaw = getYaw(robot_pose.pose);
  double desired_yaw;
  double angle_error;
  
  // Hysteresis for rotating to goal orientation
  if (!rotating_to_goal_)
  {
    if (dist_to_goal < goal_tolerance_)
    {
      rotating_to_goal_ = true;
      locked_goal_yaw_ = getYaw(goal_pose.pose);
      rotation_start_time_ = ros::Time::now();
      ROS_INFO("SimplePathFollower: Reached goal tolerance, switching to rotation mode. Target yaw: %.3f rad (%.1f deg)", 
               locked_goal_yaw_, locked_goal_yaw_ * 180.0 / M_PI);
    }
  }
  else
  {
    // Hysteresis: only switch back to moving if we moved significantly away from goal
    if (dist_to_goal > goal_tolerance_ * 1.5)
    {
      rotating_to_goal_ = false;
      ROS_INFO("SimplePathFollower: Moved away from goal, switching to path following mode");
    }
    else
    {
      // Update locked goal yaw if goal orientation changed significantly (but position didn't)
      double current_goal_yaw = getYaw(goal_pose.pose);
      double goal_yaw_diff = std::abs(angles::shortest_angular_distance(locked_goal_yaw_, current_goal_yaw));
      // Only update if goal orientation changed by more than 30 degrees
      if (goal_yaw_diff > M_PI / 6.0)
      {
        locked_goal_yaw_ = current_goal_yaw;
        rotation_start_time_ = ros::Time::now();
        ROS_INFO("SimplePathFollower: Goal orientation changed significantly, updating locked yaw to %.3f rad (%.1f deg)",
                 locked_goal_yaw_, locked_goal_yaw_ * 180.0 / M_PI);
      }
    }
  }

  if (rotating_to_goal_)
  {
    // Use locked goal orientation to avoid oscillation from path replanning
    desired_yaw = locked_goal_yaw_;
    angle_error = angles::shortest_angular_distance(robot_yaw, desired_yaw);
  }
  else
  {
    // Calculate desired heading (direction from robot to target)
    desired_yaw = std::atan2(dy, dx);
    angle_error = angles::shortest_angular_distance(robot_yaw, desired_yaw);
  }

  // Calculate velocity commands using simple P control
  double linear_vel = 0.0;
  double angular_vel = 0.0;

  // Check if we're at goal orientation
  bool at_goal_orientation = (std::abs(angle_error) < goal_tolerance_theta_);
  
  if (rotating_to_goal_)
  {
    if (at_goal_orientation)
    {
      // We are at goal position AND orientation (or very close)
      // Just stop
      linear_vel = 0.0;
      angular_vel = 0.0;
    }
    else
    {
      // We are at goal position but need to rotate
      linear_vel = 0.0;
      
      // Improved rotation control with adaptive velocity based on angle error
      // Use proportional control with adaptive gain
      double abs_error = std::abs(angle_error);
      
      // Calculate base angular velocity
      angular_vel = kp_angular_ * angle_error;
      
      // Adaptive max velocity based on angle error
      // For large errors (>90 deg), use full max velocity
      // For medium errors (30-90 deg), use moderate velocity
      // For small errors (<30 deg), use reduced velocity to avoid overshooting
      double max_rot_vel = max_vel_theta_;
      if (abs_error > M_PI / 2.0)
      {
        // Large error: use full speed
        max_rot_vel = max_vel_theta_;
      }
      else if (abs_error > M_PI / 6.0)
      {
        // Medium error: use 70% of max speed
        max_rot_vel = max_vel_theta_ * 0.7;
      }
      else
      {
        // Small error: use 40% of max speed to avoid overshooting
        max_rot_vel = max_vel_theta_ * 0.4;
      }
      
      // Ensure minimum velocity for very small errors to avoid getting stuck
      if (abs_error > 0.01 && abs_error < goal_tolerance_theta_ * 2.0)
      {
        double min_rot_vel = 0.1; // Minimum rotation velocity
        if (std::abs(angular_vel) < min_rot_vel)
        {
          angular_vel = (angle_error > 0) ? min_rot_vel : -min_rot_vel;
        }
      }
      
      angular_vel = std::max(-max_rot_vel, std::min(angular_vel, max_rot_vel));
      
      // Check if we've been rotating for too long without progress (possible oscillation)
      double rotation_duration = (ros::Time::now() - rotation_start_time_).toSec();
      if (rotation_duration > 10.0) // 10 seconds
      {
        ROS_WARN_THROTTLE(2.0, "SimplePathFollower: Rotating for %.1f seconds, angle error: %.3f rad (%.1f deg). "
                        "Possible oscillation or goal orientation issue.", 
                        rotation_duration, abs_error, abs_error * 180.0 / M_PI);
      }
    }
  }
  else
  {
    // Normal path following behavior
    
    // If angle error is large, rotate in place first
    if (std::abs(angle_error) > M_PI / 2.0)
    {
      linear_vel = 0.0;
      angular_vel = kp_angular_ * angle_error;
    }
    else
    {
      // Move forward and turn
      linear_vel = kp_linear_ * dist; // Use full distance to target, not dist_to_goal
      
      // Reduce linear velocity based on angle error
      double angle_factor = std::max(0.0, 1.0 - (std::abs(angle_error) / (M_PI / 2.0)));
      linear_vel *= angle_factor;
      
      angular_vel = kp_angular_ * angle_error;
    }

    // Limit velocities
    linear_vel = std::max(0.0, std::min(linear_vel, max_vel_x_));
    angular_vel = std::max(-max_vel_theta_, std::min(angular_vel, max_vel_theta_));

    // Special case: very close to target point but not goal (e.g. sharp corner)
    if (dist < 0.1)
    {
       // If we are very close to intermediate target, we might want to slow down or just keep going
       // The original logic stopped here which might be wrong for intermediate points
       // But if target is goal, we handled it with rotating_to_goal_
    }
  }
  
  // Final limit on angular velocity
  angular_vel = std::max(-max_vel_theta_, std::min(angular_vel, max_vel_theta_));

  // Apply acceleration limits using actual time difference
  ros::Time current_time = ros::Time::now();
  double dt = (current_time - last_cmd_time_).toSec();
  
  // Clamp dt to reasonable bounds (avoid division by zero or very large values)
  // Typical control frequency is 10-20Hz, so dt should be 0.05-0.1s
  // If dt is too small or too large, use a default value
  if (dt <= 0.0 || dt > 1.0)
  {
    dt = 0.1;  // Default to 10Hz if time is invalid
  }
  
  // Calculate maximum allowed velocity change based on acceleration limits
  double max_acc_linear = acc_lim_x_ * dt;
  double max_acc_angular = acc_lim_theta_ * dt;
  
  // Clamp linear velocity change
  if (linear_vel > last_cmd_vel_.linear.x + max_acc_linear)
  {
    linear_vel = last_cmd_vel_.linear.x + max_acc_linear;
  }
  else if (linear_vel < last_cmd_vel_.linear.x - max_acc_linear)
  {
    linear_vel = last_cmd_vel_.linear.x - max_acc_linear;
  }
  
  // Clamp angular velocity change
  if (angular_vel > last_cmd_vel_.angular.z + max_acc_angular)
  {
    angular_vel = last_cmd_vel_.angular.z + max_acc_angular;
  }
  else if (angular_vel < last_cmd_vel_.angular.z - max_acc_angular)
  {
    angular_vel = last_cmd_vel_.angular.z - max_acc_angular;
  }

  cmd_vel.linear.x = linear_vel;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = angular_vel;
  
  // Store for next cycle
  last_cmd_vel_ = cmd_vel;
  last_cmd_time_ = current_time;

  return true;
}

bool SimplePathFollower::isGoalReached()
{
  if (!initialized_)
  {
    ROS_ERROR("SimplePathFollower has not been initialized");
    return false;
  }

  if (global_plan_.empty())
  {
    return false;
  }

  // Get current robot pose
  geometry_msgs::PoseStamped robot_pose;
  if (!getRobotPose(robot_pose))
  {
    return false;
  }

  // Transform goal to global frame if needed
  geometry_msgs::PoseStamped goal = global_plan_.back();
  std::string global_frame = costmap_ros_->getGlobalFrameID();
  
  if (goal.header.frame_id != global_frame)
  {
    try
    {
      tf_->transform(goal, goal, global_frame);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE(1.0, "SimplePathFollower: Failed to transform goal: %s", ex.what());
      return false;
    }
  }
  
  // Check distance to goal
  double dist = distance(robot_pose, goal);
  
  // Check if robot orientation matches goal orientation
  double goal_yaw = getYaw(goal.pose);
  double robot_yaw = getYaw(robot_pose.pose);
  double angle_error = std::abs(angles::shortest_angular_distance(robot_yaw, goal_yaw));

  if (dist < goal_tolerance_ && angle_error < goal_tolerance_theta_)
  {
    ROS_INFO("SimplePathFollower: Goal reached! Distance: %.3f m, Angle error: %.3f rad (%.1f deg)", 
             dist, angle_error, angle_error * 180.0 / M_PI);
    return true;
  }

  // Debug output
  ROS_INFO_THROTTLE(1.0, "SimplePathFollower: Not at goal. Distance: %.3f m (tolerance: %.3f), "
                  "Angle error: %.3f rad (%.1f deg, tolerance: %.3f rad)", 
                  dist, goal_tolerance_, angle_error, angle_error * 180.0 / M_PI, goal_tolerance_theta_);

  return false;
}

bool SimplePathFollower::getRobotPose(geometry_msgs::PoseStamped& global_pose)
{
  if (!tf_ || !costmap_ros_)
  {
    return false;
  }

  geometry_msgs::PoseStamped robot_pose;
  robot_pose.header.frame_id = costmap_ros_->getBaseFrameID();
  robot_pose.header.stamp = ros::Time();
  robot_pose.pose.position.x = 0.0;
  robot_pose.pose.position.y = 0.0;
  robot_pose.pose.position.z = 0.0;
  robot_pose.pose.orientation.x = 0.0;
  robot_pose.pose.orientation.y = 0.0;
  robot_pose.pose.orientation.z = 0.0;
  robot_pose.pose.orientation.w = 1.0;

  try
  {
    tf_->transform(robot_pose, global_pose, costmap_ros_->getGlobalFrameID());
    return true;
  }
  catch (tf2::LookupException& ex)
  {
    ROS_WARN_THROTTLE(1.0, "SimplePathFollower: No Transform available: %s", ex.what());
    return false;
  }
  catch (tf2::ConnectivityException& ex)
  {
    ROS_WARN_THROTTLE(1.0, "SimplePathFollower: Connectivity Error: %s", ex.what());
    return false;
  }
  catch (tf2::ExtrapolationException& ex)
  {
    ROS_WARN_THROTTLE(1.0, "SimplePathFollower: Extrapolation Error: %s", ex.what());
    return false;
  }
}

int SimplePathFollower::findClosestPoint(const geometry_msgs::PoseStamped& robot_pose, 
                                         const std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (plan.empty())
  {
    return -1;
  }

  int closest_idx = 0;
  double min_dist = distance(robot_pose, plan[0]);

  for (size_t i = 1; i < plan.size(); ++i)
  {
    double dist = distance(robot_pose, plan[i]);
    if (dist < min_dist)
    {
      min_dist = dist;
      closest_idx = i;
    }
  }

  return closest_idx;
}

bool SimplePathFollower::findTargetPoint(const geometry_msgs::PoseStamped& robot_pose, 
                                         int closest_idx,
                                         const std::vector<geometry_msgs::PoseStamped>& plan,
                                         geometry_msgs::PoseStamped& target_pose)
{
  if (plan.empty() || closest_idx < 0)
  {
    return false;
  }

  // Start searching from the closest point (or a bit ahead to avoid going backwards)
  size_t start_idx = std::max(0, closest_idx);
  
  // Find the first point that is at least lookahead_dist away
  for (size_t i = start_idx; i < plan.size(); ++i)
  {
    double dist = distance(robot_pose, plan[i]);
    if (dist >= lookahead_dist_)
    {
      target_pose = plan[i];
      return true;
    }
  }

  // If no point is far enough, use the goal
  if (!plan.empty())
  {
    target_pose = plan.back();
    return true;
  }

  return false;
}

double SimplePathFollower::distance(const geometry_msgs::PoseStamped& pose1, 
                                     const geometry_msgs::PoseStamped& pose2)
{
  double dx = pose1.pose.position.x - pose2.pose.position.x;
  double dy = pose1.pose.position.y - pose2.pose.position.y;
  return std::sqrt(dx * dx + dy * dy);
}

double SimplePathFollower::getYaw(const geometry_msgs::Pose& pose)
{
  tf2::Quaternion q(
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

} // namespace move_base

