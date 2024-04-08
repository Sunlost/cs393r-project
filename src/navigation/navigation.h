//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <vector>

#include "eigen3/Eigen/Dense"

#include "vector_map/vector_map.h"
#include "latency_compensation.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <irobot_create_msgs/action/drive_distance.hpp>
#include <irobot_create_msgs/action/drive_arc.hpp>
#include <irobot_create_msgs/action/rotate_angle.hpp>
#include <chrono>
#include <memory>
#include "global_planner.h"


#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace navigation {

struct PathOption {
  float curvature = 0;
  float clearance = 10;
  float free_path_length = 100;
  float dist_to_closest_point = 100;
  Eigen::Vector2f obstruction = Eigen::Vector2f::Zero();
  Eigen::Vector2f closest_point = Eigen::Vector2f::Zero();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct NavigationParams {
  // frequency
  float dt = .05f;
  // max velocity
  float max_vel = 5.0f;
  // max acceleration
  float max_accel = 4.0f;
  float max_decel = 4.0f;
  // max angular velocity
  float max_omega = 1.0f;
  // max angular acceleration
  float max_alpha = 1.0f;
  // max curvature
  float max_curvature = 1.0f;
  // safety margin
  float safety_margin = 0.1f;

  // robot dimensions
  float  width = 0.281f;
  float  length = 0.535f;
  float  wheelbase = 0.324f;
  float  base_link_offset = 0.106f; // make this 0 for now

  // delays
  float actuation_latency = 0.2f;
  float observation_latency = 0.05f;
};

enum GoalStatus {
  NONE,
  PENDING_ACCEPTANCE,
  FAILED_ACCEPTANCE,
  AWAITING_RESPONSE,
  COMPLETED_GOAL,
  ABORTED_GOAL,
  CANCELLED_GOAL,
  UNKNOWN_RESULT
};

class Navigation {
  public:

  // Constructor
  explicit Navigation(const std::string &map_file, const std::shared_ptr<rclcpp::Node> &node);

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f &loc, float angle);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f &loc,
                      float angle,
                      const Eigen::Vector2f &vel,
                      float ang_vel);

  // Updates based on an observed laser scan
  void ObservePointCloud(const std::vector<Eigen::Vector2f> &cloud,
                         double time);

  // Main function called continously from main
  void Run();

  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f &loc, float angle);

  bool StartDriveDistanceAction(const double &distance, const double &max_translation_speed,
                                const std::chrono::duration<int64_t, std::milli> &wait_for_server_timeout);

  bool StartDriveArcAction(const int &translate_direction, const double &arc_radius, const double &arc_angle,
                           const double &max_translation_speed,
                           const std::chrono::duration<int64_t, std::milli> &wait_for_server_timeout);

  bool StartRotateAngleAction(const double &angle_rad, const double &max_rotation_speed,
                              const std::chrono::duration<int64_t, std::milli> &wait_for_server_timeout);

  // // Set the latency compensation object.
  void SetLatencyCompensation(LatencyCompensation* latency_compensation);

  Control GetCartesianControl(float velocity, float curvature, double time);

private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp_action::Client<irobot_create_msgs::action::DriveDistance>::SharedPtr drive_distance_client_;
  rclcpp_action::Client<irobot_create_msgs::action::DriveArc>::SharedPtr drive_arc_client_;
  rclcpp_action::Client<irobot_create_msgs::action::RotateAngle>::SharedPtr rotate_angle_client_;

  // Whether odometry has been initialized.
  bool odom_initialized_;
  // Whether localization has been initialized.
  bool localization_initialized_;
  // Current robot location.
  Eigen::Vector2f robot_loc_;
  // Current robot orientation.
  float robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Odometry-reported robot location.
  Eigen::Vector2f odom_loc_;
  // Odometry-reported robot angle.
  float odom_angle_;
  // Odometry-reported robot starting location.
  Eigen::Vector2f odom_start_loc_;
  // Odometry-reported robot starting angle.
  float odom_start_angle_;
  // Latest observed point cloud.
  std::vector<Eigen::Vector2f> point_cloud_;
  // distance traveled
  float distance_traveled_ = 0.0f;

  // Whether navigation is complete.
  bool nav_complete_;
  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle.
  float nav_goal_angle_;
  // Map of the environment.
  vector_map::VectorMap map_;

  // robot config
  NavigationParams robot_config_;

  // Latency compensation
  LatencyCompensation* latency_compensation_;

  GlobalPlanner global_planner_;
  bool goal_established_;

  // Status of the most recent goal request.
  // Should be none if there isn't one in progress / that you still have to process the completion of
  GoalStatus drive_distance_goal_status_ = GoalStatus::NONE;
  GoalStatus drive_arc_goal_status_ = GoalStatus::NONE;
  GoalStatus rotate_angle_goal_status_ = GoalStatus::NONE;

  // Time that the last command of the time was issued
  // Users should use these to ensure that there is progress being made
  // Set your distances such that you can have a reasonable timeout
  rclcpp::Time drive_distance_goal_issued_time_ = rclcpp::Time(0);
  rclcpp::Time drive_arc_goal_issued_time_ = rclcpp::Time(0);
  rclcpp::Time rotate_angle_goal_issued_time_ = rclcpp::Time(0);

  // These should not be read unless the status is COMPLETED_GOAL
  geometry_msgs::msg::PoseStamped drive_distance_result;
  geometry_msgs::msg::PoseStamped drive_arc_result;
  geometry_msgs::msg::PoseStamped rotate_angle_result;

  using DriveDistance = irobot_create_msgs::action::DriveDistance;
  using DriveArc = irobot_create_msgs::action::DriveArc;
  using RotateAngle = irobot_create_msgs::action::RotateAngle;

  using DriveDistanceFeedback = DriveDistance::Feedback;
  using DriveArcFeedback = DriveArc::Feedback;
  using RotateAngleFeedback = RotateAngle::Feedback;

  using DriveDistanceGoalHandle = rclcpp_action::ClientGoalHandle<DriveDistance>;
  using DriveArcGoalHandle = rclcpp_action::ClientGoalHandle<DriveArc>;
  using RotateAngleGoalHandle = rclcpp_action::ClientGoalHandle<RotateAngle>;

  // Feedback received. Should be emptied before the next call of the given type
  std::vector<std::shared_ptr<const DriveDistanceFeedback>>
        drive_distance_pending_feedback_msgs_;
  std::vector<std::shared_ptr<const DriveArcFeedback>>
        drive_arc_pending_feedback_msgs_;
  std::vector<std::shared_ptr<const RotateAngleFeedback>>
        rotate_angle_pending_feedback_msgs_;

  void DriveDistanceGoalResponseCallback(
        std::shared_future<DriveDistanceGoalHandle::SharedPtr> future) {
      auto drive_distance_goal_handle = future.get();
      if (!drive_distance_goal_handle) {
        LOG(ERROR) << "Drive distance goal rejected by server";
        drive_distance_goal_status_ = GoalStatus::FAILED_ACCEPTANCE;
      } else {
        LOG(INFO) << "Drive distance goal accepted by server; waiting for result";
        drive_distance_goal_status_ = GoalStatus::AWAITING_RESPONSE;
      }
  }

  void
  DriveArcGoalResponseCallback(std::shared_future<DriveArcGoalHandle::SharedPtr> future) {
      auto drive_arc_goal_handle = future.get();
      if (!drive_arc_goal_handle) {
        LOG(ERROR) << "Drive arc goal rejected by server";
        drive_arc_goal_status_ = GoalStatus::FAILED_ACCEPTANCE;
      } else {
        LOG(INFO) << "Drive arc goal accepted by server; waiting for result";
        drive_arc_goal_status_ = GoalStatus::AWAITING_RESPONSE;
      }
  }

  void RotateAngleGoalResponseCallback(
        std::shared_future<RotateAngleGoalHandle::SharedPtr> future) {
      auto rotate_angle_goal_handle = future.get();
      if (!rotate_angle_goal_handle) {
        LOG(ERROR) << "Rotate angle goal rejected by server";
        rotate_angle_goal_status_ = GoalStatus::FAILED_ACCEPTANCE;
      } else {
        LOG(INFO) << "Rotate angle goal accepted by server; waiting for result";
        rotate_angle_goal_status_ = GoalStatus::AWAITING_RESPONSE;
      }
  }

  void DriveDistanceFeedbackCallback(
        DriveDistanceGoalHandle::SharedPtr drive_distance_goal_handle,
        const std::shared_ptr<const DriveDistanceFeedback> feedback) {
      drive_distance_pending_feedback_msgs_.emplace_back(feedback);
  }

  void DriveArcFeedbackCallback(
        DriveArcGoalHandle::SharedPtr drive_arc_goal_handle,
        const std::shared_ptr<const DriveArcFeedback> feedback) {
      drive_arc_pending_feedback_msgs_.emplace_back(feedback);
  }

  void RotateAngleFeedbackCallback(
        RotateAngleGoalHandle::SharedPtr rotate_angle_goal_handle,
        const std::shared_ptr<const RotateAngleFeedback> feedback) {
      rotate_angle_pending_feedback_msgs_.emplace_back(feedback);
  }

  void DriveDistanceResultCallback(
        const DriveDistanceGoalHandle::WrappedResult &result) {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          drive_distance_goal_status_ = GoalStatus::COMPLETED_GOAL;
          drive_distance_result = result.result->pose;
          return;
        case rclcpp_action::ResultCode::ABORTED:
          drive_distance_goal_status_ = GoalStatus::ABORTED_GOAL;
          LOG(ERROR) << "Drive distance goal aborted";
          return;
        case rclcpp_action::ResultCode::CANCELED:
          drive_distance_goal_status_ = GoalStatus::CANCELLED_GOAL;
          LOG(ERROR) << "Drive distance goal cancelled";
          return;
        default:
          drive_distance_goal_status_ = GoalStatus::UNKNOWN_RESULT;
          LOG(ERROR) << "Drive distance goal unknown result code";
          return;
      }
  }

  void DriveArcResultCallback(
        const DriveArcGoalHandle::WrappedResult &result) {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          drive_arc_goal_status_ = GoalStatus::COMPLETED_GOAL;
          drive_arc_result = result.result->pose;
          return;
        case rclcpp_action::ResultCode::ABORTED:
          drive_arc_goal_status_ = GoalStatus::ABORTED_GOAL;
          LOG(ERROR) << "Drive arc goal aborted";
          return;
        case rclcpp_action::ResultCode::CANCELED:
          drive_arc_goal_status_ = GoalStatus::CANCELLED_GOAL;
          LOG(ERROR) << "Drive arc goal cancelled";
          return;
        default:
          drive_arc_goal_status_ = GoalStatus::UNKNOWN_RESULT;
          LOG(ERROR) << "Drive arc goal unknown result code";
          return;
      }
  }

  void RotateAngleResultCallback(
        const RotateAngleGoalHandle::WrappedResult &result) {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          rotate_angle_goal_status_ = GoalStatus::COMPLETED_GOAL;
          rotate_angle_result = result.result->pose;
          return;
        case rclcpp_action::ResultCode::ABORTED:
          rotate_angle_goal_status_ = GoalStatus::ABORTED_GOAL;
          LOG(ERROR) << "Rotate angle goal aborted";
          return;
        case rclcpp_action::ResultCode::CANCELED:
          rotate_angle_goal_status_ = GoalStatus::CANCELLED_GOAL;
          LOG(ERROR) << "Rotate angle goal cancelled";
          return;
        default:
          rotate_angle_goal_status_ = GoalStatus::UNKNOWN_RESULT;
          LOG(ERROR) << "Rotate angle goal unknown result code";
          return;
      }
  }


  bool goalTypeInProgress(const GoalStatus &goal_status) {
      return goal_status != GoalStatus::NONE; // All other statuses mean that the goal is either still in progress or that
      // the results have to be processed
  }

  bool anyGoalsInProgress() {
      return goalTypeInProgress(drive_distance_goal_status_) || goalTypeInProgress(drive_arc_goal_status_) ||
           goalTypeInProgress(rotate_angle_goal_status_);
  }


};


}  // namespace navigation

#endif  // NAVIGATION_H
