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
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/msg/ackermann_curvature_drive_msg.hpp"
#include "amrl_msgs/msg/pose2_df.hpp"
#include "amrl_msgs/msg/visualization_msg.hpp"
#include "glog/logging.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "ros2_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include "path_options.h"
#include "latency_compensation.h"
#include <signal.h>

#include <geometry_msgs/msg/twist.hpp>

#include <irobot_create_msgs/action/drive_distance.hpp>
#include <irobot_create_msgs/action/drive_arc.hpp>
#include <irobot_create_msgs/action/rotate_angle.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "global_planner.h" 

using Eigen::Vector2f;
using amrl_msgs::msg::VisualizationMsg;
using std::string;
using std::vector;
using namespace math_util;
using namespace ros_helpers;

namespace {
rclcpp::Publisher<VisualizationMsg>::SharedPtr viz_pub_;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
geometry_msgs::msg::Twist twist_msg_;

// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

string GetMapFileFromName(const string &map) {
  string maps_dir_ = ament_index_cpp::get_package_share_directory("amrl_maps");
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}

Navigation::Navigation(const string &map_name, const std::shared_ptr<rclcpp::Node> &node) :
    node_(node),
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0),
    latency_compensation_(new LatencyCompensation(0, 0, 0)),
    global_planner_(),
    goal_established_(false)
  {
  cout << "navigation constructor" << endl;
  map_.Load(GetMapFileFromName(map_name));
  LOG(INFO) << "Loaded map file: " << GetMapFileFromName(map_name);
  viz_pub_ = node->create_publisher<VisualizationMsg>("visualization", 1);
  twist_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("/ut/cmd_vel", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");

  drive_distance_client_ = rclcpp_action::create_client<irobot_create_msgs::action::DriveDistance>(
        node_,
        "/ut/drive_distance");
  drive_arc_client_ = rclcpp_action::create_client<irobot_create_msgs::action::DriveArc>(
        node_,
        "/ut/drive_arc");
  rotate_angle_client_ = rclcpp_action::create_client<irobot_create_msgs::action::RotateAngle>(
        node_,
        "/ut/rotate_angle");
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  cout << "set nav goal" << endl;
  // time to voronoi
  nav_goal_loc_ = loc;
  nav_goal_angle_ = angle;
  global_planner_.set_goal(nav_goal_loc_.x(), nav_goal_loc_.y());
  // replan if new nav target is established after init 
  if (goal_established_) {
    global_planner_.set_start(robot_loc_.x(), robot_loc_.y());
    global_planner_.construct_map(map_);
    global_planner_.plan_global_path();
  }
  goal_established_ = true; 

}

bool Navigation::StartDriveDistanceAction(const double &distance, const double &max_translation_speed,
                                          const std::chrono::duration<int64_t, std::milli> &wait_for_server_timeout) {
  cout << "start drive distance action" << endl;
  if (anyGoalsInProgress()) {
      LOG(ERROR)
          << "Tried to initiate new action when one is already in progress. Make sure that one is not executing and reset variables appropriately";
      return false;
  }
  using namespace std::placeholders;
  if (!drive_distance_client_->wait_for_action_server(wait_for_server_timeout)) {
      LOG(INFO) << "Timed out waiting for server";
      return false;
  }

  if (!drive_distance_pending_feedback_msgs_.empty()) {
      LOG(WARNING) << "Have unprocessed feedback for drive distance. Clearing as we issue new goal";
      drive_distance_pending_feedback_msgs_.clear();
  }

  if (distance < 0) {
      LOG(WARNING) << "Turtlebot will only back up a couple cm, because it lacks cliff sensors on the back";
  }

  auto drive_distance_goal = irobot_create_msgs::action::DriveDistance::Goal();
  drive_distance_goal.distance = distance;
  drive_distance_goal.max_translation_speed = max_translation_speed;

  auto send_goal_options = rclcpp_action::Client<irobot_create_msgs::action::DriveDistance>::SendGoalOptions();
  send_goal_options.goal_response_callback =
        std::bind(&Navigation::DriveDistanceGoalResponseCallback, this, _1);
  send_goal_options.feedback_callback =
        std::bind(&Navigation::DriveDistanceFeedbackCallback, this, _1, _2);
  send_goal_options.result_callback =
        std::bind(&Navigation::DriveDistanceResultCallback, this, _1);
  drive_distance_goal_status_ = PENDING_ACCEPTANCE;
  drive_distance_goal_issued_time_ = node_->get_clock()->now();
  drive_distance_client_->async_send_goal(drive_distance_goal, send_goal_options);
  return true;
}

bool Navigation::StartDriveArcAction(const int &translate_direction, const double &arc_radius, const double &arc_angle, const double &max_translation_speed,
                         const std::chrono::duration<int64_t, std::milli> &wait_for_server_timeout) {
  cout << "start drive arc action" << endl;
  if (anyGoalsInProgress()) {
      LOG(ERROR)
          << "Tried to initiate new action when one is already in progress. Make sure that one is not executing and reset variables appropriately";
      return false;
  }
  using namespace std::placeholders;
  if (!drive_arc_client_->wait_for_action_server(wait_for_server_timeout)) {
      LOG(INFO) << "Timed out waiting for server";
      return false;
  }

  if (!drive_arc_pending_feedback_msgs_.empty()) {
      LOG(WARNING) << "Have unprocessed feedback for drive arc. Clearing as we issue new goal";
      drive_arc_pending_feedback_msgs_.clear();
  }

  if (arc_angle < 0) {
      LOG(WARNING) << "Turtlebot will only back up a couple cm, because it lacks cliff sensors on the back";
  }

  auto drive_arc_goal = irobot_create_msgs::action::DriveArc::Goal();
  drive_arc_goal.translate_direction = translate_direction;
  drive_arc_goal.angle = arc_angle;
  drive_arc_goal.radius = arc_radius;
  drive_arc_goal.max_translation_speed = max_translation_speed;

  auto send_goal_options = rclcpp_action::Client<irobot_create_msgs::action::DriveArc>::SendGoalOptions();
  send_goal_options.goal_response_callback =
        std::bind(&Navigation::DriveArcGoalResponseCallback, this, _1);
  send_goal_options.feedback_callback =
        std::bind(&Navigation::DriveArcFeedbackCallback, this, _1, _2);
  send_goal_options.result_callback =
        std::bind(&Navigation::DriveArcResultCallback, this, _1);
  drive_arc_goal_status_ = PENDING_ACCEPTANCE;
  drive_arc_goal_issued_time_ = node_->get_clock()->now();
  drive_arc_client_->async_send_goal(drive_arc_goal, send_goal_options);
  return true;
}

bool Navigation::StartRotateAngleAction(const double &angle_rad, const double &max_rotation_speed,
                                        const std::chrono::duration<int64_t, std::milli> &wait_for_server_timeout) {
  cout << "start rotate angle action" << endl;
  if (anyGoalsInProgress()) {
      LOG(ERROR)
          << "Tried to initiate new action when one is already in progress. Make sure that one is not executing and reset variables appropriately";
      return false;
  }
  using namespace std::placeholders;
  if (!rotate_angle_client_->wait_for_action_server(wait_for_server_timeout)) {
      LOG(INFO) << "Timed out waiting for server";
      return false;
  }

  if (!rotate_angle_pending_feedback_msgs_.empty()) {
      LOG(WARNING) << "Have unprocessed feedback for rotate_angle. Clearing as we issue new goal";
      rotate_angle_pending_feedback_msgs_.clear();
  }

  auto rotate_angle_goal = irobot_create_msgs::action::RotateAngle::Goal();
  rotate_angle_goal.angle = angle_rad;
  rotate_angle_goal.max_rotation_speed = max_rotation_speed;

  auto send_goal_options = rclcpp_action::Client<irobot_create_msgs::action::RotateAngle>::SendGoalOptions();
  send_goal_options.goal_response_callback =
        std::bind(&Navigation::RotateAngleGoalResponseCallback, this, _1);
  send_goal_options.feedback_callback =
        std::bind(&Navigation::RotateAngleFeedbackCallback, this, _1, _2);

  send_goal_options.result_callback =
        std::bind(&Navigation::RotateAngleResultCallback, this, _1);
  rotate_angle_goal_status_ = PENDING_ACCEPTANCE;
  rotate_angle_goal_issued_time_ = node_->get_clock()->now();
  rotate_angle_client_->async_send_goal(rotate_angle_goal, send_goal_options);
  return true;
}

void Navigation::UpdateLocation(const Eigen::Vector2f &loc, float angle) {
  // cout << "update location" << endl;
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f &loc,
                                float angle,
                                const Vector2f &vel,
                                float ang_vel) {
  // cout << "update odometry" << endl;
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    odom_loc_ = loc;
    odom_angle_ = angle;
    return;
  }
  latency_compensation_->recordObservation(loc[0], loc[1], angle, node_->get_clock()->now().seconds());
  double node_time = node_->get_clock()->now().seconds();
  Observation predictedState = latency_compensation_->getPredictedState(node_time);
  odom_loc_ = {predictedState.x, predictedState.y};
  odom_angle_ = predictedState.theta;
  robot_vel_ = {predictedState.vx, predictedState.vy};
  robot_omega_ = predictedState.omega;

  point_cloud_ = latency_compensation_->forward_predict_point_cloud(point_cloud_, predictedState.x, predictedState.y, predictedState.theta);
}

void Navigation::ObservePointCloud(const vector<Vector2f> &cloud,
                                   double time) {
  // todo check on what time is for?
  // cout << "observe point cloud" << endl;
  point_cloud_ = cloud;                                     
}

void Navigation::SetLatencyCompensation(LatencyCompensation* latency_compensation) {
  // cout << "set latency compensation" << endl;
  latency_compensation_ = latency_compensation;
}

// Convert (velocity, curvature) to (x_dot, y_dot, theta_dot)
Control Navigation::GetCartesianControl(float velocity, float curvature, double time) {
  // cout << "get cartesian control" << endl;
  float x_dot = velocity * cos(curvature);
  float y_dot = velocity * sin(curvature);
  float theta_dot = velocity * curvature;

  return {x_dot, y_dot, theta_dot, time};
}

// sets the drive msg
void Navigation::SimpleController(Eigen::Vector2f & local_carrot) {
  float angle_to_carrot = atan2(local_carrot.y(), local_carrot.x());
  // float angle_diff = math_util::AngleDiff(angle_to_carrot, robot_angle_);
  // float distance_to_carrot = local_carrot.norm();

  // if not facing carrot, rotate to face carrot
  // otherwise, drive straight to the carrot
  if (abs(angle_to_carrot) > .1) {
    // angular should be v * curvature but I made it just curvature?? so it is 1/10th as much turning
    twist_msg_.angular.z = angle_to_carrot;
    twist_msg_.linear.x = .1;
  } else {
    // linear should be just v but I multiply by 10 bc it's slow otherwise
    twist_msg_.angular.z = 0;
    // sometimes 1dtoc returns crazy values. I am unable to replicate that so let's just hope it doesn't happen again
    twist_msg_.linear.x = run1DTimeOptimalControl(local_carrot.norm(), robot_vel_.norm(), robot_config_) * 10;

  }
}

void Navigation::Run() {
  // cout << "run" << endl;
  // This function gets called 20 times a second to form the control loop.

  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // cout << "odom initialized?: " << odom_initialized_ << "goal established?: " << goal_established_ << endl;
  // If odometry has not been initialized or goal has not been set, we can't do anything.
  if (!odom_initialized_ || !goal_established_) return;

  // robot is within .5m of goal, consider it reached
  if ((robot_loc_ - nav_goal_loc_).squaredNorm() < 0.25){
    twist_msg_.linear.x = 0;
    return;
  }

  // The control iteration goes here.
  // Feel free to make helper functions to structure the control appropriately.

  // The latest observed point cloud is accessible via "point_cloud_"

  // Eventually, you will have to set the control values to issue drive commands:
  // You can call StartDriveDistanceAction, StartDriveArcAction, or StartRotateAngleAction
  // and then wait for the status

  // You should also check that it's not stuck due to ros communication errors by keeping track of how much time
  // has elapsed since your request

  // Make sure to reset the status to NONE once you've processed the completion of an action request
  // float current_speed = robot_vel_.norm();
  // cout << current_speed << endl;
  // distance_traveled_ += current_speed * robot_config_.dt;
  // float dist_to_go = (10 - distance_traveled_); // hard code to make it go 10 forward
  // float cmd_vel = run1DTimeOptimalControl(dist_to_go, current_speed, robot_config_);


  Eigen::Vector2f carrot_loc = nav_goal_loc_;
  bool carrot_found = global_planner_.get_carrot(robot_loc_, robot_angle_, &carrot_loc, global_viz_msg_);
  // plan must have been invalid. replan and get a new carrot
  // if(!carrot_found) {
    global_planner_.set_start(robot_loc_.x(), robot_loc_.y());
    global_planner_.construct_map(map_);
    global_planner_.plan_global_path();
    carrot_found = global_planner_.get_carrot(robot_loc_, robot_angle_, &carrot_loc, global_viz_msg_);
    // plan must be unreachable. stop moving
    if(!carrot_found) {
      twist_msg_.linear.x = 0;
      goal_established_ = false;
      // print no carrot found
      cout << "No carrot found" << endl;
      return;
    }
  // }
  visualization::DrawCross(carrot_loc, 1, 0x23AB3e, global_viz_msg_);

  // cout << "0 "<<carrot_loc.x() << " " << carrot_loc.y() << endl;
  
  // transform carrot_loc to robot frame
  carrot_loc = carrot_loc - robot_loc_;

  // cout << "1 "<<carrot_loc.x() << " " << carrot_loc.y() << endl;
  // rotate goal_loc by -robot_angle_
  Eigen::Matrix2f rot;
  rot << cos(-robot_angle_), -sin(-robot_angle_), sin(-robot_angle_), cos(-robot_angle_);
  carrot_loc = rot * carrot_loc;

  // print the carrot
  // cout << "2 "<< carrot_loc.x() << " " << carrot_loc.y() << endl;

  vector<PathOption> path_options = samplePathOptions(31, point_cloud_, robot_config_, carrot_loc);
  int best_path = selectPath(path_options, carrot_loc);

  // todo: set twist, set twist.linear.x to speed twist.angular.z is curvature * speed 
  // drive_msg_.curvature = path_options[best_path].curvature;
  // drive_msg_.velocity = run1DTimeOptimalControl(path_options[best_path].free_path_length, current_speed, robot_config_);
	
  // cout << drive_msg_.curvature << " " << drive_msg_.velocity << endl;
  SimpleController(carrot_loc);
  // visualization here
  // visualization::DrawRectangle(Vector2f(robot_config_.length/2 - robot_config_.base_link_offset, 0),
  //     robot_config_.length, robot_config_.width, 0, 0x00FF00, local_viz_msg_);
    visualization::DrawArc(Vector2f(0,0), .17, 0, 360, 0x00FF00, local_viz_msg_);
  // Draw all path options in blue
  // for (unsigned int i = 0; i < path_options.size(); i++) {
  //     visualization::DrawPathOption(path_options[i].curvature, path_options[i].free_path_length, 0, 0x0000FF, false, local_viz_msg_);
  //     visualization::DrawCross(path_options[i].closest_point, .2, 0x0000FF, local_viz_msg_);
  // }
  // Draw the best path in red
  // visualization::DrawPathOption(path_options[best_path].curvature, path_options[best_path].free_path_length, path_options[best_path].clearance, 0xFF0000, true, local_viz_msg_);
  // visualization::DrawCross(path_options[best_path].closest_point, 1, 0x00ff00, local_viz_msg_);
// Find the closest point in the point cloud

  // visualize goal location
  // visualization::DrawCross(nav_goal_loc_, 2, 0x2038FB, global_viz_msg_);

  // Plot the closest point in purple
  // visualization::DrawLine(path_options[best_path].closest_point, Vector2f(0, 1/path_options[best_path].curvature), 0xFF00FF, local_viz_msg_);
  // for debugging
  global_planner_.visualize_voronoi(global_viz_msg_);
  global_planner_.visualize_global_plan(global_viz_msg_);
  
    

  // geometry_msgs::msg::Twist twist;
  // twist.linear.x = 1;
  // twist_pub_->publish(twist);

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = node_->get_clock()->now();
  global_viz_msg_.header.stamp = node_->get_clock()->now();
  // Publish messages.
  viz_pub_->publish(global_viz_msg_);
  viz_pub_->publish(local_viz_msg_);
  twist_pub_->publish(twist_msg_);

  // Record control for latency compensation
  // todo set twist stuff
  // todo fix control, use ros_helpers::rosHeaderStampToSeconds(msg.header)
  Control control = GetCartesianControl(twist_msg_.linear.x, twist_msg_.angular.z, node_->get_clock()->now().seconds());
  latency_compensation_->recordControl(control);

  // Hack because ssh -X is slow
  // if (latency_compensation_->getControlQueue().size() == 100) {
  //  exit(0);
}

} // namespace navigation
