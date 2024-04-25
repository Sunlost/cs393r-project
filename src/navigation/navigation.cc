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
#include "global_planner.h" 
#include <signal.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

using Eigen::Vector2f;
using amrl_msgs::msg::AckermannCurvatureDriveMsg;
using amrl_msgs::msg::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

namespace {
rclcpp::Publisher<AckermannCurvatureDriveMsg>::SharedPtr drive_pub_;
rclcpp::Publisher<VisualizationMsg>::SharedPtr viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

string GetMapFileFromName(const string& map) {
  string maps_dir_ = ament_index_cpp::get_package_share_directory("amrl_maps");
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}

Navigation::Navigation(const string& map_name, const std::shared_ptr<rclcpp::Node> &node) :
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
  map_.Load(GetMapFileFromName(map_name));
  LOG(INFO) << "Loaded map file: " << GetMapFileFromName(map_name);
  drive_pub_ = node->create_publisher<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = node->create_publisher<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", node->get_clock()->now(), &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
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

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
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
  latency_compensation_->recordObservation(loc[0], loc[1], angle, ros::Time::now().toSec());
  Observation predictedState = latency_compensation_->getPredictedState();
  odom_loc_ = {predictedState.x, predictedState.y};
  odom_angle_ = predictedState.theta;
  robot_vel_ = {predictedState.vx, predictedState.vy};
  robot_omega_ = predictedState.omega;

  point_cloud_ = latency_compensation_->forward_predict_point_cloud(point_cloud_, predictedState.x, predictedState.y, predictedState.theta);
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;                            
}

void Navigation::SetLatencyCompensation(LatencyCompensation* latency_compensation) {
  latency_compensation_ = latency_compensation;
}

// Convert (velocity, curvature) to (x_dot, y_dot, theta_dot)
Control Navigation::GetCartesianControl(float velocity, float curvature, double time) {
  float x_dot = velocity * cos(curvature);
  float y_dot = velocity * sin(curvature);
  float theta_dot = velocity * curvature;

  return {x_dot, y_dot, theta_dot, time};
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.

  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized or goal has not been set, we can't do anything.
  if (!odom_initialized_ || !goal_established_) return;

  // robot is within .5m of goal, consider it reached
  if ((robot_loc_ - nav_goal_loc_).squaredNorm() < 0.25){
    drive_msg_.velocity = 0;
    return;
  }

  // The control iteration goes here. 
  // Feel free to make helper functions to structure the control appropriately.
  
  // The latest observed point cloud is accessible via "point_cloud_"

  // Eventually, you will have to set the control values to issue drive commands:
  // drive_msg_.curvature = ...;
  // drive_msg_.velocity = ...;
  float current_speed = robot_vel_.norm();
  // cout << current_speed << endl;
  // distance_traveled_ += current_speed * robot_config_.dt;
  // float dist_to_go = (10 - distance_traveled_); // hard code to make it go 10 forward
  // float cmd_vel = run1DTimeOptimalControl(dist_to_go, current_speed, robot_config_);


  Eigen::Vector2f carrot_loc = nav_goal_loc_;
  bool carrot_found = global_planner_.get_carrot(robot_loc_, robot_angle_, &carrot_loc, global_viz_msg_);
  // plan must have been invalid. replan and get a new carrot
  if(!carrot_found) {
    global_planner_.set_start(robot_loc_.x(), robot_loc_.y());
    global_planner_.construct_map(map_);
    global_planner_.plan_global_path();
    carrot_found = global_planner_.get_carrot(robot_loc_, robot_angle_, &carrot_loc, global_viz_msg_);
    // plan must be unreachable. stop moving
    if(!carrot_found) {
      drive_msg_.velocity = 0;
      goal_established_ = false;
      return;
    }
  }
  visualization::DrawCross(carrot_loc, 1, 0xFF0000, global_viz_msg_);

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

  drive_msg_.curvature = path_options[best_path].curvature;
  drive_msg_.velocity = run1DTimeOptimalControl(path_options[best_path].free_path_length, current_speed, robot_config_);
	
  // cout << drive_msg_.curvature << " " << drive_msg_.velocity << endl;

  // visualization here
  visualization::DrawRectangle(Vector2f(robot_config_.length/2 - robot_config_.base_link_offset, 0),
      robot_config_.length, robot_config_.width, 0, 0x00FF00, local_viz_msg_);
  // Draw all path options in blue
  for (unsigned int i = 0; i < path_options.size(); i++) {
      visualization::DrawPathOption(path_options[i].curvature, path_options[i].free_path_length, 0, 0x0000FF, false, local_viz_msg_);
      visualization::DrawCross(path_options[i].closest_point, .2, 0xff0000, local_viz_msg_);
  }
    // visualization::DrawCross(goal_loc_rot, .2, 0x00ff00, local_viz_msg_);
  // Draw the best path in red
  visualization::DrawPathOption(path_options[best_path].curvature, path_options[best_path].free_path_length, path_options[best_path].clearance, 0xFF0000, true, local_viz_msg_);
// Find the closest point in the point cloud

  // visualize goal location
  // visualization::DrawCross(nav_goal_loc_, 2, 0x2038FB, global_viz_msg_);

  // Plot the closest point in purple
  // visualization::DrawLine(path_options[best_path].closest_point, Vector2f(0, 1/path_options[best_path].curvature), 0xFF00FF, local_viz_msg_);
  // for debugging
  global_planner_.visualize_voronoi(global_viz_msg_);
  global_planner_.visualize_global_plan(global_viz_msg_);
  
    

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = node_->get_clock()->now();
  global_viz_msg_.header.stamp = node_->get_clock()->now();
  drive_msg_.header.stamp = node_->get_clock()->now();
  // Publish messages.
  viz_pub_->publish(local_viz_msg_);
  viz_pub_->publish(global_viz_msg_);
  drive_pub_->publish(drive_msg_);
  // Record control for latency compensation
  Control control = GetCartesianControl(drive_msg_.velocity, drive_msg_.curvature, drive_msg_.header.stamp.toSec());
  latency_compensation_->recordControl(control);

  // Hack because ssh -X is slow
  // if (latency_compensation_->getControlQueue().size() == 100) {
  //  exit(0);
}

} // namespace navigation
