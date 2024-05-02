#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/msg/localization2_d_msg.hpp"
#include "gflags/gflags.h"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "glog/logging.h"
// #include "ros/ros.h"
// #include "ros/package.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
// #include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include "path_options.h"
#include "latency_compensation.h"
#include "global_planner.h" 

using Eigen::Vector2f;
// using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::msg::VisualizationMsg;
using std::string;
using std::vector;


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("ut_navigation");
    rclcpp::Publisher<VisualizationMsg>::SharedPtr viz_pub_ = node->create_publisher<VisualizationMsg>("visualization", 1);

    GlobalPlanner global_planner_;



    // string map = "UT_AUTOmata_Track";
    // global_planner_.set_start(1.5, 2.93);

    // global_planner_.set_goal(9.48, 1.02);


    string map = "ICRA2022_F1Tenth_Track";
    global_planner_.set_start(3.11, 8.18);

    // global_planner_.set_goal(27.21, 4.25);


    // string map = "CDL_Ground";
    // global_planner_.set_start(2.49, -14.37);

    // global_planner_.set_goal(12.03, -1);



    string maps_dir = "/home/dev/amrl_maps/";
    string map_name =  maps_dir + "/" + map + "/" + map + ".vectormap.txt";
    vector_map::VectorMap map_;
    map_.Load(map_name);

    global_planner_.construct_map(map_);
    
    global_planner_.plan_global_path();

    for (const auto& vertex : global_planner_.global_path_) {
        // print the pair
        std::cout << vertex->first << ", " << vertex->second << std::endl;
    }

    // std::cout << global_planner_.vd_.vertices().size() << std::endl;

    // for (const auto& edge : global_planner_.vd_.edges()) {
    //     // print the type of the edge
    //     std::cout<< edge.vertex0() << "   " << edge.vertex1() << std::endl;
    // }

    VisualizationMsg global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
    global_planner_.visualize_voronoi(global_viz_msg_);
    global_planner_.visualize_global_plan(global_viz_msg_);

    global_viz_msg_.header.stamp = node->get_clock()->now();
    viz_pub_->publish(global_viz_msg_);

    std::cout << "done" <<std::endl;

}