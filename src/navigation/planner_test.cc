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
#include "ros/ros.h"
#include "ros/package.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include "path_options.h"
#include "latency_compensation.h"
#include "global_planner.h" 

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::msg::VisualizationMsg;
using std::string;
using std::vector;


int main(int argc, char** argv) {
    string map = "ICRA2022_F1Tenth_Track";
    string maps_dir = "/home/luisamao/amrl_maps/";

    string map_name =  maps_dir + "/" + map + "/" + map + ".vectormap.txt";
    vector_map::VectorMap map_;
    map_.Load(map_name);
    GlobalPlanner global_planner_;
    global_planner_.construct_map(map_);

    Eigen::Vector2f start(5.53, 8.29);

    // global_planner_.plan_global_path(start, 0);

    for (const auto& vertex : global_planner_.global_path_) {
        // print the pair
        std::cout<< vertex.first << "   " << vertex.second << std::endl;
    }


    std::cout << global_planner_.vd_.vertices().size() << std::endl;

    // for (const auto& edge : global_planner_.vd_.edges()) {
    //     // print the type of the edge
    //     std::cout<< edge.vertex0() << "   " << edge.vertex1() << std::endl;
    // }
    VisualizationMsg viz_msg;
    global_planner_.visualize_voronoi(viz_msg, 0xff0000);

    std::cout << "done" <<std::endl;



}