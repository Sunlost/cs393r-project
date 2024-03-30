#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
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
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;


int main(int argc, char** argv) {
    string map = "GDC1";
    string maps_dir = "/home/luisamao/amrl_maps/";

    string map_name =  maps_dir + "/" + map + "/" + map + ".vectormap.txt";
    vector_map::VectorMap map_;
    map_.Load(map_name);
    GlobalPlanner global_planner_;
    global_planner_.initialize(map_);

    std::cout<< "site events here " << global_planner_.vb_.site_events_.size() << std::endl;

    global_planner_.build_voronoi();
    std::cout<< "site events here " << global_planner_.vb_.site_events_.size() << std::endl;

    std::cout << global_planner_.vd_.vertices().size() << std::endl;

    // for (const auto& edge : global_planner_.vd_.edges()) {
    //     // print the type of the edge
    //     std::cout<< edge.vertex0() << "   " << edge.vertex1() << std::endl;
    // }

    std::cout << "done" <<std::endl;



}