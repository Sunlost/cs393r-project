#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include "voronoi/voronoi_builder.hpp"
#include "voronoi/voronoi_diagram.hpp"
#include "vector_map/vector_map.h"
#include "simple_queue.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "shared/math/geometry.h"
#include "amrl_msgs/VisualizationMsg.h"
#include <cstddef>
#include <cstdint>
#include <utility>

using std::list;
using std::map;

class GlobalPlanner {
public:
    voronoi_builder<int32> vb_;
    voronoi_diagram<double> vd_;
    map<pair<float, float>, list<pair<float, float>>> voronoi_edge_map_;
    list<pair<float, float>> global_path_;
    std::vector<geometry::line2f> global_map_;

    const uint64_t SCALE_FACTOR = 100;

    pair<float, float> goal_coords_; // scaled
    Eigen::Vector2f goal_;
    Eigen::Vector2f start_;
    voronoi_diagram<double>::cell_type* goal_cell_;
    voronoi_diagram<double>::cell_type* start_cell_;

    //////

    // default constructor ... we don't really use it, but we have to have it.
    GlobalPlanner() {
        goal_cell_ = nullptr;
    };

    // find the closest vertex in the voronoi diagram to a given point
    pair<float, float> find_start_vertex(Eigen::Vector2f& curr_loc);

    // get a carrot for the local navigation code to work towards
    bool get_carrot(Eigen::Vector2f& curr_loc, float curr_angle, Eigen::Vector2f* carrot_loc);

    // run a* to plan a global path
    void plan_global_path();

    void set_goal(float goal_x, float goal_y);
    void set_start(float start_x, float start_y);

    // initialization function. builds the voronoi diagram.
    void construct_map(const vector_map::VectorMap& map);

    void visualize_global_plan(amrl_msgs::VisualizationMsg & viz_msg, uint32_t color = 0xff00ff);
    void visualize_voronoi(amrl_msgs::VisualizationMsg & viz_msg, uint32_t color = 0xff0000);
};

#endif // GLOBAL_PLANNER_H
