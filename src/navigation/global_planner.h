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
// DATA STRUCTURES
    // voronoi builder. uses int32 by default -- no time for us to build out float functionality.
    voronoi_builder<int32> vb_;
    // voronoi diagram.
    voronoi_diagram<double> vd_;
    // vertex:vertex edge representation so we don't have to interact with Boost's API
    map<pair<float, float>, list<pair<float, float>>> voronoi_edge_map_;
    // our currently-planned path
    list<pair<float, float>> global_path_;
    // list of current map lines
    std::vector<Eigen::Vector2f> global_map_;

// CONSTANT VARIABLES
    // multiplier for all float coordinate points to prevent full decimal loss when converted
    // to int32 for insertion into voronoi_builder vb_.

    // also necessary to ensure our geometry does not overlap inside of the builder -- this breaks
    // the builder entirely. sun and macy spent many hours figuring this one out.
    // see here:
        // https://github.com/boostorg/polygon/issues/32
        // https://github.com/boostorg/polygon/issues/72
    // this scale factor is sufficient for a map of size similar to GDC1
    const uint64_t SCALE_FACTOR = 1e4;

// GLOBAL VARIABLES
    // goal coords scaled by SCALE_FACTOR
    pair<float, float> goal_coords_; // scaled
    // goal coords
    Eigen::Vector2f goal_;
    // voronoi cell goal is contained within
    const voronoi_diagram<double>::cell_type* goal_cell_;

    // start position coords (i.e. current robot position)
    Eigen::Vector2f start_;
    // voronoi cell start is contained within
    const voronoi_diagram<double>::cell_type* start_cell_;

//////  

    // default constructor ... we don't really use it, but we have to have it.
    GlobalPlanner() {
        goal_cell_ = nullptr;
        start_cell_ = nullptr;
    };

    // set goal point
    void set_goal(float goal_x, float goal_y);

    // set start point
    void set_start(float start_x, float start_y);

    // initialization function. builds the voronoi diagram.
    void construct_map(const vector_map::VectorMap& map);

    // run a* to plan a global path
    void plan_global_path();

    // get a carrot for the local navigation code to work towards
    bool get_carrot(Eigen::Vector2f& curr_loc, float curr_angle, Eigen::Vector2f* carrot_loc, amrl_msgs::VisualizationMsg & viz_msg);

    // visualizes global plan on map
    void visualize_global_plan(amrl_msgs::VisualizationMsg & viz_msg, uint32_t color = 0xff00ff);
    
    // visualizes our voronoi diagram on map
    void visualize_voronoi(amrl_msgs::VisualizationMsg & viz_msg, uint32_t color = 0xff0000);
};

#endif // GLOBAL_PLANNER_H
