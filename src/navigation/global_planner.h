#include "voronoi/voronoi_builder.hpp"
#include "voronoi/voronoi_diagram.hpp"
#include "vector_map/vector_map.h"
#include "simple_queue.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
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

    const uint64_t SCALE_FACTOR = 100;

    pair<float, float> goal_coords;
    voronoi_diagram<double>::cell_type* goal_cell;

    //////

    // default constructor ... we don't really use it, but we have to have it.
    GlobalPlanner() {
        goal_cell = nullptr;
    };

    // find the closest vertex in the voronoi diagram to a given point
    pair<float, float> find_start_vertex(Eigen::Vector2f& curr_loc);

    // get a carrot for the local navigation code to work towards
    bool get_carrot(Eigen::Vector2f& curr_loc, float curr_angle, Eigen::Vector2f* carrot_loc);

    // run a* to plan a global path
    void plan_global_path(Eigen::Vector2f& curr_loc, float curr_angle);

    // initialization function. builds the voronoi diagram.
    void initialize(const vector_map::VectorMap& map, float goal_x, float goal_y);
};