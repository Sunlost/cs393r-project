#include "voronoi/voronoi_builder.hpp"
#include "voronoi/voronoi_diagram.hpp"
#include "vector_map/vector_map.h"
#include "simple_queue.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

using std::list;

class GlobalPlanner {
public:
    voronoi_builder<int32> vb_;
    voronoi_diagram<double> vd_;
    list<voronoi_diagram<double>::vertex_type> global_path_;

    const uint64_t START_ID = 1;
    const uint64_t GOAL_ID = 2;

    void initialize(const vector_map::VectorMap& map);
    Eigen::Vector2f get_carrot();
    void plan_global_path(Eigen::Vector2f& curr_loc, float curr_angle, const Eigen::Vector2f& goal_loc, float goal_angle);
};