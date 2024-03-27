#include "voronoi/voronoi_builder.hpp"
#include "voronoi/voronoi_diagram.hpp"
#include "vector_map/vector_map.h"
#include "simple_queue.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

class GlobalPlanner {
public:
    voronoi_builder<int32> vb_;
    voronoi_diagram<double> vd_;
    SimpleQueue<voronoi_diagram<double>::vertex_type, uint64_t, uint64_t> queue_;

    // GlobalPlanner();

    void initialize(const vector_map::VectorMap& map);
    void run_global_planner();
    // void build_diagram();
    void plan_global_path(Eigen::Vector2f& curr_loc, float curr_angle, const Eigen::Vector2f& goal_loc, float goal_angle);
};