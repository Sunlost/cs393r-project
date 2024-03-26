#include "voronoi/voronoi_builder.hpp"
#include "voronoi/voronoi_diagram.hpp"

#include "vector_map/vector_map.h"

#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "simple_queue.h"

#include "global_planner.h"

using Eigen::Vector2f;
using geometry::line2f;

using namespace math_util;

// GlobalPlanner::GlobalPlanner() :
//     vb_(nullptr),
//     vd_(nullptr),
//     queue_(nullptr)
// {};

void GlobalPlanner::initialize(const vector_map::VectorMap& map) {
    //         vb_(voronoi_builder<int32>),
        // vd_(voronoi_diagram<double>),
        // queue_(nullptr, 0, 0)

    // construct the diagram builder 


    // insert map gesometry into the diagram builder
    for (size_t i = 0; i < map.lines.size(); i++) {
        const line2f map_line = map.lines[i];
        vb_.insert_segment(
            map_line.p0.x(),
            map_line.p0.y(),
            map_line.p1.x(),
            map_line.p1.y()
        );
    }
    
    vb_.construct(&vd_);
    // instantiate any other global variables we're using?
    // SimpleQueue<uint64_t, Eigen::Vector2f> queue;
}



// overarching function to handle global planning process
void GlobalPlanner::run_global_planner() {


}

// function to run a*, smooth the path to be kinematically feasible?
void GlobalPlanner::plan_global_path(Vector2f& curr_loc, float curr_angle, const Vector2f& goal_loc, float goal_angle) {
    // a*
    // we can iterate through voronoi diagram like this:
    // https://www.boost.org/doc/libs/1_84_0/libs/polygon/doc/voronoi_basic_tutorial.htm

    // so we're not guaranteed that our starting location and goal location are on edges in our voronoi diagram
    // to deal with that, let's find the closest point between our desired location and a voronoi edge and call that our start and our goal
    // and! we have to make sure we don't run into a wall in order to get to/leave that edge
    voronoi_diagram<double>::vertex_type &start = voronoi_diagram<double>::voronoi_vertex();
    voronoi_diagram<double>::vertex_type &goal = voronoi_diagram<double>::voronoi_vertex();

    // push start of voronoi diagram to queue
    queue_.Push();

    // instantiate parent vector of voronoi nodes and cost vector
    // set cost for start to 0

    // while (!queue_.Empty()) {
    //     voronoi_diagram<double>::vertex_type &current = queue_.Pop()
    //     // somehow we need to have some notion of car angleyness... TODO: that
    //     if (current.x() == goal_loc.x() && current.y() == goal_loc.y()) {
    //         break;
    //     }

    //     const voronoi_diagram<double>::edge_type *edge = current.incident_edge();
    //     edge.        
    //     do {
    //     if (edge->is_primary())
    //         ++result;
    //     edge = edge->rot_next();
    //     } while (edge != vertex.incident_edge());
    //     }
    // 
}

// void visualize_voronoi(amrl_msgs::VisualizationMsg & viz_msg) {
//     // draw the line segments on the viz_msg
//     // plot the 
    
// }