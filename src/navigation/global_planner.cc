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
#include <cstddef>

using std::map;
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


    // TODO: change these.... start is closest vertex to robot? goal is closest vertex to goal?
    voronoi_diagram<double>::vertex_type start(0, 0);
    voronoi_diagram<double>::vertex_type goal(0, 0);

    SimpleQueue<uint64_t, double, double> frontier;
    frontier.Push(START_ID, 0, 0);

    uint64_t next_id = 3; // next id a previously-unidentified vertex will use
    map<uint64_t, voronoi_diagram<double>::vertex_type> id_map;
    id_map[START_ID] = start;
    id_map[GOAL_ID] = goal;

    map<uint64_t, voronoi_diagram<double>::vertex_type> parent;
    parent[START_ID] = start;

    map<uint64_t, double> cost;
    cost[START_ID] = 0;

    // parent map : node -> parent node (where it came from on lowest-cost path)
    // cost map : node -> lowest cost found 

    // push start of voronoi diagram to frontier queue
        // identify closest voronoi vertex

    // while queue not empty
        // get head of queue, A
        // if A is goal, we're done
            // break
        // iterate over A's neighbors
            // new cost = current cost + cost of edge from A to A' (neighbor)
            // if A' not already in cost, or new cost is better than A's old cost
                // insert A' into cost if needed, new value is new cost
                // insert A' into parent if needed, new value is A
                // add A' to queue, with priority value [ new cost + heuristic(A') ]

    while(!frontier.Empty()) {
        uint64_t cur_id = frontier.Pop();
        if(cur_id == GOAL_ID) break;
        voronoi_diagram<double>::vertex_type cur = id_map[cur_id];
        voronoi_diagram<double>::edge_type *edge = cur.incident_edge();
        do {
            // should always be vertex1? see voronoi_diagram.hpp:168-177
            voronoi_diagram<double>::vertex_type next = *(edge->vertex1());
            double dist = pow(cur.x() - next.x(), 2) + pow(cur.y() - next.y(), 2); // squared distance
            double new_cost = cost[cur_id] + dist;
            if(next.id() == 0) {
                next.set_id(next_id++);
                id_map[next.id()] = next;
            }
            map<uint64_t, double>::iterator entry = cost.find(next.id());
            if(entry == cost.end() || entry->second > new_cost) {
                cost[next.id()] = new_cost;
                parent[next.id()] = cur;
                frontier.Push(next.id(), new_cost, 0); // TODO: add heuristic in place of 0
            }
        } while(edge != cur.incident_edge());
    }


    // follow parent dict mappings from goal to our starting vertex

    // [MAYBE FUTURE TODO:] smooth the path along those vertices 
        // our simple carrot follower strategy handles this local smoothing for now.

    // establish/communicate carrot goal?


    // =====================================================================================


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