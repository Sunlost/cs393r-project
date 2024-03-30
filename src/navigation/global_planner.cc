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
#include <eigen3/Eigen/src/Core/Matrix.h>

using std::map;
using std::vector;
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

    vb_ = voronoi_builder<int32>();
    vd_ = voronoi_diagram<double>();
    global_path_ = vector<voronoi_diagram<double>::vertex_type>();

    // construct the diagram builder 
    

    // insert map geometry into the diagram builder
    for (size_t i = 0; i < map.lines.size(); i++) {
        const line2f map_line = map.lines[i];
        vb_.insert_segment(
            map_line.p0.x(),
            map_line.p0.y(),
            map_line.p1.x(),
            map_line.p1.y()
        );
    }
    
    // instantiate any other global variables we're using?
    // SimpleQueue<uint64_t, Eigen::Vector2f> queue;
}

void GlobalPlanner::set_goal(int64 x, int64 y) {
    vb_.insert_point(x, y);
}


Eigen::Vector2f GlobalPlanner::get_carrot(Vector2f& curr_loc, float curr_angle) {

    // iterate through lines on path, check for intersection with circle centered on car loc
        // specific attention to intersections biased towards the goal point 
            // (i.e. a line will have two intersections on a circle ... pick the one towards the goal)

    // maybe: as you iterate through the lines, if you find entirely-completed parts of the path, remove them from the path?


    assert(global_path_.size() > 1); // we have at least two vertices in our global path
                             
    Eigen::Vector2f edge_start_vec(0);
    Eigen::Vector2f edge_end_vec(global_path_[global_path_.size() - 1].x(), global_path_[global_path_.size() - 1].y());
    Eigen::Vector2f circle_center = curr_loc;
    double radius = 0; // TODO: change to whatever we make the size of the circle
    double squared_distance = 0; // ignored
    uint64_t index = global_path_.size() - 2;

    while(index >= 0) {
        edge_start_vec = edge_end_vec;
        edge_end_vec = (global_path_[index].x(), global_path_[index].y());
        Eigen::Vector2f intersection_point(0);

        // TODO: either rewrite this function or write our own function? 
            // a circle can intersect w/ a line in two places. need to decide which is correct.
        if(geometry::FurthestFreePointCircle(edge_start_vec, edge_end_vec, circle_center, radius, &squared_distance, &intersection_point)) {
            // we intersect -- use this as a carrot
            return intersection_point;
        }

        index--;
    }

    // amendment: I think we should handle this in the navigation controller instead for more control
    // so this stuff is not quite correct yet

    // we failed to find a valid intersection anywhere on our path -- replan and check for a carrot again.
    // plan_global_path(curr_loc, curr_angle, goal_loc, goal_angle);
    return get_carrot(curr_loc, curr_angle); // potentially dangerous code style, potential infinite loop ... probably should avoid this.
}

void GlobalPlanner::build_voronoi(Vector2f& curr_loc, float curr_angle, const Vector2f& goal_loc, float goal_angle) {
    // construct voronoi

    // add goal vertex edge to voronoi vertices with some proximity to the goal vertex
        // check kinematic feasibility of the edge with polygon-clipping alg
    
}

// function to run a*, smooth the path to be kinematically feasible?
void GlobalPlanner::plan_global_path(Vector2f& curr_loc, float curr_angle, const Vector2f& goal_loc, float goal_angle) {
    // a*
    // we can iterate through voronoi diagram like this:
    // https://www.boost.org/doc/libs/1_84_0/libs/polygon/doc/voronoi_basic_tutorial.htm

    build_voronoi(curr_loc, curr_angle, goal_loc, goal_angle);

    // will move some of this stuff into build_voronoi later
    // TODO: how do we work with the voronoi diagram wrt start/goal?
        // couple different ideas -- we need to meet to hash through them
    vb_.construct(&vd_);

    // TODO: change these.... start is closest vertex to robot? goal is closest vertex to goal?
    voronoi_diagram<double>::vertex_type start(0, 0);
    voronoi_diagram<double>::vertex_type goal(0, 0);

    SimpleQueue<uint64_t, double> frontier;
    frontier.Push(START_ID, 0);

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
                double heur_cost = pow(next.x() - goal.x(), 2) + pow(next.y() - goal.y(), 2); // squared distance
                frontier.Push(next.id(), new_cost + heur_cost);
            }
        } while(edge != cur.incident_edge());
    }

    // follow parent dict mappings from goal to our starting vertex
    voronoi_diagram<double>::vertex_type path_vertex = goal;
    global_path_.clear();
    while(path_vertex.id() != START_ID) {
        global_path_.push_back(path_vertex);
        path_vertex = parent[path_vertex.id()];
    }
    global_path_.push_back(start);

    // [MAYBE FUTURE TODO:] smooth the path along those vertices 
        // our simple carrot follower strategy handles this local smoothing for now.

}

// void visualize_voronoi(amrl_msgs::VisualizationMsg & viz_msg) {
//     // draw the line segments on the viz_msg
//     // plot the  
// }