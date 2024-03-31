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
#include <map>

using std::map;
using std::vector;
using geometry::line2f;


pair<float, float> GlobalPlanner::find_start_vertex(Eigen::Vector2f& curr_loc) {
    assert(vd_.vertices().size() != 0); // TODO: ensure this works
    double best_dist = DBL_MAX;
    voronoi_diagram<double>::vertex_type best_vertex;
    for (voronoi_diagram<double>::const_vertex_iterator it = vd_.vertices().begin(); it != vd_.vertices().end(); ++it) {
        const voronoi_diagram<double>::vertex_type &vertex = *it;
        double dist = pow(vertex.x() - curr_loc.x() * SCALE_FACTOR, 2) + pow(vertex.y() - curr_loc.y() * SCALE_FACTOR, 2);
        if(dist < best_dist) {
            best_vertex = vertex;
            best_dist = dist;
        }
    }
    return pair<float, float>(best_vertex.x(), best_vertex.y());
}



bool GlobalPlanner::get_carrot(Eigen::Vector2f& curr_loc, float curr_angle, Eigen::Vector2f* carrot_loc) {
    // decide what vertex on the path to return next
    // divide by SCALE_FACTOR to get back from "int" to float
    
    return false; // TODO: REPLACE
}



// function to run a*, smooth the path to be kinematically feasible?
void GlobalPlanner::plan_global_path(Eigen::Vector2f& curr_loc, float curr_angle) {
    // a*
    // we can iterate through voronoi diagram like this:
    // https://www.boost.org/doc/libs/1_84_0/libs/polygon/doc/voronoi_basic_tutorial.htm

    pair<float, float> start = find_start_vertex(curr_loc);

    SimpleQueue<pair<float, float>, double> frontier;
    frontier.Push(start, 0);

    map<pair<float, float>, pair<float, float>> parent;
    parent[start] = start;

    map<pair<float, float>, double> cost;
    cost[start] = 0;

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
        pair<float, float> cur = frontier.Pop();
        if(cur == goal_coords) break;
        
        list<pair<float, float>> adj_list = voronoi_edge_map_[cur];
        
        for(list<pair<float, float>>::iterator it = adj_list.begin(); it != adj_list.end(); it++) {
            pair<float, float> adj = *it;
            double dist = pow(cur.first - adj.first, 2) + pow(cur.second - adj.second, 2);
            double new_cost = cost[cur] + dist;
            map<pair<float, float>, double>::iterator entry = cost.find(adj);
            if(entry == cost.end() || entry->second > new_cost) {
                cost[adj] = new_cost;
                parent[adj] = cur;
                // TODO: check if we can memoize this somehow..?
                double heur_cost = pow(goal_coords.first - adj.first, 2) + pow(goal_coords.second - adj.second, 2);
                frontier.Push(adj, new_cost + heur_cost);
            }
        }

    }

    // follow parent dict mappings from goal to our starting vertex
    global_path_.clear();
    pair<float, float> backtrack = goal_coords;
    do {
        global_path_.push_front(backtrack);
        backtrack = parent[backtrack];
    } while(backtrack != start);

    // [MAYBE FUTURE TODO:] smooth the path along those vertices 
        // our simple carrot follower strategy handles this local smoothing for now.
}



void GlobalPlanner::initialize(const vector_map::VectorMap& map, float goal_x, float goal_y) {
    // insert the goal point into the voronoi builder
    vb_.insert_point(goal_x * SCALE_FACTOR, goal_y * SCALE_FACTOR);
    goal_coords = pair<float, float>(goal_x * SCALE_FACTOR, goal_y * SCALE_FACTOR);

    // insert map geometry into the voronoi builder
    for (size_t i = 0; i < map.lines.size(); i++) {
        const line2f map_line = map.lines[i];
        vb_.insert_segment(
            map_line.p0.x() * SCALE_FACTOR,
            map_line.p0.y() * SCALE_FACTOR,
            map_line.p1.x() * SCALE_FACTOR,
            map_line.p1.y() * SCALE_FACTOR
        );
    }

    // make the voronoi diagram
    vb_.construct(&vd_);

    // make our edge map representation
    std::map<pair<float, float>, list<pair<float, float>>> edge_map;

    // add mappings for each edge to our internal edge representation
    for (voronoi_diagram<double>::const_vertex_iterator it = vd_.vertices().begin(); it != vd_.vertices().end(); ++it) {
        const voronoi_diagram<double>::vertex_type &vertex = *it;
        list<pair<float, float>> list;
        pair<float, float> this_vertex(vertex.x(), vertex.y());
        edge_map[this_vertex] = list;

        // iterate over each of this vertex's edges
        const voronoi_diagram<double>::edge_type *edge = vertex.incident_edge();
        do {
            pair<float, float> destination_vertex(edge->vertex1()->x(), edge->vertex1()->y());

            // TODO: for pruning, check if the edge is feasible to traverse.

            // if feasible, add the destination vertex to our list
            list.push_back(destination_vertex);

            // continue iterating
            edge = edge->rot_next();
        } while(edge != vertex.incident_edge());
    }

    // find the cell the goal point "obstacle" generated. 
    for(voronoi_diagram<double>::const_cell_iterator it = vd_.cells().begin(); it != vd_.cells().end(); ++it) {
        voronoi_diagram<double>::cell_type cell = *it;
        if(cell.source_index() == 0) { 
            goal_cell = &cell;
            break;
        }
    }

    // add edges from each goal cell vertex to the goal
    voronoi_diagram<double>::edge_type* goal_edge = goal_cell->incident_edge();
    do {
        if (goal_edge->is_primary()) {
            pair<float, float> this_vertex(goal_edge->vertex0()->x(), goal_edge->vertex0()->y());
            list<pair<float, float>> list = edge_map[this_vertex];
            list.push_front(goal_coords);
        }
        goal_edge = goal_edge->next();
    } while(goal_edge != goal_cell->incident_edge());

    // replace our old map representation (if it exists) with the new map representation
    voronoi_edge_map_ = edge_map;
}



// void visualize_voronoi(amrl_msgs::VisualizationMsg & viz_msg) {
//     // draw the line segments on the viz_msg
//     // plot the  
// }