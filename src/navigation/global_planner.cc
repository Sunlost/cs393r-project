#include "voronoi/voronoi_builder.hpp"
#include "voronoi/voronoi_diagram.hpp"
#include "vector_map/vector_map.h"
// #include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "simple_queue.h"
#include "global_planner.h"
#include <cstddef>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <map>
#include <set>
#include <queue>
#include "visualization/visualization.h"


using std::map;
using std::set;
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
    return pair<float, float>(best_vertex.x() / SCALE_FACTOR, best_vertex.y() / SCALE_FACTOR);
}



bool GlobalPlanner::get_carrot(Eigen::Vector2f& curr_loc, float curr_angle, Eigen::Vector2f* carrot_loc) {
    // decide what vertex on the path to return next
    // divide by SCALE_FACTOR to get back from "int" to float
    
    // return false; // TODO: REPLACE
    return global_path_.size() > 0;
}



// function to run a*, smooth the path to be kinematically feasible?
void GlobalPlanner::plan_global_path() {
    // a*
    // we can iterate through voronoi diagram like this:
    // https://www.boost.org/doc/libs/1_84_0/libs/polygon/doc/voronoi_basic_tutorial.htm

    std::cout<< "planning global path" << std::endl;

    // pair<float, float> start = find_start_vertex(curr_loc);
    // start_ = curr_loc;

    pair<float, float> start = pair<float, float>(start_.x(), start_.y());
    // print edge_map[start]
    for (list<pair<float, float>>::iterator it = voronoi_edge_map_[start].begin(); it != voronoi_edge_map_[start].end(); it++) {
        std::cout<< "edge_map[start] " << it->first << " " << it->second << std::endl;
    }

    // print start
    std::cout<< "start " << start.first << " " << start.second << std::endl;
    // print goal coords
    pair<float, float> goal(goal_.x(), goal_.y());
    std::cout<< "goal " << goal.first << " " << goal.second << std::endl;

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
        if(cur == goal) {
            std::cout << "found goal!" <<std::endl;

            break;
        }
        
        list<pair<float, float>> adj_list = voronoi_edge_map_[cur];

        // std::cout<< "adj list size " << adj_list.size() << std::endl;
        // print curr
        // std::cout<< "curr " << cur.first << " " << cur.second << std::endl;
    
        
        for(list<pair<float, float>>::iterator it = adj_list.begin(); it != adj_list.end(); it++) {
            pair<float, float> adj = *it;
            double dist = pow(cur.first - adj.first, 2) + pow(cur.second - adj.second, 2);
            double new_cost = cost[cur] + dist;
            map<pair<float, float>, double>::iterator entry = cost.find(adj);
            if(entry == cost.end() || entry->second > new_cost) {
                cost[adj] = new_cost;
                parent[adj] = cur;
                // TODO: check if we can memoize this somehow..?
                double heur_cost = pow(goal.first - adj.first, 2) + pow(goal.second - adj.second, 2);
                frontier.Push(adj, new_cost + heur_cost);
            }
        }

    }

    std::cout << "done with a*" << std::endl;

    // follow parent dict mappings from goal to our starting vertex
    global_path_.clear();
    pair<float, float> backtrack = goal;
    do {
        std::cout << "backtrack " << backtrack.first << " " << backtrack.second << std::endl;
        // check if back track is in parent
        if(parent.count(backtrack) == 0) {
            std::cout << "backtrack not in parent" << std::endl;
            break;
        }
        global_path_.push_front(backtrack);
        backtrack = parent[backtrack];
    } while(backtrack != start);
    global_path_.push_front(start);

    std::cout << "done with path" << std::endl;

    // [MAYBE FUTURE TODO:] smooth the path along those vertices 
        // our simple carrot follower strategy handles this local smoothing for now.
}

float projected_dist(const Eigen::Vector2f& point, const line2f& line) {
    Eigen::Vector2f lineVec = line.p1 - line.p0;
    Eigen::Vector2f pointVec = point - line.p0;

    // Compute the projection of pointVec onto lineVec
    float t = pointVec.dot(lineVec) / lineVec.squaredNorm();

    // Clamp to the line segment
    t = std::max(0.0f, std::min(1.0f, t));

    // Compute the projected point
    Eigen::Vector2f projectedPoint = line.p0 + t * lineVec;

    return (point - projectedPoint).norm();
}


float get_gap_size(const line2f &c1, const line2f &c2) {
    // get the minimum distance between the two lines
    float d1 = projected_dist(c1.p0, c2);
    float d2 = projected_dist(c1.p1, c2);
    float d3 = projected_dist(c2.p0, c1);
    float d4 = projected_dist(c2.p1, c1);
    float min_dist = std::min(std::min(d1, d2), std::min(d3, d4));
    return min_dist;
}


// float get_gap_size(const line2f &c1, const line2f &c2) {
//     // get the minimum distance between the two lines
//     float d1 = (c1.p0 - c2.p0).norm();
//     float d2 = (c1.p0 - c2.p1).norm();
//     float d3 = (c1.p1 - c2.p0).norm();
//     float d4 = (c1.p1 - c2.p1).norm();
//     float min_dist = std::min(std::min(d1, d2), std::min(d3, d4));
//     return min_dist;
// }

void GlobalPlanner::set_goal(float goal_x, float goal_y) {
    goal_coords_ = pair<float, float>(goal_x * SCALE_FACTOR, goal_y * SCALE_FACTOR);
    goal_ = Eigen::Vector2f(goal_x, goal_y);
}

void GlobalPlanner::set_start(float start_x, float start_y) {
    start_ = Eigen::Vector2f(start_x, start_y);
}

void GlobalPlanner::construct_map(const vector_map::VectorMap& map) {
    vb_.clear();
    vd_.clear();
    voronoi_edge_map_.clear();
    global_path_.clear();
    global_map_.clear();


    std::cout<< "initializing" << std::endl;
    // insert the goal point into the voronoi builder
    vb_.insert_point(goal_coords_.first, goal_coords_.second);
    global_map_.emplace_back(line2f(goal_.x(), goal_.y(), goal_.x(), goal_.y()));

    // insert start point into the voronoi builder
    vb_.insert_point(start_.x() * SCALE_FACTOR, start_.y() * SCALE_FACTOR);
    global_map_.emplace_back(line2f(start_.x(), start_.y(), start_.x(), start_.y()));

    global_map_.insert(global_map_.begin(), map.lines.begin(), map.lines.end());

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
    std::cout<< "here 1" << std::endl;

    // add mappings for each edge to our internal edge representation
    for (voronoi_diagram<double>::const_vertex_iterator it = vd_.vertices().begin(); it != vd_.vertices().end(); ++it) {
        const voronoi_diagram<double>::vertex_type &vertex = *it;
        // list<pair<float, float>> list;
        pair<float, float> this_vertex(vertex.x() / SCALE_FACTOR, vertex.y() / SCALE_FACTOR);
        // edge_map[this_vertex] = list;

        // std::cout << "vertex incident edge "<< vertex.incident_edge() << std::endl;

        // iterate over each of this vertex's edges
        const voronoi_diagram<double>::edge_type *edge = vertex.incident_edge();
        do {

            if (edge->is_finite()) {
                // std::cout << "edge vertex "<< edge->vertex1() << std::endl;
                pair<float, float> destination_vertex(edge->vertex1()->x() / SCALE_FACTOR, edge->vertex1()->y() / SCALE_FACTOR);

                // TODO: for pruning, check if the edge is feasible to traverse.

                // if feasible, add the destination vertex to our list
                // list.push_back(destination_vertex);
                line2f l1 = global_map_[edge->cell()->source_index()];
                line2f l2 = global_map_[edge->twin()->cell()->source_index()];
                if (get_gap_size(l1, l2) > 0.5) {
                    edge_map[this_vertex].push_back(destination_vertex);
                }
            }

            // print list size

            // continue iterating
            edge = edge->rot_next();
        } while(edge != vertex.incident_edge());
        std::cout << "list size " << edge_map[this_vertex].size() << " | " << edge_map[this_vertex].size() << " | " <<this_vertex.first<< std::endl;
    }

    std::cout<< "here 2" << std::endl;

    // find the cell the goal point "obstacle" generated. 
    bool found_start = false;
    bool found_goal = false;
    for(voronoi_diagram<double>::const_cell_iterator it = vd_.cells().begin(); it != vd_.cells().end(); ++it) {
        voronoi_diagram<double>::cell_type cell = *it;
        if(cell.source_index() == 0) { 
            goal_cell_ = &cell;
            std::cout << "found goal cell" << std::endl;
            found_goal = true;
        }
        else if (cell.source_index() == 1) {
            std::cout << "found start cell" << std::endl;
            start_cell_ = &cell;
            found_start = true;
        }
        
        if (found_start && found_goal) {
            break;
        }
    }

    std::cout<< "here 3" << std::endl;

    // iterate through edge_map and print the size of the list
    // for (std::map<pair<float, float>, list<pair<float, float>>>::iterator it = edge_map.begin(); it != edge_map.end(); ++it) {
    //     std::cout << "vertex " << it->first.first << " " << it->first.second << " has " << it->second.size() << " edges" << std::endl;
    // }

    // add edges from each goal cell vertex to the goal
    voronoi_diagram<double>::edge_type* goal_edge = goal_cell_->incident_edge();
    pair<float, float> goal(goal_.x(), goal_.y());
    do {
        if (goal_edge->is_primary() && goal_edge->is_finite()) {
            // goal_cell_->incident_edge() = nullptr;
            // check that the point is not nan
            if (!std::isnan(goal_edge->vertex0()->x()) && !std::isnan(goal_edge->vertex0()->y())) {
                // std::cout<< "here 3.1 " << goal_cell_->incident_edge_<< std::endl;
                pair<float, float> this_vertex(goal_edge->vertex0()->x() / SCALE_FACTOR, goal_edge->vertex0()->y() / SCALE_FACTOR);
                edge_map[this_vertex].push_front(goal);
                // print this_vertex and print if 10, 10 is in the list
                std::cout << "goal this_vertex " << this_vertex.first << " " << this_vertex.second << std::endl;
            }
        }
        // std::cout << "goal edge "<< goal_edge <<" incident edge " << goal_cell_->incident_edge() << " cell "<< goal_cell_<< std::endl;
        goal_edge = goal_edge->next();
    } while(goal_edge != goal_cell_->incident_edge());


    // add edges from start cell to each connecting vertex
    voronoi_diagram<double>::edge_type* start_edge = start_cell_->incident_edge();
    pair<float, float> start(start_.x(), start_.y());
    do {
        if (start_edge->is_primary() && start_edge->is_finite()) {
            // goal_cell_->incident_edge() = nullptr;
            // check that the point is not nan
            if (!std::isnan(start_edge->vertex1()->x()) && !std::isnan(start_edge->vertex1()->y())) {
                // std::cout<< "here 3.1 " << goal_cell_->incident_edge_<< std::endl;
                pair<float, float> this_vertex(start_edge->vertex1()->x() / SCALE_FACTOR, start_edge->vertex1()->y() / SCALE_FACTOR);
                edge_map[start].push_back(this_vertex);
                // print this_vertex and print if 10, 10 is in the list
                std::cout << "start this_vertex " << this_vertex.first << " " << this_vertex.second << std::endl;
            }
        }
        // std::cout << "goal edge "<< goal_edge <<" incident edge " << goal_cell_->incident_edge() << " cell "<< goal_cell_<< std::endl;
        start_edge = start_edge->next();
    } while(goal_edge != start_cell_->incident_edge());

    // print start
    std::cout<< "start " << start.first << " " << start.second << std::endl;

    // replace our old map representation (if it exists) with the new map representation
    voronoi_edge_map_ = edge_map;
    std::cout<< "done initializing" << std::endl;
}

void GlobalPlanner::visualize_global_plan(amrl_msgs::VisualizationMsg & viz_msg, uint32_t color) {
    Eigen::Vector2f prev(global_path_.front().first, global_path_.front().second);
    for (const auto& vertex : global_path_) {
        Eigen::Vector2f p(vertex.first, vertex.second);
        // visualization::DrawCross(p, .1, color, viz_msg);
        visualization::DrawLine(prev, p, color, viz_msg);
        prev = p;
    }
    visualization::DrawCross(Eigen::Vector2f(global_path_.back().first, global_path_.back().second), .2, 0x0000ff, viz_msg);
    visualization::DrawCross(Eigen::Vector2f(global_path_.front().first, global_path_.front().second), .2, 0x00f0f0, viz_msg);

}


void GlobalPlanner::visualize_voronoi(amrl_msgs::VisualizationMsg & viz_msg, uint32_t color) {
    std::cout << "visualizing" << std::endl;
    // draw the line segments on the viz_msg
    // for (const auto& edge : vd_.edges()) {
    //     if (edge.is_finite() && edge.is_linear()) {
    //         Eigen::Vector2f p0(edge.vertex0()->x(), edge.vertex0()->y());
    //         Eigen::Vector2f p1(edge.vertex1()->x(), edge.vertex1()->y());
    //         // visualization::DrawLine(p0 / SCALE_FACTOR, p1 / SCALE_FACTOR, color, viz_msg);
    //         visualization::DrawCross(p0 / SCALE_FACTOR, .1, color, viz_msg);
    //         visualization::DrawCross(p1 / SCALE_FACTOR, .1, color, viz_msg);
    //     }
    // }

    // Initialize all nodes as not visited
    for (const auto& node : voronoi_edge_map_) {
        // visited[node.first] = false;
        Eigen::Vector2f p(node.first.first, node.first.second);
        for (const auto& adjacent : node.second) {
            Eigen::Vector2f adj(adjacent.first, adjacent.second);
            visualization::DrawLine(p, adj, 0x0000ff, viz_msg);
        }
    }
}