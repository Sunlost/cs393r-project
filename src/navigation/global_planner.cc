// math headers
#include "shared/math/line2d.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include <eigen3/Eigen/src/Core/Matrix.h>

// other project file headers
#include "vector_map/vector_map.h"
#include "amrl_msgs/msg/localization2_d_msg.hpp"
#include "gflags/gflags.h"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "amrl_msgs/msg/visualization_msg.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "simple_queue.h"
#include "global_planner.h"

// visualization header
#include "visualization/visualization.h"

// voronoi headers (from boost) 
#include "voronoi/voronoi_builder.hpp"
#include "voronoi/voronoi_diagram.hpp"

// std lib headers
#include <cstddef>
#include <map>
#include <queue>


using std::map;
using std::vector;
using geometry::line2f;


///////////////////////////////////////////////////////////////////////////////
//                             PRIVATE FUNCTIONS                             //
///////////////////////////////////////////////////////////////////////////////


// returns smallest dist between point and line via point projection
float projected_dist(const Eigen::Vector2f& point, const line2f& line) {
    Eigen::Vector2f lineVec = line.p1 - line.p0;
    Eigen::Vector2f pointVec = point - line.p0;

    // Compute the projection of pointVec onto lineVec
    float t = pointVec.dot(lineVec) / lineVec.squaredNorm();

    // Clamp to the line segment
    t = std::max(0.0f, std::min(1.0f, t));

    // Compute the projected point
    Eigen::Vector2f projectedPoint = line.p0 + t * lineVec;

    // Return distance between point and point projected on line (e.g. the gap)
    return (point - projectedPoint).norm();
}



// returns smallest gap size between two lines
// float get_gap_size(const line2f &c1, const line2f &c2) {
//     // get the minimum distance between the two lines
//     float d1 = projected_dist(c1.p0, c2);
//     float d2 = projected_dist(c1.p1, c2);
//     float d3 = projected_dist(c2.p0, c1);
//     float d4 = projected_dist(c2.p1, c1);
//     float min_dist = std::min(std::min(d1, d2), std::min(d3, d4));
//     return min_dist;
// }

float get_gap_size(const Eigen::Vector2f &c1, const Eigen::Vector2f &c2) {
    // // get the minimum distance between the two lines
    // float d1 = projected_dist(c1.p0, c2);
    // float d2 = projected_dist(c1.p1, c2);
    // float d3 = projected_dist(c2.p0, c1);
    // float d4 = projected_dist(c2.p1, c1);
    // float min_dist = std::min(std::min(d1, d2), std::min(d3, d4));
    // return min_dist;
    return (c1 - c2).norm();
}



// return boolean stating if passed point is inside passed cell
bool inside_cell(const voronoi_diagram<double>::cell_type * cell, const Eigen::Vector2f& point) {
    bool inside = false;

    // Ray casting algorithm
    const voronoi_diagram<double>::edge_type* edge = cell->incident_edge();
    do {
        if (edge->is_primary() && edge->is_finite()) {
            float x1 = edge->vertex0()->x();
            float y1 = edge->vertex0()->y();
            float x2 = edge->vertex1()->x();
            float y2 = edge->vertex1()->y();
            if (((y1 > point.y()) != (y2 > point.y())) &&
                (point.x() < (x2 - x1) * (point.y() - y1) / (y2 - y1) + x1)) {
                inside = !inside;
            }
        }
        edge = edge->next();
    } while(edge != cell->incident_edge());

    return inside;
}



bool is_valid_point(const voronoi_diagram<double>::vertex_type* point) {
    return !std::isnan(point->x()) && !std::isnan(point->y()) && !std::isinf(point->x()) && !std::isinf(point->y());
}


///////////////////////////////////////////////////////////////////////////////
//                             PUBLIC FUNCTIONS                              //
///////////////////////////////////////////////////////////////////////////////


// sets the goal point
void GlobalPlanner::set_goal(float goal_x, float goal_y) {
    goal_coords_ = pair<float, float>(goal_x * SCALE_FACTOR, goal_y * SCALE_FACTOR);
    goal_ = Eigen::Vector2f(goal_x, goal_y);
}



// sets the start point
void GlobalPlanner::set_start(float start_x, float start_y) {
    start_ = Eigen::Vector2f(start_x, start_y);
}

// Function to perform linear interpolation between two Eigen Vector2f endpoints
vector<Eigen::Vector2f> interpolate_points(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2, float spacing_distance) {
    vector<Eigen::Vector2f> interpolated_points;

    // Calculate the total distance between p1 and p2
    float total_distance = (p2 - p1).norm();

    // Calculate the number of segments based on spacing distance
    int num_segments = std::max(1, static_cast<int>(total_distance / spacing_distance));

    // Calculate the increment for parameter t
    float delta_t = 1.0f / num_segments;

    for (int i = 1; i <= num_segments; ++i) {
        float t = i * delta_t;
        Eigen::Vector2f interpolated_point = p1 + t * (p2 - p1);
        interpolated_points.push_back(interpolated_point);
    }

    return interpolated_points;
}

void GlobalPlanner::construct_map(const vector_map::VectorMap& map) {
    vb_.clear();
    vd_.clear();
    global_path_.clear();
    global_map_.clear();

    // insert the goal point into the voronoi builder
    vb_.insert_point(goal_coords_.first, goal_coords_.second);
    global_map_.push_back(goal_);

    // insert start point into the voronoi builder
    vb_.insert_point(start_.x() * SCALE_FACTOR, start_.y() * SCALE_FACTOR);
    global_map_.push_back(start_);

    // insert map geometry into the voronoi builder
    for (size_t i = 0; i < map.lines.size(); i++) {
        const line2f map_line = map.lines[i];
        vector<Eigen::Vector2f> vec = interpolate_points(map_line.p0, map_line.p1, .2);
        for (const auto & p : vec) {
            vb_.insert_point(
                p.x() * SCALE_FACTOR,
                p.y() * SCALE_FACTOR
            );
            global_map_.push_back(p);
        }
    }

    // construct the voronoi diagram
    vb_.construct(&vd_);

    // make our new edge map representation
    std::map<pair<float, float>, list<pair<float, float>>> edge_map;

    // add mappings for each edge to our internal edge representation
    for (voronoi_diagram<double>::const_vertex_iterator it = vd_.vertices().begin(); 
            it != vd_.vertices().end(); ++it) {
       
        const voronoi_diagram<double>::vertex_type &vertex = *it;

        if(!is_valid_point(&vertex)) continue;

        pair<float, float> this_vertex(vertex.x() / SCALE_FACTOR, vertex.y() / SCALE_FACTOR);

        // iterate over each of this vertex's edges
        const voronoi_diagram<double>::edge_type *edge = vertex.incident_edge();
        do {
            if (edge->is_finite() && is_valid_point(edge->vertex1())) {
                pair<float, float> destination_vertex(edge->vertex1()->x() / SCALE_FACTOR, 
                                                      edge->vertex1()->y() / SCALE_FACTOR);

                // for pruning, check if the edge is feasible to traverse.
                // if feasible, add the destination vertex to our list
                
                // cells are built around a source (obstacle). source_index is 
                // the order the obstacles were added to the map
                Eigen::Vector2f l1 = global_map_[edge->cell()->source_index()];
                Eigen::Vector2f l2 = global_map_[edge->twin()->cell()->source_index()];
                if (get_gap_size(l1, l2) > 0.5) {
                    edge_map[this_vertex].push_back(destination_vertex);
                }
            }
            edge = edge->rot_next();
        } while(edge != vertex.incident_edge());
    }

    // find the cell the goal point "obstacle" generated. 
    bool found_start = false;
    bool found_goal = false;
    for(voronoi_diagram<double>::const_cell_iterator it = vd_.cells().begin(); 
        it != vd_.cells().end(); ++it) {

        if(it->source_index() == 0) { 
            goal_cell_ = &(*it);
            found_goal = true;
        }
        else if (it->source_index() == 1) {
            start_cell_ = &(*it);
            found_start = true;
        }
        
        if (found_start && found_goal) break;
    }

    // add edges from each goal cell vertex to the goal
    const voronoi_diagram<double>::edge_type* goal_edge = goal_cell_->incident_edge();
    pair<float, float> goal(goal_.x(), goal_.y());
    do {
        if (goal_edge->is_primary() && goal_edge->is_finite()) {
            const voronoi_diagram<double>::vertex_type* vertex = goal_edge->vertex0();
            if (!std::isnan(vertex->x()) && !std::isnan(vertex->y())) {
                pair<float, float> this_vertex(vertex->x() / SCALE_FACTOR, vertex->y() / SCALE_FACTOR);
                edge_map[this_vertex].push_front(goal);
            }
        }
        goal_edge = goal_edge->next();
    } while(goal_edge != goal_cell_->incident_edge());


    // add edges from start cell to each connecting vertex
    const voronoi_diagram<double>::edge_type* start_edge = start_cell_->incident_edge();
    pair<float, float> start(start_.x(), start_.y());
    do {
        if (start_edge->is_primary() && start_edge->is_finite()) {
            const voronoi_diagram<double>::vertex_type* vertex = start_edge->vertex1();
            if (!std::isnan(vertex->x()) && !std::isnan(vertex->y())) {
                pair<float, float> this_vertex(vertex->x() / SCALE_FACTOR, vertex->y() / SCALE_FACTOR);
                edge_map[start].push_back(this_vertex);
            }
        }
        start_edge = start_edge->next();
    } while(start_edge != start_cell_->incident_edge());

    // replace our old map representation (if it exists) with the new map representation
    voronoi_edge_map_ = edge_map;
}



// a* pseudocode
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



// function to run a*
void GlobalPlanner::plan_global_path() {
    pair<float, float> start = pair<float, float>(start_.x(), start_.y());
    pair<float, float> goal(goal_.x(), goal_.y());

    SimpleQueue<pair<float, float>, double> frontier;
    frontier.Push(start, 0);

    map<pair<float, float>, pair<float, float>> parent;
    parent[start] = start;

    map<pair<float, float>, double> cost;
    cost[start] = 0;

    bool goal_found = false;

    while(!frontier.Empty()) {
        pair<float, float> cur = frontier.Pop();
        if(cur == goal) {
            // if the goal is unreachable, bad things happen. so we set a flag for whether goal was reachable
            goal_found = true;
            break;
        };
        
        list<pair<float, float>> adj_list = voronoi_edge_map_[cur];
        
        for(list<pair<float, float>>::iterator it = adj_list.begin(); it != adj_list.end(); it++) {
            pair<float, float> adj = *it;
            double dist = sqrt(pow(cur.first - adj.first, 2) + pow(cur.second - adj.second, 2));
            double new_cost = cost[cur] + dist;
            map<pair<float, float>, double>::iterator entry = cost.find(adj);
            if(entry == cost.end() || entry->second > new_cost) {
                cost[adj] = new_cost;
                parent[adj] = cur;
                // TODO: check if we can memoize this somehow..?
                double heur_cost = sqrt(pow(goal.first - adj.first, 2) + pow(goal.second - adj.second, 2));
                frontier.Push(adj, new_cost + heur_cost);
            }
        }
    }

    // follow parent dict mappings from goal to our starting vertex to build path
    if (goal_found) {
        global_path_.clear();
        pair<float, float> backtrack = goal;
        do {
            global_path_.push_front(backtrack);
            backtrack = parent[backtrack];
        } while(backtrack != start);
        // global_path_.push_front(start);
    }
    
    return;
}



// return next carrot point
bool GlobalPlanner::get_carrot(Eigen::Vector2f& curr_loc, float curr_angle, Eigen::Vector2f* carrot_loc, amrl_msgs::msg::VisualizationMsg & viz_msg) {
    // decide what vertex on the path to return next
    // divide by SCALE_FACTOR to get back from "int" to float
    // print get_carrot called
    // std::cout << "Get Carrot Called" << std::endl;
    if (global_path_.size() == 0) {
        return false;
    }

    if (global_path_.size() == 2) { // robot can go directly to goal, so set goal as carrot.
        carrot_loc->x() = global_path_.back().first;
        carrot_loc->y() = global_path_.back().second;
        return true;
    }
    else {

        // carrot_loc->x() = global_path_.front().first;
        // carrot_loc->y() = global_path_.front().second;

         // Iterate backwards using reverse iterators
        for (auto rit = global_path_.rbegin(); rit != global_path_.rend(); ++rit) {
            carrot_loc->x() = rit->first;
            carrot_loc->y() = rit->second;
            if ((curr_loc - *carrot_loc).norm() < 1) { // can change min dist here
                break;
            }
        }
    }

    // print the carrot
    // std::cout << "Carrot: " << carrot_loc->x() << " " << carrot_loc->y() << std::endl;
    return true;
}


///////////////////////////////////////////////////////////////////////////////
//                          VISUALIZATION FUNCTIONS                          //
///////////////////////////////////////////////////////////////////////////////


// visualize our plan on the map
void GlobalPlanner::visualize_global_plan(amrl_msgs::msg::VisualizationMsg & viz_msg, uint32_t color) {
    Eigen::Vector2f prev(global_path_.front().first, global_path_.front().second);
    for (const auto& vertex : global_path_) {
        Eigen::Vector2f p(vertex.first, vertex.second);
        visualization::DrawCross(p, .1, color, viz_msg);
        visualization::DrawLine(prev, p, color, viz_msg);
        prev = p;
    }
    visualization::DrawCross(Eigen::Vector2f(global_path_.back().first, global_path_.back().second), .2, 0x0000ff, viz_msg);
    visualization::DrawCross(Eigen::Vector2f(global_path_.front().first, global_path_.front().second), .2, 0x00f0f0, viz_msg);
}


// visualize our voronoi diagram on the map
void GlobalPlanner::visualize_voronoi(amrl_msgs::msg::VisualizationMsg & viz_msg, uint32_t color) {
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
            visualization::DrawLine(p, adj, 0x007777, viz_msg);
        }
    }

    // draw the start_cell in blue
    const voronoi_diagram<double>::edge_type* edge = start_cell_->incident_edge();
    do {
        if (edge->is_primary() && edge->is_finite()) {
            // draw it
            Eigen::Vector2f p0(edge->vertex0()->x(), edge->vertex0()->y());
            p0 /= SCALE_FACTOR;
            Eigen::Vector2f p1(edge->vertex1()->x(), edge->vertex1()->y());
            p1 /= SCALE_FACTOR;
            visualization::DrawLine(p0, p1, 0x0000ff, viz_msg);
        }
        edge = edge->next();
    } while(edge != start_cell_->incident_edge());
}
