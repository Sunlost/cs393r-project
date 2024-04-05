#include "path_options.h"
#include <cstdio>
#include <algorithm>
#include <iostream>
using std::min;
// #include <cmath>
// 1d time optimal control
// given distance to go, max decel, max vel
// out vel

// do this on path option after selecting it

const double clearance_cap = .005;

float run1DTimeOptimalControl(float dist_to_go, float current_speed, const navigation::NavigationParams& robot_config) {
    float max_accel = robot_config.max_accel;
    float max_decel = robot_config.max_decel;
    float max_vel = robot_config.max_vel;
    float dt = robot_config.dt;
    float cruise_stopping_dist = pow(current_speed, 2) / (2 * max_decel);
    float accel_stopping_dist = pow(current_speed + dt * max_accel, 2) / (2 * max_decel);
    // std::cout << "Current Speed: " << current_speed << std::endl;
    // std::cout << "Max Velocity: " << max_vel << std::endl;
    // std::cout << "dt: " << dt << std::endl;
    // std::cout << "Cruise Stopping Distance: " << cruise_stopping_dist << std::endl;
    // std::cout << "Dist to go: " << dist_to_go << std::endl;

    // if dist_to_go is larger than stopping_dist and not at max vel, can accelerate
    if (dist_to_go > accel_stopping_dist && current_speed < max_vel) {
        return std::min(max_vel, current_speed + max_accel * dt);
    }
    else if (dist_to_go > cruise_stopping_dist && current_speed == max_vel) {  // can stop in time and at max vel
                                                                        // probably needs hysteresis
        return current_speed;
    }
    else {  // otherwise needs to decelerate
        return std::max(current_speed - max_decel * dt, 0.0f);
    }
}



// set curvature, free path length, obstruction for a path option
void setPathOption(navigation::PathOption& path_option,
                        float curvature, const vector<Eigen::Vector2f>& point_cloud,
                        const navigation::NavigationParams& robot_config,
                        Eigen::Vector2f robot_rel_carrot) {
    path_option.curvature = curvature;
    float h = robot_config.length - robot_config.base_link_offset; // distance from base link to front bumper
    if (curvature == 0) {
        for (auto p: point_cloud) {
            if (robot_config.width/2 + robot_config.safety_margin >= abs(p[1])
                && p[0] < path_option.free_path_length) {
                path_option.free_path_length = p[0] - h - robot_config.safety_margin;
                path_option.obstruction = p;
            }
        }
        // clearance
        for (auto p: point_cloud) {
            if (p[0] >=0 and p[0] < path_option.free_path_length) {
                float clearance_p = abs(p[1]) - robot_config.width / 2 - robot_config.safety_margin;
                if (clearance_p > clearance_cap) 
                    clearance_p = clearance_cap; // set clearance to c_max because we don't care at some point
                if (clearance_p < path_option.clearance) {
                    path_option.clearance = clearance_p;
                    //path_option.closest_point = p;
                }
            }
        }
        return;
    }

    Vector2f c = Vector2f(0, 1 / curvature);
    float r_inner = c.norm() - robot_config.width / 2 - robot_config.safety_margin;
    float r_outer = c.norm() + robot_config.width / 2 + robot_config.safety_margin;
    float r_tl = (Vector2f(0, r_inner) - Vector2f(h + robot_config.safety_margin, 0)).norm();
    float r_tr = (Vector2f(0, r_outer) - Vector2f(h + robot_config.safety_margin, 0)).norm();
    float r_br = (Vector2f(0, r_outer) - Vector2f(robot_config.base_link_offset + robot_config.safety_margin, 0)).norm();
    path_option.free_path_length = std::min(M_PI * c.norm(), 5.0);  // some large number
    // float omega = atan2(h, r_inner);

    float theta_br = asin(robot_config.base_link_offset + robot_config.safety_margin / r_br); // angle where back right would hit
    float phi = 0;
    //	cout << "curvature " << curvature << endl;
    //	bool front_side = false, outer_side = false, inner_side = false;
    for (unsigned int i = 0; i < point_cloud.size(); i++) {
        Vector2f p = point_cloud[i];
        float r_p = (c-p).norm();
        float theta = curvature < 0 ? atan2(p[0], p[1]- c[1]) : atan2(p[0], c[1] - p[1]); // angle between p and c
        float length = 5.0;
        // cout << "curvature " << curvature << endl;
        if (r_inner <= r_p && r_p <= r_tl) {    // inner side hit
                phi = acos(r_inner / r_p);
                length = (theta - phi) * c.norm();
            // inner_side = true;
                // cout << "inner side hit" << endl;
        }
        if ((r_inner <= r_p && r_p <= r_br) && (-theta_br <= theta && theta <= theta_br)) {    // outer side hit
            phi = acos(r_p / (c.norm() + robot_config.width / 2));
            length = (theta - phi) * c.norm();
	    // outer_side = true;
	    // cout << "outer side hit" << endl;
        }

        if (r_tl <= r_p && r_p <= r_tr) {    // front side hit
            phi = asin(h / r_p);
            length = (theta - phi) * c.norm();
	    // front_side = true;
	    // cout << "front side hit" << endl;
        }

        // At this point the feasible fpl has been calculated like we want it.
        if (length < path_option.free_path_length && length > 0) {
            path_option.free_path_length = length;
            path_option.obstruction = p;
        }
    }
	// if (inner_side)
	//  	cout << "intersecting particle found with inner side" << endl;
	// if (outer_side)
	//	cout << "intersecting particle found with outer side" << endl;
	//if (front_side)
	//	cout << "intersecting particle found with front side" << endl;

    // float theta = M_PI / 2;
    // if (path_option.obstruction != Eigen::Vector2f::Zero()) {
    //     theta = curvature < 0 ? atan2(path_option.obstruction[0], path_option.obstruction[1]- c[1]) :
    //         atan2(path_option.obstruction[0], c[1] - path_option.obstruction[1]);
    // }
    // clearance
    // path_option.clearance = 100; // some large number
    for (auto p: point_cloud) {
        float theta_p =  curvature < 0 ? atan2(p[0], p[1]- c[1]) :
            atan2(p[0], c[1] - p[1]);
        float path_len_p = theta_p * (p-c).norm();
        if (path_len_p >=0 and path_len_p < path_option.free_path_length) {  // if p is within the fp length
            float inner = abs((c - p).norm() - r_inner);
            float outer = abs((c - p).norm() - r_tr);
            float clearance_p = std::min(inner, outer);
            if (clearance_p < path_option.clearance) {
                path_option.clearance = clearance_p;
                //path_option.closest_point = p;
            }
        }
    }

    // NEW: Adding code to calculate the closest point properly, may need to look at this whole method
    // again to optimize + simplify 
    // deal with the map-relative carrot point
    // zero out with the carrot point to get it robot-relative
    // need to find the deltas, for now we won't use any angles tho.
    // the x and y deltas are the robot-relative carrot.
    // float angle_delta = carrot_angle - carrot_angle;
    
    Vector2f temp_endpt(INFINITY, INFINITY);
    double temp_fpl = INFINITY;

    //go thru the point cloud to set the closest_point ONLY
    bool mirrored = false;
    float w = robot_config.width/2 + robot_config.safety_margin;
    for (auto p: point_cloud) {
        double radius = 1.0 / curvature;
        if (radius < 0.0) {
            mirrored = true;
            radius = -1 * radius; 
        } 

        Eigen::Vector2f center(0, radius); // right = negative value
        double goal_mag = (robot_rel_carrot - center).norm();

        // fpl = f(c, p) if c > 0
        // fpl = f(-c, Mirrored(p)) if c < 0, we flip our curve back into the +ve, and flip points in cloud y

        // Straight path init vals
        Eigen::Vector2f optimal_endpt(robot_rel_carrot.x(), 0);
        // could change cmp_opt_fpl to path_option.free_path_length, but for now we only want to change closest point
        double cmp_opt_fpl = robot_rel_carrot.x();

        // if curved, the endpoint is not just straight ahead of the robot.
        if (curvature != 0) {
            // path is curved.
            optimal_endpt.x() = center.x() + (robot_rel_carrot.x() - center.x()) / goal_mag * radius;
            optimal_endpt.y() = center.y() + (robot_rel_carrot.y() - center.y()) / goal_mag * radius;
            cmp_opt_fpl = (2 * radius) * asin(optimal_endpt.norm() / (2 * radius)); // init to some high value
        } 

        // flip the point cloud across the x axis
        Eigen::Vector2f point(p.x(), p.y());
        if (mirrored) point.y() = -1 * p.y();

        // now the math should work as we know it should, for curves
        if (curvature != 0) {
            double r_1 = radius - w;
            Eigen::Vector2f r_2_v(radius + w, h);
            double r_2 = r_2_v.norm();
            double omega = atan2(h, radius - w);
            Eigen::Vector2f mag_v(p.x() - center.x(), p.y() - center.y());
            double mag = mag_v.norm();
            double theta = atan2(p.x(), radius - p.y());
            double phi = (theta - omega);

            // rotate r from (0,r) by phi radians to find the endpoint
            Eigen::Affine2f rotate_phi = Eigen::Translation2f(0, radius) * Eigen::Rotation2Df(phi);
            Eigen::Vector2f circle_center(0, -radius);
            Eigen::Vector2f obstructed_endpt = rotate_phi * circle_center; // this wrong, need to use affines.
            double obstructed_fpl = radius * phi; // need to find where this point is in the graph

            // this point is an obstruction for this path
            if ((mag >= r_1 && mag <= r_2) && theta > 0) {
                Eigen::Affine2f translate_center = Eigen::Translation2f(0, -radius) * Eigen::Rotation2Df(0);
                Eigen::Vector2f center_optimal_endpt = translate_center * optimal_endpt;
                double optimal_central_angle = abs(atan(center_optimal_endpt.x() / center_optimal_endpt.y()));
                double optimal_fpl = optimal_central_angle * radius;

                if (obstructed_fpl < path_option.free_path_length) {
                    temp_fpl = obstructed_fpl;
                    temp_endpt = obstructed_endpt;
                } else {
                    temp_fpl = optimal_fpl;
                    temp_endpt = optimal_endpt;
                }

                // if (mirrored) visualization::DrawCross(Eigen::Vector2f (temp_endpt.x(), -1 * temp_endpt.y()), .1, 0xFF0000, local_viz_msg_);
                // else visualization::DrawCross(Eigen::Vector2f (temp_endpt.x(), temp_endpt.y()), .1, 0xFF0000, local_viz_msg_);

                if (temp_fpl < cmp_opt_fpl) {
                    // could switch to path_option's fpl
                    cmp_opt_fpl = min(cmp_opt_fpl, temp_fpl); // need to do same 3 way min for end of path
                    temp_endpt.y() = (mirrored) ? -1 * temp_endpt.y() : temp_endpt.y();
                    path_option.closest_point = temp_endpt;
                }
            }
        }
    }
}


// sample path options
// given point cloud (robot frame), num options, max curvature
// out const vector path options

vector<navigation::PathOption> samplePathOptions(int num_options,
                                                    const vector<Eigen::Vector2f>& point_cloud,
                                                    const navigation::NavigationParams& robot_config,
                                                    Eigen::Vector2f robot_rel_carrot) {
    static vector<navigation::PathOption> path_options;
    path_options.clear();
    float max_curvature = robot_config.max_curvature;

    // loop through curvature from max to -max
    for (int i = 0; i < num_options; i++) { 
        float curvature = max_curvature * pow(2*i/float(num_options-1) - 1, 2);
        if (i < num_options / 2) {
            curvature = -curvature;
        }
        
        navigation::PathOption path_option;
        setPathOption(path_option, curvature, point_cloud, robot_config, robot_rel_carrot);
        path_options.push_back(path_option);
    }
    // exit(0);
    return path_options;
}


float score(float free_path_length, float goal_dist, float clearance) {
    const float w1 = 1;
    const float w2 = 0;
    const float w3 = 0.1;
    // TODO: currently we are weighting this 0 but will have to tune this later
    // float score = w1*free_path_length + w2 * goal_dist + w3 * clearance;
    // std::cout << "fpl weight " << w1*free_path_length/score << "goal dist weight " << (w2 * goal_dist) / score << "clearance weight " << (w3 * clearance) / score << " final score "<< score <<endl;
    return w1*free_path_length + w2 * goal_dist + w3 * clearance;
}

// returns the index of the selected path
// for now, just return the index of the path with the longest free path length
// if there are multiple paths with the same free path length, return the one with the smallest curvature
int selectPath(const vector<navigation::PathOption>& path_options, Eigen::Vector2f robot_rel_carrot) {
    int selected_path = 0;
    float best_score = 0;
    for (unsigned int i = 0; i < path_options.size(); i++) {
        double goal_dist = pow(robot_rel_carrot.x() - path_options[i].closest_point.x(), 2) +  pow(robot_rel_carrot.y() - path_options[i].closest_point.y(), 2);
        float s = score(path_options[i].free_path_length, goal_dist, path_options[i].clearance);
        if (s > best_score) {
            best_score = s;
            selected_path = i;
        }
        // std::cout << "selected path fpl:" << path_options[selected_path].free_path_length << " goal dist: " << pow(robot_rel_carrot.x() - path_options[selected_path].closest_point.x(), 2) +  pow(robot_rel_carrot.y() - path_options[selected_path].closest_point.y(), 2) << " clearance: " << path_options[selected_path].clearance << std::endl;
    }
    return selected_path;
}

