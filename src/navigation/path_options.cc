#include "path_options.h"
#include <cstdio>
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
                if (clearance_p < path_option.clearance) {
                    path_option.clearance = clearance_p;
                    //path_option.closest_point = p;
                }
            }
        }
        return;
    }

    float r = 1 / curvature;
    Vector2f c = Vector2f(0, r);
    float r_inner = c.norm() - robot_config.width / 2 - robot_config.safety_margin;
    float r_outer = c.norm() + robot_config.width / 2 + robot_config.safety_margin;
    float r_tl = (Vector2f(0, r_inner) - Vector2f(h + robot_config.safety_margin, 0)).norm();
    float r_tr = (Vector2f(0, r_outer) - Vector2f(h + robot_config.safety_margin, 0)).norm();
    float r_br = (Vector2f(0, r_outer) - Vector2f(robot_config.base_link_offset + robot_config.safety_margin, 0)).norm();

    // float omega = atan2(h, r_inner);

    float theta_br = asin(robot_config.base_link_offset + robot_config.safety_margin / r_br); // angle where back right would hit
    float phi = 0;
    //	cout << "curvature " << curvature << endl;
    //	bool front_side = false, outer_side = false, inner_side = false;

    // angle between c and robot_rel_carrot
    float optimal_theta = curvature < 0 ? atan2(robot_rel_carrot[0], robot_rel_carrot[1]- c[1]) : atan2(robot_rel_carrot[0], c[1] - robot_rel_carrot[1]);
    float optimal_fpl = optimal_theta * c.norm();
    // Used for finding the optimal_endpt
    Eigen::Affine2f rot_1 = Eigen::Translation2f(0, r) * Eigen::Rotation2Df(optimal_theta);
    Vector2f r_rot = Vector2f(0, -r);
    Vector2f optimal_endpt = rot_1 * r_rot; //start off with optimal endpt, select between the short fpl when checking with the p.

    path_option.free_path_length = optimal_fpl; // will always be <= feasible free path length.
    path_option.closest_point = optimal_endpt; // has to >= in distance to the feasible end point.

    for (unsigned int i = 0; i < point_cloud.size(); i++) {
        Vector2f p = point_cloud[i];
        float r_p = (c-p).norm();


        float theta = curvature < 0 ? atan2(p[0], p[1]- c[1]) : atan2(p[0], c[1] - p[1]); // angle between p and c
        Vector2f obst_endpt = optimal_endpt;

        float length = 5.0;
        // cout << "curvature " << curvature << endl;
        if (r_inner <= r_p && r_p <= r_tl) {    // inner side hit
            phi = acos(r_inner / r_p);
            length = (theta - phi) * c.norm();

            // now for the closest point, which depends on phi.
            Eigen::Affine2f rot_2 = Eigen::Translation2f(0, r) * Eigen::Rotation2Df(phi);
            obst_endpt = rot_2 * r_rot;

            // inner_side = true;
            // cout << "inner side hit" << endl;
        }
        if ((r_inner <= r_p && r_p <= r_br) && (-theta_br <= theta && theta <= theta_br)) {    // outer side hit
            phi = acos(r_p / (c.norm() + robot_config.width / 2));
            length = (theta - phi) * c.norm();

            // now for the closest point, which depends on phi.
            Eigen::Affine2f rot_2 = Eigen::Translation2f(0, r) * Eigen::Rotation2Df(phi);
            obst_endpt = rot_2 * r_rot;

            // outer_side = true;
            // cout << "outer side hit" << endl;
        }

        if (r_tl <= r_p && r_p <= r_tr) {    // front side hit
            phi = asin(h / r_p);
            length = (theta - phi) * c.norm();

            // now for the closest point, which depends on phi.
            Eigen::Affine2f rot_2 = Eigen::Translation2f(0, r) * Eigen::Rotation2Df(phi);
            obst_endpt = rot_2 * r_rot;

            // front_side = true;
            // cout << "front side hit" << endl;
        }

        // At this point the feasible fpl has been calculated like we want it.
        if (length < path_option.free_path_length && length > 0) {
            path_option.free_path_length = length;
            path_option.obstruction = p;
            path_option.closest_point = obst_endpt;
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
            float clearance_p = fmin(std::min(inner, outer), clearance_cap);
            if (clearance_p < path_option.clearance) {
                path_option.clearance = clearance_p;
                path_option.obstruction = p;
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
    const float w2 = -1;
    const float w3 = 1.5;
    // TODO: currently we are weighting this 0 but will have to tune this later
    return w1 * free_path_length + w2 * goal_dist + w3 * clearance;
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
    }
    return selected_path;
}

