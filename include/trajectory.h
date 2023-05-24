#define _USE_MATH_DEFINES
#include <math.h>
#include <algorithm>
#include <stdexcept>

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>


std::vector<geometry_msgs::msg::Pose> rotate(std::vector<geometry_msgs::msg::Pose> ps, float theta){
    std::vector<geometry_msgs::msg::Pose> ps_r;
    geometry_msgs::msg::Pose p_r;
    tf2::Quaternion q_rot, q_new;
    q_rot.setRPY(0.0, 0.0, theta);
    for (unsigned int i = 0; i < size(ps); i++) {
        geometry_msgs::msg::Pose p = ps.at(i);
        p_r = p;
        p_r.position.x = p.position.x * std::cos(theta) - p.position.y * std::sin(theta);
        p_r.position.y = p.position.x * std::sin(theta) + p.position.y * std::cos(theta);
        tf2::Quaternion q_orig = tf2::Quaternion(p_r.orientation.x, p_r.orientation.y, p_r.orientation.z, p_r.orientation.w);
        q_new = q_rot * q_orig;
        q_new.normalize();
        p_r.orientation.w = q_new.w();
        p_r.orientation.x = q_new.x();
        p_r.orientation.y = q_new.y();
        p_r.orientation.z = q_new.z();
        ps_r.push_back(p_r);
    }
    return ps_r;
}

std::vector<geometry_msgs::msg::Pose> create_cylinder_section(
    double R = 0.3 /* radius of cylinder */,
    double step_size = 0.05 /* distance between curves */,
    double min_angle = M_PI / 8 /* start angle of cylinder section */,
    double max_angle = 7 * M_PI / 8 /* end angle of cylinder section */,
    double x_offset = 0.5 /* x position of first point of cylinder */,
    double y_offset = 0.5 /* y position of first point of cylinder */,
    double z_offset = 0.25 /* z position of first point of cylinder */){
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose target_pose;

    double end_step;
    double angle_step_size = M_PI/16;
    for (int p = 0; p < 4; p += 2) {
        for (double t = min_angle; t < max_angle; t += angle_step_size) {
            target_pose.position.y = R * cos(t) + y_offset;
            target_pose.position.z = R * sin(t) + z_offset;
            target_pose.position.x = step_size * p + x_offset;
            tf2::Quaternion q;
            q.setRPY((M_PI/4.0)-(2.0/3.0)*(t-(M_PI/8.0)), M_PI, 0.0);
            target_pose.orientation.w = q.w();
            target_pose.orientation.x = q.x();
            target_pose.orientation.y = q.y();
            target_pose.orientation.z = q.z();
            waypoints.push_back(target_pose);
        }
        end_step = target_pose.position.x + step_size;
        for (double t = 0.0; target_pose.position.x < end_step; t += 0.01) {
            target_pose.position.x += t;
            waypoints.push_back(target_pose);
        }
        for (double t = max_angle; t > min_angle ; t -= angle_step_size){
            target_pose.position.y = R * cos(t) + y_offset;
            target_pose.position.z = R * sin(t) + z_offset;
            target_pose.position.x = step_size * (p + 1) + x_offset;
            tf2::Quaternion q;
            q.setRPY((M_PI/4.0)-(2.0/3.0)*(t-(M_PI/8.0)), M_PI, 0.0);
            target_pose.orientation.w = q.w();
            target_pose.orientation.x = q.x();
            target_pose.orientation.y = q.y();
            target_pose.orientation.z = q.z();
            waypoints.push_back(target_pose);
        }
        end_step = target_pose.position.x + step_size;
        for (double t = 0.0; target_pose.position.x < end_step; t += 0.01) {
            target_pose.position.x += t;
            waypoints.push_back(target_pose);
        }
    }
    return rotate(waypoints, M_PI/4);
}

std::vector<geometry_msgs::msg::Pose> create_rectangle(
    double y_min = -0.5 /* y position of first point of rectangle */,
    double x_offset = 0.5 /* x position of first point of rectangle */,
    double z_min = 0.1 /* z position of first point of rectangle */,
    double y_max = 0.5 /* y position of last point of rectangle */,
    double z_max = 0.5 /* z position of last point of rectangle */,
    double steps = 4.0 /* divisions of the rectangle */,
    double pitch_min = -M_PI/16,
    double pitch_max = M_PI/16,
    double pitch_step = M_PI/16,
    bool pitch_adjust = false,
    double theta = M_PI/4) {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose target_pose;

    double height_diff = fabs(z_max - z_min);
    double step_size = height_diff / steps;

    for (double pitch = pitch_min; pitch <= pitch_max; pitch += pitch_step){
        if(!pitch_adjust){
            pitch = 0.0;
        }
        for (double z = 0.0; z < height_diff; z += 2 * step_size) {
            for (double y = y_min; y < y_max; y += 0.25){
                target_pose.position.x = x_offset;
                target_pose.position.z = z_min + z;
                target_pose.position.y = y;
                tf2::Quaternion q;
                q.setRPY(M_PI/2 + pitch, M_PI/4, M_PI/2 + pitch);
                target_pose.orientation.w = q.w();
                target_pose.orientation.x = q.x();
                target_pose.orientation.y = q.y();
                target_pose.orientation.z = q.z();
                waypoints.push_back(target_pose);
            }
            for (double intermediate_z = z; intermediate_z < z + step_size; intermediate_z += 0.25){
                target_pose.position.x = x_offset;
                target_pose.position.z = z_min + intermediate_z;
                target_pose.position.y = y_max;
                tf2::Quaternion q;
                q.setRPY(M_PI/2 + pitch, M_PI/4, M_PI/2 + pitch);
                target_pose.orientation.w = q.w();
                target_pose.orientation.x = q.x();
                target_pose.orientation.y = q.y();
                target_pose.orientation.z = q.z();
                waypoints.push_back(target_pose);
            }
            for (double y = y_max; y > y_min; y -= 0.25){
                target_pose.position.x = x_offset;
                target_pose.position.z = z_min + z + step_size;
                target_pose.position.y = y;
                tf2::Quaternion q;
                q.setRPY(M_PI/2 + pitch, M_PI/4, M_PI/2 + pitch);
                target_pose.orientation.w = q.w();
                target_pose.orientation.x = q.x();
                target_pose.orientation.y = q.y();
                target_pose.orientation.z = q.z();
                waypoints.push_back(target_pose);
            }
            if (z < z_max){
                for (double intermediate_z = z + step_size; intermediate_z < z + 2 * step_size; intermediate_z += 0.25){
                    target_pose.position.x = x_offset;
                    target_pose.position.z = z_min + intermediate_z;
                    target_pose.position.y = y_min;
                    tf2::Quaternion q;
                    q.setRPY(M_PI/2 + pitch, M_PI/4, M_PI/2 + pitch);
                    target_pose.orientation.w = q.w();
                    target_pose.orientation.x = q.x();
                    target_pose.orientation.y = q.y();
                    target_pose.orientation.z = q.z();
                    waypoints.push_back(target_pose);
                }
            }
        }
        if(!pitch_adjust){
            break;
        }
    }
    waypoints.push_back(waypoints.at(0));
    return rotate(waypoints, theta);
}

std::vector<geometry_msgs::msg::Pose> create_rectangle_for_specimen(
    double x_width = 1.0 /* width of the specimen in x direction (forward-backward) in meters */,
    double y_width = 1.0 /* width of the specimen in y direction (left-right) in meters */,
    double theta = 0.785398163 /* rotation around z-axis as radians */,
    double x_off = 0.8 /* offset in x direction (forward-backward) in meters */,
    double y_off = 0.0 /* offset in y direction (left-right) in meters */,
    double z_min = 0.0 /* lower limit for height in meters */,
    double z_max = 1.0 /* upper limit for height in meters */
){
    if (x_width < 0.0) throw std::invalid_argument("x_width can't be less than 0.0");
    if (y_width < 0.0) throw std::invalid_argument("y_width can't be less than 0.0");
    if (x_off < 0.5) throw std::invalid_argument("x_off can't be less than 0.5");
    if (std::abs(y_off) > 0.65) throw std::invalid_argument("the absolute value of y_off can't exceed 0.65");
    if (z_min > z_max) throw std::invalid_argument("z_min can't exceed z_max");

    x_off = std::min(std::max(x_off - 0.25, 0.25), 0.75);
    double y_min = std::max(y_off - y_width / 2, -0.5);
    double y_max = std::min(y_off + y_width / 2, 0.5);
    z_min = std::max(z_min, 0.1);
    z_max = std::min(z_max, 1.0);
    return create_rectangle(y_min, x_off, z_min, y_max, z_max, 4.0, 0.0, 0.0, 0.0, false, theta);
}