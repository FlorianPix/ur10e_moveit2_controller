#define _USE_MATH_DEFINES
#include <math.h>

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

moveit_msgs::msg::CollisionObject create_collision_box(
        std::string frame_id,
        std::string object_id,
        double size_x,
        double size_y,
        double size_z,
        double position_x,
        double position_y,
        double position_z,
        double roll,
        double pitch,
        double yaw
    ){
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;

    // The id of the object is used to identify it.
    collision_object.id = object_id;

    // Define a box to add to the world.
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = size_x;
    primitive.dimensions[primitive.BOX_Y] = size_y;
    primitive.dimensions[primitive.BOX_Z] = size_z;

    // Define a pose for the box (specified relative to frame_id).
    geometry_msgs::msg::Pose box_pose;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    box_pose.orientation.w = q.w();
    box_pose.orientation.x = q.x();
    box_pose.orientation.y = q.y();
    box_pose.orientation.z = q.z();
    box_pose.position.x = position_x;
    box_pose.position.y = position_y;
    box_pose.position.z = position_z;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
}

std::vector<geometry_msgs::msg::Pose> create_rectangle(
    double y_min = -0.75 /* x position of first point of rectangle */,
    double x_offset = 0.6 /* y position of first point of rectangle */,
    double z_min = 0.0 /* z position of first point of rectangle */,
    double y_max = 0.75 /* x position of last point of rectangle */,
    double z_max = 1.0 /* z position of last point of rectangle */,
    double steps = 4.0 /* divisions of the rectangle */){
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose target_pose;

    double height_diff = fabs(z_max - z_min);
    double step_size = height_diff / steps;

    for (double pitch = -M_PI/16; pitch <= M_PI/16; pitch += M_PI/16){
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
    }
    waypoints.push_back(waypoints.at(0));
    return rotate(waypoints, 0); // M_PI/4
}

int main(int argc, char * argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "ur10e_moveit2_controller",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    static const std::string PLANNING_GROUP = "ur_manipulator";
    // Create a ROS logger
    auto const logger = rclcpp::get_logger("ur10e_moveit2_controller");

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, PLANNING_GROUP);
    move_group_interface.setPlanningTime(15.0);
    move_group_interface.setMaxVelocityScalingFactor(0.25);
    move_group_interface.setMaxAccelerationScalingFactor(0.25);

    using moveit::planning_interface::PlanningSceneInterface;
    auto planning_scene_interface = PlanningSceneInterface();

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    // floor
    collision_objects.push_back(create_collision_box(
        move_group_interface.getPlanningFrame(),
        "floor",
        3.0, 3.0, 0.1,
        0.0, 0.0, -0.78,
        0.0, 0.0, -M_PI/4
    ));

    // virtual walls
    collision_objects.push_back(create_collision_box(
        move_group_interface.getPlanningFrame(),
        "wall1",
        0.1, 3.0, 3.0,
        -0.21, -0.21, -0.78,
        0.0, 0.0, M_PI/4
    ));
    /*
    collision_objects.push_back(create_collision_box(
        move_group_interface.getPlanningFrame(),
        "wall2",
        3.0, 0.1, 3.0,
        0.0, 1.5, -0.78,
        0.0, 0.0, 0.0
    ));
    collision_objects.push_back(create_collision_box(
        move_group_interface.getPlanningFrame(),
        "wall3",
        0.1, 3.0, 3.0,
        -0.5, 0.0, -0.78,
        0.0, 0.0, 0.0
    ));
    collision_objects.push_back(create_collision_box(
        move_group_interface.getPlanningFrame(),
        "wall4",
        3.0, 0.1, 3.0,
        0.0, -1.5, -0.78,
        0.0, 0.0, 0.0
    ));
    */
    // boxes on which the ur10 stands
    collision_objects.push_back(create_collision_box(
        move_group_interface.getPlanningFrame(),
        "stand",
        0.2, 0.2, 0.53,
        0.0, 0.0, -0.27,
        0.0, 0.0, -M_PI/4
    ));
    collision_objects.push_back(create_collision_box(
        move_group_interface.getPlanningFrame(),
        "linear_axis",
        1.0, 2.15, 0.25,
        -0.2, -0.2, -0.655,
        0.0, 0.0, -M_PI/4
    ));
    // control box conveyor
    collision_objects.push_back(create_collision_box(
        move_group_interface.getPlanningFrame(),
        "conveyor_control",
        0.35, 0.48, 0.68,
        0.85, -0.85, -0.44,
        0.0, 0.0, -M_PI/4
    ));
    // control box UR10
    collision_objects.push_back(create_collision_box(
        move_group_interface.getPlanningFrame(),
        "UR10_control",
        0.8, 0.8, 0.8,
        0.0, -1.3, -0.4,
        0.0, 0.0, -M_PI/4
    ));
    // conveyor
    collision_objects.push_back(create_collision_box(
        move_group_interface.getPlanningFrame(),
        "conveyor",
        2.6, 1.54, 0.55,
        0.95, 0.95, -0.45,
        0.0, 0.0, -M_PI/4
    ));
    /*
    // specimen
    collision_objects.push_back(create_collision_box(
        move_group_interface.getPlanningFrame(),
        "specimen",
        0.1, 0.1, 0.25,
        0.5, 0.5, -0.035,
        0.0, 0.0, 0.0
    ));
    */
    // camera
    moveit_msgs::msg::CollisionObject camera = create_collision_box(
        move_group_interface.getEndEffectorLink(),
        "camera",
        0.18, 0.03, 0.05,
        0.0, 0.0, 0.04,
        0.0, 0.0, M_PI/4.0
    );
    collision_objects.push_back(camera);
    // planning_scene_interface.applyCollisionObjects(collision_objects);

    // attach camera to wrist
    moveit_msgs::msg::AttachedCollisionObject camera2;
    camera2.link_name = "wrist_3_link";
    camera2.object = camera;
    
    std::vector<std::string> touch_links;
    if (!planning_scene_interface.applyAttachedCollisionObject(camera2)) {
        RCLCPP_ERROR(logger, "Couldn't attach object!");
        rclcpp::shutdown();
        return 0;
    };

    std::vector<geometry_msgs::msg::Pose> waypoints = create_rectangle();

    // check if whole trajectory is valid
    bool is_valid_trajectory = false;
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 50.0;
    const double eef_step = 0.02;
    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    if (fraction == 1.0){
        // valid trajectory
        // if fraction is less than 1.0 it means that only a fraction of the waypoints could be reached in cartesian space
        // if fraction is -1.0 there was an error
        is_valid_trajectory = true;
        RCLCPP_INFO(logger, "Planning succeeded");
        // move_group_interface.execute(trajectory);
    } else {
        RCLCPP_ERROR(logger, "Planning failed at (%.2f%%)", fraction * 100.0);
        rclcpp::shutdown();
        return 0;
    }

    unsigned int wait_milliseconds = 1000;
    if (is_valid_trajectory){
        for (unsigned int i = 0; i < waypoints.size() - 1; i++){
            std::vector<geometry_msgs::msg::Pose>::const_iterator first = waypoints.begin() + i;
            std::vector<geometry_msgs::msg::Pose>::const_iterator last = waypoints.begin() + i + 1;
            std::vector<geometry_msgs::msg::Pose> sub_path(first, last);
            fraction = move_group_interface.computeCartesianPath(sub_path, eef_step, jump_threshold, trajectory);
            if (fraction == 1.0){
                // valid trajectory
                // if fraction is less than 1.0 it means that only a fraction of the waypoints could be reached in cartesian space
                // if fraction is -1.0 there was an error
                is_valid_trajectory = true;
                RCLCPP_INFO(logger, "Planning to: x_%.2f y_%.2f z_%.2f succeeded", waypoints[i].position.x, waypoints[i].position.y, waypoints[i].position.z);
                move_group_interface.execute(trajectory);
                // pause to record pcd
                RCLCPP_INFO(logger, "Pausing for %i milliseconds to enable stable point cloud recording", wait_milliseconds);
                std::this_thread::sleep_for(std::chrono::milliseconds(wait_milliseconds));
            } else {
                RCLCPP_ERROR(logger, "Planning to: x_%.2f y_%.2f z_%.2f failed at (%.2f%%)", waypoints[i].position.x, waypoints[i].position.y, waypoints[i].position.z, fraction * 100.0);
                rclcpp::shutdown();
                return 0;
            }
        }
    }

    rclcpp::shutdown();
    return 0;
}