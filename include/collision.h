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