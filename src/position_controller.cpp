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

    using moveit::planning_interface::PlanningSceneInterface;
    auto planning_scene_interface = PlanningSceneInterface();

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.4;
    target_pose.position.y = -0.4;
    target_pose.position.z = 0.4;
    tf2::Quaternion q;
    q.setRPY(M_PI/2, 0.0, 0.0);
    target_pose.orientation.w = q.w();
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    move_group_interface.setPoseTarget(target_pose);
    // Create a plan to that target pose
    auto const [success, plan] = [&move_group_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if(success) {
        move_group_interface.execute(plan);
    } else {
        RCLCPP_ERROR(logger, "Planning failed!");
        rclcpp::shutdown();
        return 0;
    }

    rclcpp::shutdown();
    return 0;
}