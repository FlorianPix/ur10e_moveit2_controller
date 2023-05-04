#define _USE_MATH_DEFINES
#include <math.h>

#include <memory>

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include "std_msgs/msg/bool.hpp"
#include "trajectory.h"
#include "collision.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class Controller : public rclcpp::Node
{
public:
    double timer_interval = 1.0;
    std_msgs::msg::Bool finished_msg = std_msgs::msg::Bool();

    Controller() : Node("ur10e_moveit2_controller", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)){
        finished_msg.data = true;
        publisher_ = this->create_publisher<std_msgs::msg::Bool>("/finished", 10);
        timer_ = rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration(timer_interval*1e9), std::bind(&Controller::timer_callback, this));
    }

    void timer_callback()
    {
        this->publisher_->publish(finished_msg);
    }

private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Controller>();

    bool collision_aware = node->get_parameter("collision_aware").as_bool();
    bool collision_walls = node->get_parameter("collision_walls").as_bool();
    std::vector<double> floor_coll = node->get_parameter("collision.floor").as_double_array();
    std::vector<double> wall1_coll = node->get_parameter("collision.wall1").as_double_array();
    std::vector<double> wall2_coll = node->get_parameter("collision.wall2").as_double_array();
    std::vector<double> wall3_coll = node->get_parameter("collision.wall3").as_double_array();
    std::vector<double> wall4_coll = node->get_parameter("collision.wall4").as_double_array();
    std::vector<double> stand_coll = node->get_parameter("collision.stand").as_double_array();
    std::vector<double> linear_axis_coll = node->get_parameter("collision.linear_axis").as_double_array();
    std::vector<double> conveyor_control_coll = node->get_parameter("collision.conveyor_control").as_double_array();
    std::vector<double> UR10_control_coll = node->get_parameter("collision.UR10_control").as_double_array();
    std::vector<double> conveyor_coll = node->get_parameter("collision.conveyor").as_double_array();
    std::vector<double> specimen_coll = node->get_parameter("collision.specimen").as_double_array();
    std::vector<double> camera_coll = node->get_parameter("collision.camera").as_double_array();
    unsigned int wait_milliseconds = node->get_parameter("wait_milliseconds").as_int();
    bool use_rectangle_trajectory = node->get_parameter("use_rectangle_trajectory").as_bool();
    double rectangle_y_min = node->get_parameter("rectangle.y_min").as_double();
    double rectangle_x_offset = node->get_parameter("rectangle.x_offset").as_double();
    double rectangle_z_min = node->get_parameter("rectangle.z_min").as_double();
    double rectangle_y_max = node->get_parameter("rectangle.y_max").as_double();
    double rectangle_z_max = node->get_parameter("rectangle.z_max").as_double();
    double rectangle_steps = node->get_parameter("rectangle.steps").as_double();
    double rectangle_pitch_min = node->get_parameter("rectangle.pitch_min").as_double();
    double rectangle_pitch_max = node->get_parameter("rectangle.pitch_max").as_double();
    double rectangle_pitch_step = node->get_parameter("rectangle.pitch_step").as_double();
    bool rectangle_pitch_adjust = node->get_parameter("rectangle.pitch_adjust").as_bool();
    double cylinder_section_R = node->get_parameter("cylinder_section.R").as_double();
    double cylinder_section_step_size = node->get_parameter("cylinder_section.step_size").as_double();
    double cylinder_section_min_angle = node->get_parameter("cylinder_section.min_angle").as_double();
    double cylinder_section_max_angle = node->get_parameter("cylinder_section.max_angle").as_double();
    double cylinder_section_x_offset = node->get_parameter("cylinder_section.x_offset").as_double();
    double cylinder_section_y_offset = node->get_parameter("cylinder_section.y_offset").as_double();
    double cylinder_section_z_offset = node->get_parameter("cylinder_section.z_offset").as_double();

    static const std::string PLANNING_GROUP = "ur_manipulator";

    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, PLANNING_GROUP);
    move_group_interface.setPlanningTime(15.0);
    move_group_interface.setMaxVelocityScalingFactor(0.25);
    move_group_interface.setMaxAccelerationScalingFactor(0.25);
    using moveit::planning_interface::PlanningSceneInterface;
    auto planning_scene_interface = PlanningSceneInterface();

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    if (collision_aware) {
        collision_objects.push_back(create_collision_box(move_group_interface.getPlanningFrame(), "floor", floor_coll[0], floor_coll[1], floor_coll[2], floor_coll[3], floor_coll[4], floor_coll[5], floor_coll[6], floor_coll[7], floor_coll[8]));
        collision_objects.push_back(create_collision_box(move_group_interface.getPlanningFrame(), "stand", stand_coll[0], stand_coll[1], stand_coll[2], stand_coll[3], stand_coll[4], stand_coll[5], stand_coll[6], stand_coll[7], stand_coll[8]));
        collision_objects.push_back(create_collision_box(move_group_interface.getPlanningFrame(), "linear_axis", linear_axis_coll[0], linear_axis_coll[1], linear_axis_coll[2], linear_axis_coll[3], linear_axis_coll[4], linear_axis_coll[5], linear_axis_coll[6], linear_axis_coll[7], linear_axis_coll[8]));
        collision_objects.push_back(create_collision_box(move_group_interface.getPlanningFrame(), "conveyor_control", conveyor_control_coll[0], conveyor_control_coll[1], conveyor_control_coll[2], conveyor_control_coll[3], conveyor_control_coll[4], conveyor_control_coll[5], conveyor_control_coll[6], conveyor_control_coll[7], conveyor_control_coll[8]));
        collision_objects.push_back(create_collision_box(move_group_interface.getPlanningFrame(), "UR10_control", UR10_control_coll[0], UR10_control_coll[1], UR10_control_coll[2], UR10_control_coll[3], UR10_control_coll[4], UR10_control_coll[5], UR10_control_coll[6], UR10_control_coll[7], UR10_control_coll[8]));
        collision_objects.push_back(create_collision_box(move_group_interface.getPlanningFrame(), "conveyor", conveyor_coll[0], conveyor_coll[1], conveyor_coll[2], conveyor_coll[3], conveyor_coll[4], conveyor_coll[5], conveyor_coll[6], conveyor_coll[7], conveyor_coll[8]));
        collision_objects.push_back(create_collision_box(move_group_interface.getPlanningFrame(), "specimen", specimen_coll[0], specimen_coll[1], specimen_coll[2], specimen_coll[3], specimen_coll[4], specimen_coll[5], specimen_coll[6], specimen_coll[7], specimen_coll[8]));
        if (collision_walls) {
            collision_objects.push_back(create_collision_box(move_group_interface.getPlanningFrame(), "wall1", wall1_coll[0], wall1_coll[1], wall1_coll[2], wall1_coll[3], wall1_coll[4], wall1_coll[5], wall1_coll[6], wall1_coll[7], wall1_coll[8]));
            collision_objects.push_back(create_collision_box(move_group_interface.getPlanningFrame(), "wall2", wall2_coll[0], wall2_coll[1], wall2_coll[2], wall2_coll[3], wall2_coll[4], wall2_coll[5], wall2_coll[6], wall2_coll[7], wall2_coll[8]));
            collision_objects.push_back(create_collision_box(move_group_interface.getPlanningFrame(), "wall3", wall3_coll[0], wall3_coll[1], wall3_coll[2], wall3_coll[3], wall3_coll[4], wall3_coll[5], wall3_coll[6], wall3_coll[7], wall3_coll[8]));
            collision_objects.push_back(create_collision_box(move_group_interface.getPlanningFrame(), "wall4", wall4_coll[0], wall4_coll[1], wall4_coll[2], wall4_coll[3], wall4_coll[4], wall4_coll[5], wall4_coll[6], wall4_coll[7], wall4_coll[8]));
        }
    }
    // camera
    moveit_msgs::msg::CollisionObject camera = create_collision_box(move_group_interface.getEndEffectorLink(), "camera", camera_coll[0], camera_coll[1], camera_coll[2], camera_coll[3], camera_coll[4], camera_coll[5], camera_coll[6], camera_coll[7], camera_coll[8]);
    collision_objects.push_back(camera);
    planning_scene_interface.applyCollisionObjects(collision_objects);

    // attach camera to wrist
    moveit_msgs::msg::AttachedCollisionObject camera2;
    camera2.link_name = "wrist_3_link";
    camera2.object = camera;

    std::vector<std::string> touch_links;
    if (!planning_scene_interface.applyAttachedCollisionObject(camera2)) {
        RCLCPP_ERROR(node->get_logger(), "Couldn't attach object!");
        rclcpp::shutdown();
        return 0;
    };

    std::vector<geometry_msgs::msg::Pose> waypoints;
    if (use_rectangle_trajectory){
        waypoints = create_rectangle(
            rectangle_y_min,
            rectangle_x_offset,
            rectangle_z_min,
            rectangle_y_max,
            rectangle_z_max,
            rectangle_steps,
            rectangle_pitch_min,
            rectangle_pitch_max,
            rectangle_pitch_step,
            rectangle_pitch_adjust
        );
    } else {
        waypoints = create_cylinder_section(
            cylinder_section_R,
            cylinder_section_step_size,
            cylinder_section_min_angle,
            cylinder_section_max_angle,
            cylinder_section_x_offset,
            cylinder_section_y_offset,
            cylinder_section_z_offset
        );
    }

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
        RCLCPP_INFO(node->get_logger(), "Planning succeeded");
        // move_group_interface.execute(trajectory);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Planning failed at (%.2f%%)", fraction * 100.0);
        RCLCPP_INFO(node->get_logger(), "First waypoint would have been x_%f y_%f z_%f", waypoints[0].position.x, waypoints[0].position.y, waypoints[0].position.z);
        rclcpp::shutdown();
        return 0;
    }

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
                RCLCPP_INFO(node->get_logger(), "Planning to: x_%.2f y_%.2f z_%.2f succeeded", waypoints[i].position.x, waypoints[i].position.y, waypoints[i].position.z);
                move_group_interface.execute(trajectory);
                // pause to record pcd
                RCLCPP_INFO(node->get_logger(), "Pausing for %i milliseconds to enable stable point cloud recording", wait_milliseconds);
                std::this_thread::sleep_for(std::chrono::milliseconds(wait_milliseconds));
            } else {
                RCLCPP_ERROR(node->get_logger(), "Planning to: x_%.2f y_%.2f z_%.2f failed at (%.2f%%)", waypoints[i].position.x, waypoints[i].position.y, waypoints[i].position.z, fraction * 100.0);
                rclcpp::shutdown();
                return 0;
            }
        }
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}