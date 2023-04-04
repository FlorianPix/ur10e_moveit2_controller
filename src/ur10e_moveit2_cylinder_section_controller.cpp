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

geometry_msgs::msg::Quaternion ToQuaternion(double roll, double pitch, double yaw)
{
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);

    geometry_msgs::msg::Quaternion  q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
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
    box_pose.orientation = ToQuaternion(roll, pitch, yaw);
    box_pose.position.x = position_x;
    box_pose.position.y = position_y;
    box_pose.position.z = position_z;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
}

std::vector<geometry_msgs::msg::Pose> cylinder_section(
    double R = 0.3 /* radius of cylinder */,
    double step_size = 0.05 /* distance between curves */,
    double min_angle = M_PI / 8 /* start angle of cylinder section */,
    double max_angle = 7 * M_PI / 8 /* end angle of cylinder section */,
    double x_offset = 0.5 /* x position of first point of cylinder */,
    double y_offset = 0.5 /* y position of first point of cylinder */,
    double z_offset = 0.25 /* z position of first point of cylinder */){
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose target_pose;

    for (int p = 0; p < 4; p += 2) {
        for (double t = min_angle; t < max_angle; t += 0.1) {
            target_pose.position.x = R * cos(t) + x_offset;
            target_pose.position.z = R * sin(t) + z_offset;
            target_pose.position.y = step_size * p + y_offset;
            target_pose.orientation = ToQuaternion(M_PI, (M_PI/4.0)-(2.0/3.0)*(t-(M_PI/8.0)), 0.0);
            waypoints.push_back(target_pose);
        }
        for (double t = 0.0; t < step_size; t += 0.01) {
            target_pose.position.x = R * cos(max_angle) + x_offset;
            target_pose.position.z = R * sin(max_angle) + z_offset;
            target_pose.position.y = step_size * (p + t) + y_offset;
            target_pose.orientation = ToQuaternion(M_PI, -M_PI/4.0, 0.0);
            waypoints.push_back(target_pose);
        }

        for (double t = max_angle; t > min_angle ; t -= 0.1){
            target_pose.position.x = R * cos(t) + x_offset;
            target_pose.position.z = R * sin(t) + z_offset;
            target_pose.position.y = step_size * (p + 1) + y_offset;
            target_pose.orientation = ToQuaternion(M_PI, (M_PI/4.0)-(2.0/3.0)*(t-(M_PI/8.0)), 0.0);
            waypoints.push_back(target_pose);
        }

        for (double t = 1.0; t < 2.0; t += 0.01) {
            target_pose.position.x = R * cos(min_angle) + x_offset;
            target_pose.position.z = R * sin(min_angle) + z_offset;
            target_pose.position.y = step_size * (p + t) + y_offset;
            target_pose.orientation = ToQuaternion(M_PI, M_PI/4.0, 0.0);
            waypoints.push_back(target_pose);
        }
    }

    return waypoints;
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

    using moveit::planning_interface::PlanningSceneInterface;
    auto planning_scene_interface = PlanningSceneInterface();

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    
    // floor
    collision_objects.push_back(create_collision_box(
        move_group_interface.getPlanningFrame(),
        "floor",
        3.0, 3.0, 0.1,
        0.0, 0.0, -0.78,
        0.0, 0.0, 0.0
    ));

    // virtual walls
    /*
    collision_objects.push_back(create_collision_box(
        move_group_interface.getPlanningFrame(),
        "wall1",
        0.1, 3.0, 3.0,
        1.5, 0.0, -0.78, 
        0.0, 0.0, 0.0
    ));
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

    // boxes on which the ur10 stands
    collision_objects.push_back(create_collision_box(
        move_group_interface.getPlanningFrame(),
        "box1",
        0.3, 0.45, 0.53,
        -0.155, 0.0, -0.27,
        0.0, 0.0, 0.0
    ));
    collision_objects.push_back(create_collision_box(
        move_group_interface.getPlanningFrame(),
        "box2",
        0.8, 0.8, 0.25,
        0.0, 0.0, -0.655,
        0.0, 0.0, 0.0
    ));

    // specimen
    collision_objects.push_back(create_collision_box(
        move_group_interface.getPlanningFrame(),
        "stand",
        0.35, 0.35, 0.62,
        0.5, 0.5, -0.47,
        0.0, 0.0, 0.0
    ));
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
    planning_scene_interface.applyCollisionObjects(collision_objects);

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

    std::vector<geometry_msgs::msg::Pose> waypoints = cylinder_section();
    geometry_msgs::msg::Pose target_pose = waypoints.at(0);

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

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 20.0;
    const double eef_step = 0.01;
    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    if (fraction == 1.0){
        // valid trajectory
        // if fraction is less than 1.0 it means that only a fraction of the waypoints could be reached in cartesian space
        // if fraction is -1.0 there was an error
        move_group_interface.execute(trajectory);
    } else {
        RCLCPP_ERROR(logger, "Planning failed at (%.2f%%)", fraction * 100.0);
        rclcpp::shutdown();
        return 0;
    }

    rclcpp::shutdown();
    return 0;
}