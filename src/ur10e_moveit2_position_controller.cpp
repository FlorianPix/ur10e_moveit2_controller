#define _USE_MATH_DEFINES
#include <math.h>

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

struct Quaternion
{
    double w, x, y, z;
};

Quaternion ToQuaternion(double roll, double pitch, double yaw)
{
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

int main(int argc, char * argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "ur10e_moveit2_position_controller",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("ur10e_moveit2_position_controller");

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

    double const R = 0.1;  // winding radius
    double const step_size = 0.1;
    int const array_size = 341; // (10.0 / 2.0) * (2.0 * ((M_PI / 0.1)) + 4.0) + ?;
    double xs[array_size], ys[array_size], zs[array_size];
    Quaternion qs[array_size];
    int i = 0;

    double min_angle = M_PI / 8;
    double max_angle = 7 * M_PI / 8;
    double y_offset = 0.5;
    double x_offset = 0.5;
    double z_offset = 0.5;

    for (int p = 0; p < 6; p += 2) {
        for (double t = min_angle; t < max_angle; t += 0.1) {
            xs[i] = R * cos(t) + x_offset;
            zs[i] = R * sin(t) + z_offset;
            ys[i] = step_size * p + y_offset;
            qs[i] = ToQuaternion(M_PI, (M_PI/4.0)-(2.0/3.0)*(t-(M_PI/8.0)), 0.0);
            i++;
        }
        
        xs[i] = R * cos(max_angle) + x_offset;
        zs[i] = R * sin(max_angle) + z_offset;
        ys[i] = step_size * p + y_offset;
        qs[i] = ToQuaternion(M_PI, -M_PI/4.0, 0.0);
        i++;
        xs[i] = R * cos(max_angle) + x_offset;
        zs[i] = R * sin(max_angle) + z_offset;
        ys[i] = step_size * (p + 1) + y_offset;
        qs[i] = ToQuaternion(M_PI, -M_PI/4.0, 0.0);
        i++;
    
        for (double t = max_angle; t > min_angle ; t -= 0.1){
            xs[i] = R * cos(t) + x_offset;
            zs[i] = R * sin(t) + z_offset;
            ys[i] = step_size * (p + 1) + y_offset;
            qs[i] = ToQuaternion(M_PI, (M_PI/4.0)-(2.0/3.0)*(t-(M_PI/8.0)), 0.0);
            i++;
        }

        xs[i] = R * cos(min_angle) + x_offset;
        zs[i] = R * sin(min_angle) + z_offset;
        ys[i] = step_size * (p + 1) + y_offset;
        qs[i] = ToQuaternion(M_PI, M_PI/4.0, 0.0);
        i++;
        xs[i] = R * cos(min_angle) + x_offset;
        zs[i] = R * sin(min_angle) + z_offset;
        ys[i] = step_size * (p + 2) + y_offset;
        qs[i] = ToQuaternion(M_PI, M_PI/4.0, 0.0);
        i++;
    }

    for (int j = 0; j < i; j++){
        // Set a target Pose
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link_inertia";
        target_pose.pose.position.x = xs[j];
        target_pose.pose.position.y = ys[j];
        target_pose.pose.position.z = zs[j];
        target_pose.pose.orientation.x = qs[j].x;
        target_pose.pose.orientation.y = qs[j].y;
        target_pose.pose.orientation.z = qs[j].z;
        target_pose.pose.orientation.w = qs[j].w;

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
            RCLCPP_ERROR(logger, "Planing failed!");
            break;
        }
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}