#define _USE_MATH_DEFINES
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

class RectangleController : public rclcpp::Node
{
public:
  RectangleController() : Node("rectangle_controller")
  {
      this->declare_parameter<double>("cropping.x_width", 0.0);
      this->declare_parameter<double>("cropping.y_width", 0.0);
      this->declare_parameter<double>("cropping.theta", 0.0);
      this->declare_parameter<double>("cropping.x_off", 0.0);
      this->declare_parameter<double>("cropping.y_off", 0.0);
      this->declare_parameter<double>("cropping.z_min", 0.0);
      this->declare_parameter<double>("cropping.z_max", 0.0);
      double y_width = this->get_parameter("cropping.y_width").as_double();
      double x_width = this->get_parameter("cropping.x_width").as_double();
      double theta = this->get_parameter("cropping.theta").as_double();
      double x_off = this->get_parameter("cropping.x_off").as_double();
      double y_off = this->get_parameter("cropping.y_off").as_double();
      double z_min = this->get_parameter("cropping.z_min").as_double();
      double z_max = this->get_parameter("cropping.z_max").as_double();
      RCLCPP_INFO(this->get_logger(), "test: %f", x_width);
      RCLCPP_INFO(this->get_logger(), "test: %f", y_width);
      RCLCPP_INFO(this->get_logger(), "test: %f", theta);
      RCLCPP_INFO(this->get_logger(), "test: %f", x_off);
      RCLCPP_INFO(this->get_logger(), "test: %f", y_off);
      RCLCPP_INFO(this->get_logger(), "test: %f", z_min);
      RCLCPP_INFO(this->get_logger(), "test: %f", z_max);
  }
private:
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RectangleController>());
  rclcpp::shutdown();
  return 0;
}