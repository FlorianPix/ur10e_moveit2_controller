#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

class SimCameraAdapter : public rclcpp::Node
{
public:
  SimCameraAdapter() : Node("sim_camera_adapter")
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(0.01s, std::bind(&SimCameraAdapter::on_timer, this));

    publisher_camera_pose_ = this->create_publisher<geometry_msgs::msg::Pose>("/depth_camera/pose", 10);
  }

private:
  void on_timer()
  {
    // Store frame names in variables that will be used to
    // compute transformations
    std::string fromFrameRel = "world";
    std::string toFrameRel = "depth_camera/link/depth_camera1";

    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform(
        fromFrameRel, toFrameRel,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(), "Could not transform %s to %s: %s",
        toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
      return;
    }

    auto pose_msg = geometry_msgs::msg::Pose();
    pose_msg.position.x = t.transform.translation.x;
    pose_msg.position.y = t.transform.translation.y;
    pose_msg.position.z = t.transform.translation.z;
    // RCLCPP_INFO(this->get_logger(), "X: %f, Y: %f, Z: %f", pose_msg.position.x, pose_msg.position.y, pose_msg.position.z);
    pose_msg.orientation.x = t.transform.rotation.w;
    pose_msg.orientation.y = t.transform.rotation.x;
    pose_msg.orientation.z = t.transform.rotation.y;
    pose_msg.orientation.w = t.transform.rotation.z;
    publisher_camera_pose_->publish(pose_msg);
  }

  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_camera_pose_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimCameraAdapter>());
  rclcpp::shutdown();
  return 0;
}