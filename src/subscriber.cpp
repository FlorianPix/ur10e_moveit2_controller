#define _USE_MATH_DEFINES
#include <math.h>

#include <memory>

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/bool.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class Subscriber : public rclcpp::Node
{
public:
    double timer_interval = 0.5;
    int count = 0;
    std_msgs::msg::Bool finished_msg = std_msgs::msg::Bool();

    Subscriber() : Node("subscriber")
    {
        subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "/finished",
            10,
            std::bind(&Subscriber::callback, this, _1)
        );
    }

    void callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "trajectory finished");
        } else {
            RCLCPP_INFO(this->get_logger(), "trajectory in progress");
        }
    }

private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Subscriber>());
    rclcpp::shutdown();
    return 0;
}