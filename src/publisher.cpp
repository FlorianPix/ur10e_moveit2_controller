#define _USE_MATH_DEFINES
#include <math.h>

#include <memory>

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class Publisher : public rclcpp::Node
{
public:
    double timer_interval = 0.5;
    int count = 0;
    std_msgs::msg::Int32 finished_msg = std_msgs::msg::Int32();

    Publisher() : Node("publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("/z", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Publisher::timer_callback, this));
    }

    void timer_callback()
    {
        // publish the message
        finished_msg.data = count;
        RCLCPP_INFO(this->get_logger(), "%i", finished_msg.data);
        this->publisher_->publish(finished_msg);

        // increment the count
        count++;
    }

private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Publisher>());
    rclcpp::shutdown();
    return 0;
}