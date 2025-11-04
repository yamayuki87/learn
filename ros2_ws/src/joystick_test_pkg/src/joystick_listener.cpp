#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class JoystickListener : public rclcpp::Node {
public:
  JoystickListener()
  : Node("joystick_listener")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      [this](sensor_msgs::msg::Joy::SharedPtr msg) {
        last_msg_ = msg;
        RCLCPP_INFO(this->get_logger(), "Received joy message");
      });
  }

  sensor_msgs::msg::Joy::SharedPtr get_last_msg() const
  {
    return last_msg_;
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  sensor_msgs::msg::Joy::SharedPtr last_msg_;
};
