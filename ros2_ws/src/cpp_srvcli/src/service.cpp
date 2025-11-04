#include "rclcpp/rclcpp.hpp"
#include "cpp_servcli/srv/back.hpp"
#include "sensor_msgs/msg/joy.hpp"


class ModeServer : public rclcpp::Node
{
public:
  ModeServer()
  : Node("mode_server")
  {
    service_ = this->create_service<cpp_servcli::srv::Back>("mode_service",
        [this](auto request, auto response) {
          this->handle_service(request, response);
    });

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
      [this](sensor_msgs::msg::Joy::UniquePtr msg) {
        latest_joy_ = *msg;
            });

    RCLCPP_INFO(this->get_logger(), "Server setup");
  }

private:
  void handle_service(
    const std::shared_ptr<cpp_servcli::srv::Back::Request> request,
    std::shared_ptr<cpp_servcli::srv::Back::Response> response)
  {
    int index = request->button_index;
    if (index >= 0 && index < latest_joy_.buttons.size()) {
      response->change = latest_joy_.buttons[index];
    } else {
      response->change = false;
    }
  }

  rclcpp::Service<cpp_servcli::srv::Back>::SharedPtr service_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  sensor_msgs::msg::Joy latest_joy_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ModeServer>());
  rclcpp::shutdown();
  return 0;
}
