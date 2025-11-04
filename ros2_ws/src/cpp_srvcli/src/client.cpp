#include "rclcpp/rclcpp.hpp"
#include "cpp_srvcli/srv/back.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("string_service_client");
  auto client = node->create_client<your_package_name::srv::StringSrv>("string_service");

  while (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO(node->get_logger(), "wait");
  }

  auto request = std::make_shared<your_package_name::srv::StringSrv::Request>();
  request->request_data = "hello";

  auto result_future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node,
    result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(), "response" : "%s", result_future.get()->response_data.c_str());
  } else {
    RCLCPP_ERROR(node->get_logger(), "fail");
  }

  rclcpp::shutdown();
  return 0;
}
