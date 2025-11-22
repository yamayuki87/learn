
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  using moveit::planning_interface::MoveGroupInterface;
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");
  // rclcpp::Parameter srdf_param;
  // for (int i = 0; i < 10; ++i) {
  //   if (node->has_parameter("robot_description_semantic") ||
  //     node->get_parameter("robot_description_semantic", srdf_param)) break;
  //   RCLCPP_WARN(logger, "Waiting for robot_description_semantic...");
  //   std::this_thread::sleep_for(std::chrono::seconds(1));
  // }
  auto move_group_interface = MoveGroupInterface(node, "sekirei_arm");


  // Create the MoveIt MoveGroup Interface
 
 

  // Set a target Pose
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = 0.2;
    msg.position.z = 1.0;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
  {
    move_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
