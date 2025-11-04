#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "../src/joystick_listener.cpp"  // 簡単なテスト用なので直接include

TEST(JoystickListenerTest, ReceivesJoyMessage) {// test case name(category), test name
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<JoystickListener>();
  auto publisher = node->create_publisher<sensor_msgs::msg::Joy>("joy", 10);

  auto msg = sensor_msgs::msg::Joy();
  msg.axes = {0.5, -0.5};
  msg.buttons = {1, 0};

  // Publish message
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  publisher->publish(msg);

  
  //callbackが呼ばれるようにspin_someを複数回呼ぶ
  //spinは手動で止めるまで回り続けるので、テストではspin_someを使う
  rclcpp::spin_some(node);//一瞬だけspin
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  rclcpp::spin_some(node);

  
  auto received = node->get_last_msg();//サブスクリプションで受け取ったメッセージを取得
  ASSERT_NE(received, nullptr); // nullでないことを確認　ASSERT_NEは等しくないことを確認するマクロ
  EXPECT_EQ(received->axes[0], 0.5);//EXPECT_EQは等しいことを確認するマクロ
  EXPECT_EQ(received->buttons[0], 1);

  rclcpp::shutdown();
}
