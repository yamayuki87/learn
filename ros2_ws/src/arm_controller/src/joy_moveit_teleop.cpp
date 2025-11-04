#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using std::placeholders::_1;

class JoyMoveItTeleop : public rclcpp::Node
{
public:
    JoyMoveItTeleop() : Node("joy_moveit_teleop"), 
                        speed_linear_(0.01),
                        speed_angular_(0.05)
    {
        // MoveGroupInterfaceのセットアップ
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "sekirei_arm");

        // 現在のposeを取得
        current_pose_ = move_group_->getCurrentPose().pose;

        // Joy購読
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoyMoveItTeleop::joyCallback, this, _1));

        // Pose出力確認用Publisher（任意）
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);

        RCLCPP_INFO(this->get_logger(), "Joy MoveIt Teleop initialized");
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // 軸数確認
        if (msg->axes.size() < 4) return;

        // 左スティック: X, Y 平行移動
        double dx = msg->axes[0] * speed_linear_;  // X方向
        double dy = msg->axes[1] * speed_linear_;  // Y方向

        // 右スティック: Z方向と回転
        double dz = msg->axes[4] * speed_linear_;  // 上下移動
        double dyaw = msg->axes[3] * speed_angular_; // 回転

        // 位置更新
        current_pose_.position.x += dx;
        current_pose_.position.y += dy;
        current_pose_.position.z += dz;

        // 簡易的なyaw回転
        double yaw = getYawFromQuaternion(current_pose_.orientation);
        yaw += dyaw;
        current_pose_.orientation = createQuaternionMsgFromYaw(yaw);

        // パブリッシュ（RViz確認用）
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "base_link";
        pose_msg.pose = current_pose_;
        pose_pub_->publish(pose_msg);

        // MoveItに目標Poseをセット
        move_group_->setPoseTarget(current_pose_);

        // 計画・実行
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            move_group_->execute(plan);
            RCLCPP_INFO(this->get_logger(), "Pose updated via joystick");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Planning failed");
        }
    }

    // Yawからクォータニオン生成
    geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw)
    {
        geometry_msgs::msg::Quaternion q;
        q.x = 0.0;
        q.y = 0.0;
        q.z = sin(yaw / 2.0);
        q.w = cos(yaw / 2.0);
        return q;
    }

    // 現在のYawを取得
    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion &q)
    {
        return atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    geometry_msgs::msg::Pose current_pose_;

    double speed_linear_;
    double speed_angular_;
};
  
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyMoveItTeleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
