#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>

class MoveItController : public rclcpp::Node {
public:
  MoveItController()
  : Node("moveit_controller") {  
    // Subscribers
    pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "hand_tracking/left_hand_pose", 10,
      std::bind(&MoveItController::pose_callback, this, std::placeholders::_1));

    gripper_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "hand_tracking/left_gripper_command", 10,
      std::bind(&MoveItController::gripper_callback, this, std::placeholders::_1));
  }

  void initialize_move_group() {
    move_group_interface_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "left_arm");
    hand_move_group_interface_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "left_hand");
  }

private:
  void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    if (!move_group_interface_) {
      RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface is not initialized!");
      return;
    }

    // Scale hand tracking coordinates to match robot workspace
    geometry_msgs::msg::Pose target_pose;
    // Scale the input values and apply a transformation to align with the robot's workspace
    target_pose.position.x = msg->position.x;  // Adjust scaling & offset
    target_pose.position.y = msg->position.y * 0.0005;
    target_pose.position.z = msg->position.z * 0.0005 + 0.8;

    target_pose.orientation = msg->orientation;

    // log target pose
    RCLCPP_INFO(this->get_logger(), "Received target pose: x=%f, y=%f, z=%f",
                target_pose.position.x, target_pose.position.y, target_pose.position.z);

    move_group_interface_->setPoseTarget(target_pose);

    auto const [success, plan] = [&] {
      moveit::planning_interface::MoveGroupInterface::Plan plan_msg;
      auto const ok = static_cast<bool>(move_group_interface_->plan(plan_msg));
      return std::make_pair(ok, plan_msg);
    }();

    if (success) {
      move_group_interface_->execute(plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed!");
    }
  }

  void gripper_callback(const std_msgs::msg::String::SharedPtr msg) {
    if(!hand_move_group_interface_) {
      RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface is not initialized!");
      return;
    }

    if (msg->data == "close") {
      RCLCPP_INFO(this->get_logger(), "Closing gripper");
      
      // Add actual gripper control logic
      hand_move_group_interface_->setNamedTarget("left_hand_close");
      hand_move_group_interface_->move();

    } else if (msg->data == "open") {
      RCLCPP_INFO(this->get_logger(), "Opening gripper");
      // Add actual gripper control logic
      hand_move_group_interface_->setNamedTarget("left_hand_open");
      hand_move_group_interface_->move();
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gripper_subscriber_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> hand_move_group_interface_;
};

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<MoveItController>();  
  node->initialize_move_group();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
