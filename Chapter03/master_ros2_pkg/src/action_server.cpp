#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "master_ros2_interface/action/my_custom_action.hpp"

class ActionServer : public rclcpp::Node {
public:
  using MyCustomAction = master_ros2_interface::action::MyCustomAction;
  using GoalHandle = rclcpp_action::ServerGoalHandle<MyCustomAction>;

  ActionServer() : Node("action_server") {
  
    RCLCPP_INFO(this->get_logger(), "Started Action Server Node");
    action_server_ = rclcpp_action::create_server<MyCustomAction>(
      this,
      "my_custom_action",
      std::bind(&ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&ActionServer::handle_accepted, this, std::placeholders::_1)
    );
  }

private:
  rclcpp_action::Server<MyCustomAction>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const MyCustomAction::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request with value %d", goal->goal_value);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    std::thread{std::bind(&ActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<MyCustomAction::Result>();
    auto feedback = std::make_shared<MyCustomAction::Feedback>();

    for (int i = 1; i <= goal->goal_value; ++i) {
      feedback->progress = i ;
      goal_handle->publish_feedback(feedback);
      rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    result->result_value = goal->goal_value;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

