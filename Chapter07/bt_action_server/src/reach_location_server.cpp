#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <random>
#include "bt_action_server/action/reach_location.hpp"

class ReachLocationActionServer : public rclcpp::Node {
public:
    using ReachLocation = bt_action_server::action::ReachLocation;
    using GoalHandleReachLocation = rclcpp_action::ServerGoalHandle<ReachLocation>;

    explicit ReachLocationActionServer()
    : Node("reach_location_action_server") {
        // Your initialization code
        action_server_ = rclcpp_action::create_server<ReachLocation>(
            this,
            "reach_location",
            std::bind(&ReachLocationActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ReachLocationActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&ReachLocationActionServer::handle_accepted, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Action server started.");
    }
private:
    rclcpp_action::Server<ReachLocation>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ReachLocation::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request for location (%f, %f) with timeout %f",
                    goal->x, goal->y, goal->timeout);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleReachLocation> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleReachLocation> goal_handle) {
        std::thread{std::bind(&ReachLocationActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleReachLocation> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal...");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<ReachLocation::Feedback>();
        auto result = std::make_shared<ReachLocation::Result>();

        float current_x = 0.0;
        float current_y = 0.0;
        float target_x = goal->x;
        float target_y = goal->y;
        float timeout = goal->timeout;

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> velocity_dist(0.1, 2.0); // Velocity between 0.1 and 2.0

        rclcpp::Rate rate(1.0); // 1 Hz loop rate
        auto start_time = this->now();

        while ((this->now() - start_time).seconds() < timeout) {
            float velocity = velocity_dist(gen);

            current_x += velocity;  // Simplified movement model, assumes movement in x direction only
            current_y += velocity;  // Simplified movement model, assumes movement in y direction only

            feedback->current_x = current_x;
            feedback->current_y = current_y;
            goal_handle->publish_feedback(feedback);

            RCLCPP_INFO(this->get_logger(), "Current position: (%f, %f)", current_x, current_y);

            if (current_x >= target_x && current_y >= target_y) {
                result->success = true;
                result->message = "Target reached successfully.";
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                return;
            }

            rate.sleep();
        }

        result->success = false;
        result->message = "Failed to reach the target within the given timeout.";
        goal_handle->abort(result);
        RCLCPP_INFO(this->get_logger(), "Goal failed");
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ReachLocationActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
