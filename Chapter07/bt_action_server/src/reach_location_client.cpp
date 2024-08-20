#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <bt_action_server/action/reach_location.hpp>

using Action = bt_action_server::action::ReachLocation;

bool done = false;

void result_callback(const rclcpp_action::ClientGoalHandle<Action>::WrappedResult & result)
{

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal succeeded!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknown result code");
            break;
    }


    done = true;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("action_client_node");

    // Create a client for the Hello action
    auto client = rclcpp_action::create_client<Action>(
        node, "reach_location");

    // Wait for the action server to become available
    if (!client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
        return 1;
    }

    

    // Create a goal message
    auto goal = Action::Goal();
    goal.x = 4;
    goal.y = 4;
    goal.timeout = 10;
    std::cout << "Goal: " << goal.x << std::endl;
    
    // Send the goal and wait for the result
    auto send_goal_future = client->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(node, send_goal_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to send goal");
        return 1;
    }

    auto goal_handle = send_goal_future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
        return 1;
    }

    std::cout << "AQUI!" << std::endl;
    client->async_get_result(goal_handle, result_callback);

    /*
    // Wait for the result
    std::cout << "Pre get result" << std::endl;
    auto result_future = client->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(node, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to get result");
        return 1;
    }
    std::cout << "After get result" << std::endl;
    auto result = result_future.get();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(node->get_logger(), "Result: %d", result.result->success);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Action did not succeed");
    }
    */

    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}