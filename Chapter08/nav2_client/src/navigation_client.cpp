#include "nav2_client/navigation_client.hpp"
#include <chrono>
#include <thread>

NavigationClient::NavigationClient() : Node("navigation_client"),
    goal_accepted_(false),
    navigation_complete_(false),
    navigation_successful_(false)
{
    action_client_ = rclcpp_action::create_client<NavigateToPose>(
        this, "navigate_to_pose");
        
    locations_["initial_pose"] = {-0.04, -0.06, 0.0};
    locations_["medical_shop"] = {26.36, -6.18, 0.0};
    locations_["room1"] = {10.97, 4.0, 0.0};
    locations_["room2"] = {15.92, -4.0, 0.0};
    
    RCLCPP_INFO(get_logger(), "Navigation client initialized");
}

bool NavigationClient::navigateToPosition(double x, double y, double z, double orientation_w)
{
    navigation_complete_ = false;
    navigation_successful_ = false;
    goal_accepted_ = false;

    while (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_INFO(get_logger(), "Waiting for action server...");
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();
    
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.position.z = z;
    
    goal_msg.pose.pose.orientation.w = orientation_w;
    goal_msg.pose.pose.orientation.x = 0.0;
    goal_msg.pose.pose.orientation.y = 0.0;
    goal_msg.pose.pose.orientation.z = 0.0;

    RCLCPP_INFO(get_logger(), "Navigating to position: x=%.2f, y=%.2f, z=%.2f", x, y, z);
    
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&NavigationClient::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&NavigationClient::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&NavigationClient::resultCallback, this, std::placeholders::_1);

    auto goal_future = action_client_->async_send_goal(goal_msg, send_goal_options);
    
    if (rclcpp::spin_until_future_complete(shared_from_this(), goal_future) != 
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "Failed to send goal");
        return false;
    }

    // Wait for goal to complete
    while (!navigation_complete_ && rclcpp::ok()) {
        rclcpp::spin_some(shared_from_this());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!navigation_successful_) {
        RCLCPP_ERROR(get_logger(), "Navigation failed");
        return false;
    }

    // Add delay between consecutive goals
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    return true;
}

bool NavigationClient::deliverMedicine(int order_number)
{
    // Navigate to medical shop
    RCLCPP_INFO(get_logger(), "Moving to medical shop to collect medicine...");
    auto shop = locations_["medical_shop"];
    if (!navigateToPosition(shop.x, shop.y, shop.z)) {
        RCLCPP_ERROR(get_logger(), "Failed to reach medical shop");
        return false;
    }
    
    RCLCPP_INFO(get_logger(), "Collecting medicine...");
    std::this_thread::sleep_for(std::chrono::seconds(15));
    
    // Navigate to room
    std::string room = (order_number == 1) ? "room1" : "room2";
    RCLCPP_INFO(get_logger(), "Delivering medicine to %s...", room.c_str());
    auto room_loc = locations_[room];
    if (!navigateToPosition(room_loc.x, room_loc.y, room_loc.z)) {
        RCLCPP_ERROR(get_logger(), "Failed to reach room");
        return false;
    }
    
    RCLCPP_INFO(get_logger(), "Delivering medicine...");
    std::this_thread::sleep_for(std::chrono::seconds(10));
    
    // Return to initial position
    RCLCPP_INFO(get_logger(), "Returning to initial position...");
    auto initial = locations_["initial_pose"];
    if (!navigateToPosition(initial.x, initial.y, initial.z)) {
        RCLCPP_ERROR(get_logger(), "Failed to return to initial position");
        return false;
    }
    
    RCLCPP_INFO(get_logger(), "Delivery completed!");
    return true;
}

void NavigationClient::goalResponseCallback(
    std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
    if (!goal_handle) {
        goal_accepted_ = false;
        RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
    } else {
        goal_accepted_ = true;
        RCLCPP_INFO(get_logger(), "Goal accepted by server");
    }
}

void NavigationClient::feedbackCallback(
    GoalHandleNavigateToPose::SharedPtr goal_handle,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    (void)goal_handle;  // Unused
    double distance_remaining = feedback->distance_remaining;
    RCLCPP_INFO(get_logger(), "Distance remaining: %.2f", distance_remaining);
}

void NavigationClient::resultCallback(
    const GoalHandleNavigateToPose::WrappedResult & result)
{
    navigation_complete_ = true;
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            navigation_successful_ = true;
            RCLCPP_INFO(get_logger(), "Navigation successful");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            navigation_successful_ = false;
            RCLCPP_ERROR(get_logger(), "Navigation aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            navigation_successful_ = false;
            RCLCPP_ERROR(get_logger(), "Navigation canceled");
            break;
        default:
            navigation_successful_ = false;
            RCLCPP_ERROR(get_logger(), "Unknown result code");
            break;
    }
}