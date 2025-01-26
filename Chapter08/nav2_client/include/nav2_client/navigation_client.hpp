#ifndef NAVIGATION_CLIENT_HPP_
#define NAVIGATION_CLIENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <map>
#include <string>

struct Location {
    double x;
    double y;
    double z;
};

class NavigationClient : public rclcpp::Node {
public:
    NavigationClient();
    bool navigateToPosition(double x, double y, double z, double orientation_w = 1.0);
    bool deliverMedicine(int order_number);

private:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    std::map<std::string, Location> locations_;
    
    bool goal_accepted_;
    bool navigation_complete_;
    bool navigation_successful_;
    std::shared_ptr<GoalHandleNavigateToPose> current_goal_handle_;

    void goalResponseCallback(std::shared_ptr<GoalHandleNavigateToPose> goal_handle);
    void feedbackCallback(GoalHandleNavigateToPose::SharedPtr goal_handle,
                         const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    void resultCallback(const GoalHandleNavigateToPose::WrappedResult & result);
};

#endif