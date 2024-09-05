#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


int main(int argc, char** argv) {
    
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, "arm");



    geometry_msgs::msg::Pose start_pose;
    start_pose.orientation.w = 0;
    start_pose.orientation.x = -1.0;
    start_pose.position.x = 0.55;
    start_pose.position.y = -0.05;
    start_pose.position.z = 0.8;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(start_pose);

    geometry_msgs::msg::Pose target_pose = start_pose;

    target_pose.position.z -= 0.2;
    waypoints.push_back(target_pose);  // down

    target_pose.position.y -= 0.2;
    waypoints.push_back(target_pose);  // right

    target_pose.position.z += 0.2;
    target_pose.position.y += 0.2;
    target_pose.position.x -= 0.2;
    waypoints.push_back(target_pose);  // up and left

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    move_group.execute(trajectory); 
  

}