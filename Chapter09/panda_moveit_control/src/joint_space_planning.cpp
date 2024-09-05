#include <moveit/move_group_interface/move_group_interface.h>



int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("joint_space_planning", node_options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions = {0.0, 0.0, 0.0, -0.35, 0.0, 1.57, 0.78};

	bool within_bounds = move_group.setJointValueTarget(joint_group_positions);
	if( !within_bounds ) {
		  std::cout << "Joint outer bounds" << std::endl;
		  return -1;
	}

	move_group.setMaxVelocityScalingFactor(1.0);
	move_group.setMaxAccelerationScalingFactor(1.0);
	bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
	if (success)
		  move_group.move(); 

	sleep(3);

    joint_group_positions = {0.0, -0.78, 0.0, -2.35, 0.0, 1.57, 0.78};

	within_bounds = move_group.setJointValueTarget(joint_group_positions);
	
	move_group.setMaxVelocityScalingFactor(1.0);
	move_group.setMaxAccelerationScalingFactor(1.0);

	success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
	if (success)
		  move_group.move(); 

}