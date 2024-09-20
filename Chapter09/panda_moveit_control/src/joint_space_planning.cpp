#include <moveit/move_group_interface/move_group_interface.h>



int main(int argc, char** argv) {

	rclcpp::init(argc, argv);
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        auto node = rclcpp::Node::make_shared("JointSpacePlanning");
	moveit::planning_interface::MoveGroupInterface move_group(node, "arm");
	std::vector<double> des_joint_pos = {0.0, 0.0, 0.0, -0.35, 0.0, 1.57, 0.78};

	bool within_bounds = move_group.setJointValueTarget(des_joint_pos);
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

	des_joint_pos = {0.0, -0.78, 0.0, -2.35, 0.0, 1.57, 0.78};

	within_bounds = move_group.setJointValueTarget(des_joint_pos);

	move_group.setMaxVelocityScalingFactor(1.0);
	move_group.setMaxAccelerationScalingFactor(1.0);

	success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
	if (success)
		move_group.move(); 

}
