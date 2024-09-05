#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>



#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>



int main(int argc, char** argv) {
    bool success = false;
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, "arm");
    const moveit::core::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup("arm");


    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


   
    move_group.setStartState(*move_group.getCurrentState());
    geometry_msgs::msg::Pose another_pose;
    another_pose.orientation.w = 0;
    another_pose.orientation.x = -1.0;
    another_pose.position.x = 0.7;
    another_pose.position.y = 0.0;
    another_pose.position.z = 0.59;
    move_group.setPoseTarget(another_pose);


    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();

    
    collision_object.id = "box1";

    // Define a box to add to the world.
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.1;
    primitive.dimensions[primitive.BOX_Y] = 1.5;
    primitive.dimensions[primitive.BOX_Z] = 0.3;

    // Define a pose for the box (specified relative to frame_id).
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.48;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.25;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    planning_scene_interface.applyCollisionObject(collision_objects);

    //planning_scene_interface.addCollisionObjects(collision_objects);
    // Now, when we plan a trajectory it will avoid the obstacle.
    //bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
	//move_group.move(); 

    sleep(4);
    // Attaching objects to the robot
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // You can attach an object to the robot, so that it moves with the robot geometry.
    // This simulates picking up the object for the purpose of manipulating it.
    // The motion planning should avoid collisions between objects as well.
    moveit_msgs::msg::CollisionObject object_to_attach;
    object_to_attach.id = "cylinder1";

    shape_msgs::msg::SolidPrimitive cylinder_primitive;
    cylinder_primitive.type = primitive.CYLINDER;
    cylinder_primitive.dimensions.resize(2);
    cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.1;
    cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.04;

    // We define the frame/pose for this cylinder so that it appears in the gripper.
    object_to_attach.header.frame_id = move_group.getEndEffectorLink();
    geometry_msgs::msg::Pose grab_pose;
    grab_pose.orientation.w = 1.0;
    grab_pose.position.z = 0.3;

    // First, we add the object to the world (without using a vector).
    object_to_attach.primitives.push_back(cylinder_primitive);
    object_to_attach.primitive_poses.push_back(grab_pose);
    object_to_attach.operation = object_to_attach.ADD;
    planning_scene_interface.applyCollisionObject(object_to_attach);

    // Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
    // We also need to tell MoveIt that the object is allowed to be in collision with the finger links of the gripper.
    // You could also use applyAttachedCollisionObject to attach an object to the robot directly.
    std::vector<std::string> touch_links;
    touch_links.push_back("panda_rightfinger");
    touch_links.push_back("panda_leftfinger");
    move_group.attachObject(object_to_attach.id, "hand", touch_links);

    // Replan, but now with the object in hand.
    move_group.setStartStateToCurrentState();
    success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group.move(); 
    
    sleep(3);
    
    // The result may look something like this:
    //
    // .. image:: ./move_group_interface_tutorial_attached_object.gif
    //    :alt: animation showing the arm moving differently once the object is attached
    //
    // Detaching and Removing Objects
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // Now, let's detach the cylinder from the robot's gripper.
    
    move_group.detachObject(object_to_attach.id);

    
    std::vector<std::string> object_ids;
    object_ids.push_back(collision_object.id);
    object_ids.push_back(object_to_attach.id);
    planning_scene_interface.removeCollisionObjects(object_ids);

    
}