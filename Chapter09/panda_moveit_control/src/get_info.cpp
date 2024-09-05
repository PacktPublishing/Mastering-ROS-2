#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
//#include <moveit_visual_tools/moveit_visual_tools.h>
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

#define TEST 2

int main(int argc, char** argv)
{

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();


  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the ``JointModelGroup``. Throughout MoveIt, the terms "planning group" and "joint model group"
  // are used interchangeably.
  static const std::string PLANNING_GROUP = "arm";

  // The
  // :moveit_codedir:`MoveGroupInterface<moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h>`
  // class can be easily set up using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);


  // We will use the
  // :moveit_codedir:`PlanningSceneInterface<moveit_ros/planning_interface/planning_scene_interface/include/moveit/planning_scene_interface/planning_scene_interface.h>`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  //RCLCPP_INFO_STREAM(
  //  move_group_node->get_logger(),
  //  "This controller will not be used in chained mode. The references will be ignored!"); 
    

      // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  //RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  //std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
  //          std::ostream_iterator<std::string>(std::cout, ", "));


  #if TEST == 1
    // Start the demo
    geometry_msgs::msg::Pose target_pose1;
    target_pose1.orientation.x = 0.92;
    target_pose1.orientation.y = -0.383;
    target_pose1.orientation.z = 0.0;
    target_pose1.orientation.w = 0.0;
    

    target_pose1.position.x = 0.38;
    target_pose1.position.y = -0.2;
    target_pose1.position.z = 0.4;
    move_group.setPoseTarget(target_pose1);

    // Now, we call the planner to compute the plan and visualize it.
    // Note that we are just planning, not asking move_group
    // to actually move the robot.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;


    RCLCPP_INFO(LOGGER, "Doing");


    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "SUCCESS" : "FAILED");
    move_group.move(); 
    
    

  // To start, we'll create an pointer that references the current robot's state.
    // RobotState is the object that contains all the current position/velocity/acceleration data.
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
    //
    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    /*
    // Now, let's modify one of the joints, plan to the new joint space goal, and visualize the plan.
    joint_group_positions[0] = -1.2;  // radians
    bool within_bounds = move_group.setJointValueTarget(joint_group_positions);
    if (!within_bounds)
    {
      RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
    }

    // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
    // The default values are 10% (0.1).
    // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
    // or set explicit factors in your code if you need your robot to move faster.
    */
    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);

    success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Visualizing plan 2 (joint space goal) %s", success ? "SUCCESS" : "FAILED");


    move_group.move(); 
  #endif 
  #if TEST == 2

  
    geometry_msgs::msg::Pose target_pose1;
    target_pose1.orientation.x = 0.0;
    target_pose1.orientation.y = 0.0;
    target_pose1.orientation.z = 0.0;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.3;
    target_pose1.position.y = 0.0;
    target_pose1.position.z = 0.7;
    move_group.setPoseTarget(target_pose1);

    // Now, we call the planner to compute the plan and visualize it.
    // Note that we are just planning, not asking move_group
    // to actually move the robot.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;


    RCLCPP_INFO(LOGGER, "Doing");


    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    // Planning to a joint-space goal
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // Let's set a joint space goal and move towards it.  This will replace the
    // pose target we set above.
    //
    // To start, we'll create an pointer that references the current robot's state.
    // RobotState is the object that contains all the current position/velocity/acceleration data.
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
    //
    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // Now, let's modify one of the joints, plan to the new joint space goal, and visualize the plan.
    joint_group_positions[0] = 1.0;  // radians
    bool within_bounds = move_group.setJointValueTarget(joint_group_positions);
    if (!within_bounds)
    {
      RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
    }

    // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
    // The default values are 10% (0.1).
    // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
    // or set explicit factors in your code if you need your robot to move faster.
    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);

    success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
    move_group.move(); 


    #endif

    #if TEST == 3 

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        // Planning to a joint-space goal
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        //
        // Let's set a joint space goal and move towards it.  This will replace the
        // pose target we set above.
        //
        // To start, we'll create an pointer that references the current robot's state.
        // RobotState is the object that contains all the current position/velocity/acceleration data.
        moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
        //
        // Next get the current set of joint values for the group.
        std::vector<double> joint_group_positions;
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

        // Now, let's modify one of the joints, plan to the new joint space goal, and visualize the plan.
        joint_group_positions[0] = 0.0;  // radians
        joint_group_positions[1] = -0.78;  // radians
        joint_group_positions[2] = 0.0;  // radians
        joint_group_positions[3] = -2.35;  // radians
        joint_group_positions[4] = 0.0;  // radians
        joint_group_positions[5] = 1.57;  // radians
        joint_group_positions[6] = 0.78;  // radians


        bool within_bounds = move_group.setJointValueTarget(joint_group_positions);
        if (!within_bounds)
        {
          RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
        }

        // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
        // The default values are 10% (0.1).
        // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
        // or set explicit factors in your code if you need your robot to move faster.
        move_group.setMaxVelocityScalingFactor(1.0);
        move_group.setMaxAccelerationScalingFactor(1.0);

        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(LOGGER, "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

        move_group.move(); 


    #endif

    #if TEST == 4 
      // Enforce Planning in Joint Space
      // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      //
      // Depending on the planning problem MoveIt chooses between
      // ``joint space`` and ``cartesian space`` for problem representation.
      // Setting the group parameter ``enforce_joint_model_state_space:true`` in
      // the ompl_planning.yaml file enforces the use of ``joint space`` for all plans.
      //
      // By default, planning requests with orientation path constraints
      // are sampled in ``cartesian space`` so that invoking IK serves as a
      // generative sampler.
      //
      // By enforcing ``joint space``, the planning process will use rejection
      // sampling to find valid requests. Please note that this might
      // increase planning time considerably.
      //
      // We will reuse the old goal that we had and plan to it.
      // Note that this will only work if the current state already
      // satisfies the path constraints. So we need to set the start
      // state to a new pose.
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;

      geometry_msgs::msg::Pose target_pose1;
      target_pose1.orientation.w = 1.0;
      target_pose1.position.x = 0.28;
      target_pose1.position.y = -0.2;
      target_pose1.position.z = 0.5;
      moveit::core::RobotState start_state(*move_group.getCurrentState());
      geometry_msgs::msg::Pose start_pose2;
      start_pose2.orientation.w = 1.0;
      start_pose2.position.x = 0.55;
      start_pose2.position.y = -0.05;
      start_pose2.position.z = 0.8;
      start_state.setFromIK(joint_model_group, start_pose2);
      move_group.setStartState(start_state);

      // Now, we will plan to the earlier pose target from the new
      // start state that we just created.
      move_group.setPoseTarget(target_pose1);

      // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
      // Let's increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
      move_group.setPlanningTime(10.0);

      bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      RCLCPP_INFO(LOGGER, "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

      move_group.move(); 
    #endif
    #if TEST == 5 
      // Cartesian Paths
      // ^^^^^^^^^^^^^^^
      // You can plan a Cartesian path directly by specifying a list of waypoints
      // for the end-effector to go through. Note that we are starting
      // from the new start state above.  The initial pose (start state) does not
      // need to be added to the waypoint list but adding it can help with visualizations

      geometry_msgs::msg::Pose start_pose2;
      start_pose2.orientation.w = 1.0;
      start_pose2.position.x = 0.55;
      start_pose2.position.y = -0.05;
      start_pose2.position.z = 0.8;

      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.push_back(start_pose2);

      geometry_msgs::msg::Pose target_pose3 = start_pose2;

      target_pose3.position.z -= 0.2;
      waypoints.push_back(target_pose3);  // down

      target_pose3.position.y -= 0.2;
      waypoints.push_back(target_pose3);  // right

      target_pose3.position.z += 0.2;
      target_pose3.position.y += 0.2;
      target_pose3.position.x -= 0.2;
      waypoints.push_back(target_pose3);  // up and left

      // We want the Cartesian path to be interpolated at a resolution of 1 cm
      // which is why we will specify 0.01 as the max step in Cartesian
      // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
      // Warning - disabling the jump threshold while operating real hardware can cause
      // large unpredictable motions of redundant joints and could be a safety issue
      moveit_msgs::msg::RobotTrajectory trajectory;
      const double jump_threshold = 0.0;
      const double eef_step = 0.01;
      double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
      RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

      move_group.execute(trajectory); 
    #endif

    /*
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();

    // The id of the object is used to identify it.
    collision_object.id = "box1";

    // Define a box to add to the world.
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.1;
    primitive.dimensions[primitive.BOX_Y] = 1.5;
    primitive.dimensions[primitive.BOX_Z] = 0.5;

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

    // Now, let's add the collision object into the world
    // (using a vector that could contain additional objects)
    RCLCPP_INFO(LOGGER, "Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);
    */


}