#!/usr/bin/env python3

"""
This script defines a ROS2 node that integrates with OpenAI's language model to control a UR robot.
The node uses MoveIt 2 (moveit_py) for motion planning and execution.
"""

import os
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, JointConstraint
from moveit_msgs.msg import BoundingVolume
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3
from langchain.agents import AgentExecutor, create_openai_tools_agent
from langchain_openai import ChatOpenAI
from langchain.tools import tool
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
from dotenv import load_dotenv
from tf2_ros import TransformListener, Buffer
from rclpy.duration import Duration
from ament_index_python.packages import get_package_share_directory

class ROS2AIAgent(Node):
    def __init__(self):
        super().__init__('ros2_ai_agent')
        self.get_logger().info('ROS2 AI Agent for UR MoveIt2 has been started')

        # Create action client
        self.move_action = ActionClient(self, MoveGroup, 'move_action')
        
        # Wait for action server
        self.get_logger().info('Waiting for move_action server...')
        self.move_action.wait_for_server()
        self.get_logger().info('Action server is available!')

        # Initialize tf2 listener for getting current pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create tools
        self.move_to_pose_tool = tool(self.move_to_pose)
        self.get_current_pose_tool = tool(self.get_current_pose)
        self.move_to_named_target_tool = tool(self.move_to_named_target)

        # Define predefined positions
        self.goal_states = {
            'home': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],  # Home position
            'up': [0.0, -2.0, 0.0, -1.57, 0.0, 0.0]      # Up position
        }

        self.prompt = ChatPromptTemplate.from_messages([
            ("system", """You are a UR robot control assistant using MoveIt 2.
            You can control the robot using these commands:
            - move_to_pose(x, y, z): Move end effector to specified x,y,z coordinates
            - get_current_pose(): Get current position of the end effector
            - move_to_named_target(target_name): Move to predefined position (home, up)
            
            Return only the necessary actions and their results. e.g
            Human: Move the end effector to position x=0.5, y=0.0, z=0.5
            AI: Moving end effector to position x: 0.5, y: 0.0, z: 0.5
            Human: Move robot to home position
            AI: Moving robot to home position
            """),
            MessagesPlaceholder("chat_history", optional=True),
            ("human", "{input}"),
            MessagesPlaceholder("agent_scratchpad"),
        ])

        # Load OpenAI configuration
        share_dir = get_package_share_directory('ros2_basic_agent')
        config_dir = share_dir + '/config' + '/openai.env'
        load_dotenv(Path(config_dir))

        # Setup toolkit
        self.toolkit = [
            self.move_to_pose_tool, 
            self.get_current_pose_tool,
            self.move_to_named_target_tool
        ]

        # Initialize LLM and agent
        self.llm = ChatOpenAI(model="gpt-4o-mini", temperature=0)
        self.agent = create_openai_tools_agent(self.llm, self.toolkit, self.prompt)
        self.agent_executor = AgentExecutor(agent=self.agent, tools=self.toolkit, verbose=True)

        # Create prompt subscriber
        self.subscription = self.create_subscription(
            String,
            'prompt',
            self.prompt_callback,
            10
        )

    def move_to_pose(self, x: float, y: float, z: float) -> str:
        """Move robot end effector to specified x,y,z coordinates."""
        try:
            # Create goal pose
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "base_link"
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position = Point(x=x, y=y, z=z)
            goal_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            # Create motion plan request
            motion_request = MotionPlanRequest()
            motion_request.workspace_parameters.header.frame_id = "base_link"
            motion_request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
            motion_request.group_name = "ur_manipulator"
            motion_request.num_planning_attempts = 10
            motion_request.allowed_planning_time = 5.0
            motion_request.max_velocity_scaling_factor = 0.1
            motion_request.max_acceleration_scaling_factor = 0.1
            motion_request.goal_constraints = [self.create_pose_goal(goal_pose)]

            # Create goal message
            goal_msg = MoveGroup.Goal()
            goal_msg.request = motion_request
            goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True

            # Send goal
            self.get_logger().info(f'Sending goal position: x={x}, y={y}, z={z}')
            future = self.move_action.send_goal_async(goal_msg)
            future.add_done_callback(self.goal_response_callback)
            return f"Sending motion command to position x: {x}, y: {y}, z: {z}"

        except Exception as e:
            return f"Error sending goal: {str(e)}"

    def create_pose_goal(self, pose_stamped):
        """Create pose goal constraints"""
        constraints = Constraints()
        constraints.name = "pose_goal"
        
        # Add position constraints
        position_constraint = PositionConstraint()
        position_constraint.header = pose_stamped.header
        position_constraint.link_name = "tool0"
        position_constraint.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)
        
        # Define the constraint region
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.01]  # 1cm tolerance sphere
        
        bounding_volume = BoundingVolume()
        bounding_volume.primitives = [primitive]
        bounding_volume.primitive_poses = [pose_stamped.pose]
        
        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0
        
        constraints.position_constraints = [position_constraint]
        
        return constraints

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == 1:  # SUCCESS
            self.get_logger().info('Motion executed successfully')
        else:
            self.get_logger().error(f'Motion failed with error code: {result.error_code.val}')

    def get_current_pose(self) -> str:
        """Get current pose of the robot end effector using TF2."""
        try:
            # Get the transform from base_link to tool0
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'tool0',
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0)
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            return f"Current end effector position - x: {x:.3f}, y: {y:.3f}, z: {z:.3f}"
        except Exception as e:
            return f"Error getting current pose: {str(e)}"

    def move_to_named_target(self, target_name: str) -> str:
        """Move robot to a predefined joint configuration."""
        try:
            if target_name not in self.goal_states:
                return f"Unknown target position: {target_name}"

            joint_positions = self.goal_states[target_name]
            
            # Create motion plan request
            motion_request = MotionPlanRequest()
            motion_request.workspace_parameters.header.frame_id = "base_link"
            motion_request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
            motion_request.group_name = "ur_manipulator"
            motion_request.num_planning_attempts = 10
            motion_request.allowed_planning_time = 5.0
            motion_request.max_velocity_scaling_factor = 0.1
            motion_request.max_acceleration_scaling_factor = 0.1

            # Create joint constraints
            joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                         "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
            
            joint_constraints = []
            for name, position in zip(joint_names, joint_positions):
                constraint = JointConstraint()
                constraint.joint_name = name
                constraint.position = position
                constraint.tolerance_above = 0.01
                constraint.tolerance_below = 0.01
                constraint.weight = 1.0
                joint_constraints.append(constraint)

            # Set joint constraints
            constraints = Constraints()
            constraints.name = f"joint_goal_{target_name}"
            constraints.joint_constraints = joint_constraints
            motion_request.goal_constraints = [constraints]

            # Create goal message
            goal_msg = MoveGroup.Goal()
            goal_msg.request = motion_request
            goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True

            # Send goal
            self.get_logger().info(f'Moving to {target_name} position')
            future = self.move_action.send_goal_async(goal_msg)
            future.add_done_callback(self.goal_response_callback)
            return f"Moving robot to {target_name} position"

        except Exception as e:
            return f"Error moving to {target_name}: {str(e)}"

    def prompt_callback(self, msg):
        try:
            result = self.agent_executor.invoke({"input": msg.data})
            self.get_logger().info(f"Result: {result['output']}")
        except Exception as e:
            self.get_logger().error(f'Error processing prompt: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ROS2AIAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
