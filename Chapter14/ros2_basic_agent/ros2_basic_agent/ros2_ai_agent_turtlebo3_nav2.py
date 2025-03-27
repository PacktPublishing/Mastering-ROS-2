#!/usr/bin/env python3

"""
This script defines a ROS2 node that integrates with OpenAI's language model to control a TurtleBot3 robot.
The node subscribes to a topic and processes incoming messages using an AI agent with predefined tools for navigation and pose retrieval.

Classes:
    ROS2AIAgent(Node): A ROS2 node that subscribes to a topic and uses an AI agent to process messages and control the robot.

Functions:
    move_to_goal(x: float, y: float) -> str: Navigates the robot to specified x, y coordinates.
    get_current_pose() -> str: Retrieves the current position and orientation of the robot.
    pose_callback(msg: Odometry): Callback function to update the robot's pose.
    prompt_callback(msg: String): Callback function to process incoming messages.
    main(args=None): Initializes and spins the ROS2 node.

Dependencies:
    - os
    - math
    - geometry_msgs.msg (PoseStamped)
    - nav_msgs.msg (Odometry)
    - rclpy.action (ActionClient)
    - nav2_msgs.action (NavigateToPose)
    - std_msgs.msg (String)
    - rclpy (rclpy, Node)
    - ament_index_python.packages (get_package_share_directory)
    - pathlib (Path)
    - transforms3d.euler (quat2euler)
    - langchain.agents (AgentExecutor, create_openai_tools_agent)
    - langchain_openai (ChatOpenAI)
    - langchain.tools (tool)
    - langchain_core.prompts (ChatPromptTemplate, MessagesPlaceholder)
    - dotenv (load_dotenv)
"""

import os
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

from std_msgs.msg import String
import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from pathlib import Path

from transforms3d.euler import quat2euler  # Replace tf_transformations import
from langchain.agents import AgentExecutor, create_openai_tools_agent
from langchain_openai import ChatOpenAI
from langchain.tools import BaseTool, StructuredTool, tool
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
from dotenv import load_dotenv

class ROS2AIAgent(Node):
    def __init__(self):
        super().__init__('ros2_ai_agent')
        self.get_logger().info('ROS2 AI Agent has been started')
        
        # Initialize robot pose
        self.current_pose = PoseStamped()
        
        # Create action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Create subscriber for robot pose
        self.pose_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.pose_callback,
            10
        )

        # Create tools using class methods
        self.move_to_goal_tool = tool(self.move_to_goal)
        self.get_current_pose_tool = tool(self.get_current_pose)

        self.prompt = ChatPromptTemplate.from_messages([
            ("system", """You are a TurtleBot3 control assistant.
            You can control the robot using these commands:
            - move_to_goal(x, y): Navigate robot to specified x,y coordinates
            - get_current_pose(): Get current position and orientation of robot
            
            Return only the necessary actions and their results. e.g
            Human: Move the robot to position x=1.0, y=2.0
            AI: Navigating to position x: 1.0, y: 2.0
            """),
            MessagesPlaceholder("chat_history", optional=True),
            ("human", "{input}"),
            MessagesPlaceholder("agent_scratchpad"),
        ])

        share_dir = get_package_share_directory('ros2_basic_agent')
        config_dir = share_dir + '/config' + '/openai.env'
        load_dotenv(Path(config_dir))

        # setup the toolkit with the decorated class methods
        self.toolkit = [self.move_to_goal_tool, self.get_current_pose_tool]

        # Choose the LLM that will drive the agent
        self.llm = ChatOpenAI(model="gpt-4o-mini", temperature=0)

        # Construct the OpenAI Tools agent
        self.agent = create_openai_tools_agent(self.llm, self.toolkit, self.prompt)

        # Create an agent executor
        self.agent_executor = AgentExecutor(agent=self.agent, tools=self.toolkit, verbose=True)

        # Create the subscriber for prompts
        self.subscription = self.create_subscription(
            String,
            'prompt',
            self.prompt_callback,
            10
        )

    def move_to_goal(self, x: float, y: float) -> str:
        """Navigate robot to specified x,y coordinates."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.w = 1.0
        
        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            return "Navigation server not available"
        
        # Send goal
        self.nav_client.send_goal_async(goal_msg)
        return f"Navigating to position x: {x}, y: {y}"

    def get_current_pose(self) -> str:
        """Get current pose of the robot."""
        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y
        orientation = self.current_pose.pose.orientation
        roll, pitch, yaw = quat2euler([
            orientation.w,
            orientation.x,
            orientation.y,
            orientation.z
        ])
        return f"x: {x:.2f}, y: {y:.2f}, theta: {math.degrees(yaw):.2f} degrees"

    def pose_callback(self, msg):
        """Callback to update robot's pose"""
        self.current_pose.pose = msg.pose.pose
        self.current_pose.header = msg.header

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

if __name__ == '__main__':
    main()
