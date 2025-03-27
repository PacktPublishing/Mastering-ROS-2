#!/usr/bin/env python3

"""
This script defines a ROS2 node that integrates with OpenAI's language model to control a turtle in the turtlesim simulation.
The node subscribes to a topic and processes incoming messages using an AI agent with predefined tools.

Classes:
    ROS2AIAgent(Node): A ROS2 node that subscribes to a topic and uses an AI agent to control the turtle.

Functions:
    move_forward(distance: float) -> str: Moves the turtle forward by the specified distance.
    rotate(angle: float) -> str: Rotates the turtle by the specified angle in degrees.
    get_pose() -> str: Gets the current position and orientation of the turtle.
    pose_callback(msg: Pose): Callback function to update the turtle's pose.
    prompt_callback(msg: String): Callback function to process incoming messages.
    main(args=None): Initializes and spins the ROS2 node.

Dependencies:
    - os
    - math
    - geometry_msgs.msg (Twist)
    - turtlesim.msg (Pose)
    - langchain.agents (AgentExecutor, create_openai_tools_agent)
    - langchain_openai (ChatOpenAI)
    - langchain.tools (BaseTool, StructuredTool, tool)
    - langchain_core.prompts (ChatPromptTemplate, MessagesPlaceholder)
    - dotenv (load_dotenv)
    - std_msgs.msg (String)
    - rclpy (rclpy, Node)
"""

import os
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import String
import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from pathlib import Path

from langchain.agents import AgentExecutor, create_openai_tools_agent
from langchain_openai import ChatOpenAI
from langchain.tools import BaseTool, StructuredTool, tool
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
from dotenv import load_dotenv


class ROS2AIAgent(Node):
    def __init__(self):
        super().__init__('ros2_ai_agent')
        self.get_logger().info('ROS2 AI Agent has been started')
        
        # Initialize turtle pose
        self.turtle_pose = Pose()
        
        # Create publisher for turtle commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Create subscriber for turtle pose
        self.pose_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        # setup the tools as class methods
        @tool
        def move_forward(distance: float) -> str:
            """Move turtle forward by specified distance."""
            msg = Twist()
            duration = float(distance)

            msg.linear.x = duration * 0.1
            
            # Publish for the calculated duration
            self.cmd_vel_pub.publish(msg)
            #self.create_timer(duration, lambda: self.cmd_vel_pub.publish(Twist()))
            #self.create_timer(duration, lambda: self.cmd_vel_pub.publish(msg))
            return f"Moved forward {distance} units"

        @tool
        def rotate(angle: float) -> str:
            """Rotate turtle by specified angle in degrees (positive for counterclockwise)."""
            msg = Twist()
            msg.angular.z = math.radians(float(angle))
            duration = 1.0  # Time to complete rotation
            
            self.cmd_vel_pub.publish(msg)
            self.create_timer(duration, lambda: self.cmd_vel_pub.publish(Twist()))
            return f"Rotated {angle} degrees"

        @tool
        def get_pose() -> str:
            """Get current pose of the turtle."""
            return f"x: {self.turtle_pose.x:.2f}, y: {self.turtle_pose.y:.2f}, theta: {math.degrees(self.turtle_pose.theta):.2f} degrees"

        self.prompt = ChatPromptTemplate.from_messages([
            ("system", """You are a turtle control assistant for ROS 2 turtlesim.
            You can control the turtle using these commands:
            - move_forward(distance): Move turtle forward by specified distance
            - rotate(angle): Rotate turtle by specified angle in degrees
            - get_pose(): Get current position and orientation of turtle
            
            Return only the necessary actions and their results. e.g
            Human: Move the turtle forward 2 units
            AI: Moving forward 2 units
            """),
            MessagesPlaceholder("chat_history", optional=True),
            ("human", "{input}"),
            MessagesPlaceholder("agent_scratchpad"),
        ])

        share_dir = get_package_share_directory('ros2_basic_agent')
        config_dir = share_dir + '/config' + '/openai.env'
        load_dotenv(Path(config_dir))

        # setup the toolkit with the class methods
        self.toolkit = [move_forward, rotate, get_pose]

        # Choose the LLM that will drive the agent
        self.llm = ChatOpenAI(model="gpt-4o-mini", temperature=0)

        # Construct the OpenAI Tools agent
        self.agent = create_openai_tools_agent(self.llm, self.toolkit, self.prompt)

        # Create an agent executor by passing in the agent and tools
        self.agent_executor = AgentExecutor(agent=self.agent, tools=self.toolkit, verbose=True)

        # Create the subscriber for prompts
        self.subscription = self.create_subscription(
            String,
            'prompt',
            self.prompt_callback,
            10
        )

    def pose_callback(self, msg):
        """Callback to update turtle's pose"""
        self.turtle_pose = msg

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
