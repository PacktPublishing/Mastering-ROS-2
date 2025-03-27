#!/usr/bin/env python3

"""
This script defines a ROS 2 node that integrates with OpenAI's language model to interact with a ROS 2 system.
The node subscribes to a topic and processes incoming messages using an AI agent equipped with predefined tools.

Classes:
    ROS2AIAgent(Node): A ROS 2 node that uses an AI agent to process messages received on a subscribed topic.

Key Methods:
    - prompt_callback(msg: String): Handles incoming messages and processes them using the AI agent.
    - get_ros_distro() -> str: Retrieves the current ROS distribution name.
    - get_domain_id() -> str: Retrieves the current ROS domain ID.

Main Functionality:
    - Subscribes to a topic named 'prompt' to receive user queries.
    - Uses an AI agent with tools to provide system information about ROS 2, such as the ROS distribution and domain ID.

Dependencies:
    - ROS 2 libraries: rclpy, std_msgs.msg, ament_index_python.packages
    - OpenAI integration: langchain, langchain_openai
    - Environment management: dotenv
    - Utility libraries: os, pathlib, subprocess, transforms3d, typing
"""
import os

import math
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

import subprocess
from typing import List


class ROS2AIAgent(Node):
    def __init__(self):
        super().__init__('ros2_ai_agent')
        self.get_logger().info('ROS2 AI Agent has been started')

        # Create tools
        self.get_ros_distro_tool = tool(self.get_ros_distro)
        self.get_domain_id_tool = tool(self.get_domain_id)

        self.prompt = ChatPromptTemplate.from_messages([
            ("system", """You are a ROS 2 system information assistant.
            You can check ROS 2 system status using these commands:
            - get_ros_distro(): Get the current ROS distribution name
            - get_domain_id(): Get the current ROS_DOMAIN_ID
            
            Return only the necessary information and results. e.g
            Human: What ROS distribution am I using?
            AI: Current ROS distribution: humble
            Human: What is my ROS domain ID?
            AI: Current ROS domain ID: 0
            """),
            MessagesPlaceholder("chat_history", optional=True),
            ("human", "{input}"),
            MessagesPlaceholder("agent_scratchpad"),
        ])

        share_dir = get_package_share_directory('ros2_basic_agent')
        config_dir = share_dir + '/config' + '/openai.env'
        load_dotenv(Path(config_dir))

        # Setup the toolkit with both tools
        self.toolkit = [self.get_ros_distro_tool, self.get_domain_id_tool]

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


    def get_ros_distro(self) -> str:
        """Get the current ROS distribution name."""
        try:
            ros_distro = os.environ.get('ROS_DISTRO')
            if (ros_distro):
                return f"Current ROS distribution: {ros_distro}"
            else:
                return "ROS distribution environment variable (ROS_DISTRO) not set"
        except Exception as e:
            return f"Error getting ROS distribution: {str(e)}"

    def get_domain_id(self) -> str:
        """Get the current ROS domain ID."""
        try:
            domain_id = os.environ.get('ROS_DOMAIN_ID', '0')  # Default is 0 if not set
            return f"Current ROS domain ID: {domain_id}"
        except Exception as e:
            return f"Error getting ROS domain ID: {str(e)}"

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
