#!/usr/bin/env python3

"""
This script defines a ROS2 node that integrates with OpenAI's language model to interact with a ROS 2 system.
The node subscribes to a topic and processes incoming messages using an AI agent with predefined tools.

Classes:
    ROS2AIAgent(Node): A ROS2 node that subscribes to a topic and uses an AI agent to process messages.

Methods:
    prompt_callback(msg: String): Callback function to process incoming messages.
    list_topics() -> str: Lists all available ROS 2 topics.
    list_nodes() -> str: Lists all running ROS 2 nodes.
    list_services() -> str: Lists all available ROS 2 services.
    list_actions() -> str: Lists all available ROS 2 actions.
    main(args=None): Initializes and spins the ROS2 node.

Dependencies:
    - os
    - math
    - std_msgs.msg (String)
    - rclpy (rclpy, Node)
    - ament_index_python.packages (get_package_share_directory)
    - pathlib (Path)
    - transforms3d.euler (quat2euler)
    - langchain.agents (AgentExecutor, create_openai_tools_agent)
    - langchain_openai (ChatOpenAI)
    - langchain.tools (BaseTool, StructuredTool, tool)
    - langchain_core.prompts (ChatPromptTemplate, MessagesPlaceholder)
    - dotenv (load_dotenv)
    - subprocess
    - typing (List)
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
        

        # Create tools using class methods
        self.list_topics_tool = tool(self.list_topics)
        self.list_nodes_tool = tool(self.list_nodes)
        self.list_services_tool = tool(self.list_services)
        self.list_actions_tool = tool(self.list_actions)

        self.prompt = ChatPromptTemplate.from_messages([
            ("system", """You are a ROS 2 system information assistant.
            You can check ROS 2 system status using these commands:
            - list_topics(): List all available ROS 2 topics
            - list_nodes(): List all running ROS 2 nodes
            - list_services(): List all available ROS 2 services
            - list_actions(): List all available ROS 2 actions
            
            Return only the necessary information and results. e.g
            Human: Show me all running nodes
            AI: Here are the running ROS 2 nodes: [node list]
            """),
            MessagesPlaceholder("chat_history", optional=True),
            ("human", "{input}"),
            MessagesPlaceholder("agent_scratchpad"),
        ])


        share_dir = get_package_share_directory('ros2_basic_agent')
        config_dir = share_dir + '/config' + '/openai.env'
        load_dotenv(Path(config_dir))

        # setup the toolkit with the decorated class methods
        self.toolkit = [
            self.list_topics_tool,
            self.list_nodes_tool,
            self.list_services_tool,
            self.list_actions_tool
        ]

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


    def list_topics(self) -> str:
        """List all available ROS 2 topics."""
        try:
            result = subprocess.run(['ros2', 'topic', 'list'], 
                                 capture_output=True, text=True, check=True)
            topics = result.stdout.strip().split('\n')
            return f"Available ROS 2 topics:\n{result.stdout}"
        except subprocess.CalledProcessError as e:
            return f"Error listing topics: {str(e)}"

    def list_nodes(self) -> str:
        """List all running ROS 2 nodes."""
        try:
            result = subprocess.run(['ros2', 'node', 'list'], 
                                 capture_output=True, text=True, check=True)
            return f"Running ROS 2 nodes:\n{result.stdout}"
        except subprocess.CalledProcessError as e:
            return f"Error listing nodes: {str(e)}"

    def list_services(self) -> str:
        """List all available ROS 2 services."""
        try:
            result = subprocess.run(['ros2', 'service', 'list'], 
                                 capture_output=True, text=True, check=True)
            return f"Available ROS 2 services:\n{result.stdout}"
        except subprocess.CalledProcessError as e:
            return f"Error listing services: {str(e)}"

    def list_actions(self) -> str:
        """List all available ROS 2 actions."""
        try:
            result = subprocess.run(['ros2', 'action', 'list'], 
                                 capture_output=True, text=True, check=True)
            return f"Available ROS 2 actions:\n{result.stdout}"
        except subprocess.CalledProcessError as e:
            return f"Error listing actions: {str(e)}"

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
