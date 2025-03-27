from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ros2_basic_agent'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*'))  # Add this line

    ],
    install_requires=['setuptools','langchain','langchain-openai','python-dotenv'],
    zip_safe=True,
    maintainer='Lentin Joseph',
    maintainer_email='runtimerobotics@gmail.com',
    description='Basic ROS 2 AI agent package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_ai_agent_basic = ros2_basic_agent.ros2_ai_agent_basic:main',            
            'ros2_ai_agent_basic_tools = ros2_basic_agent.ros2_ai_agent_basic_tools:main',            
            'ros2_ai_agent_turtlesim = ros2_basic_agent.ros2_ai_agent_turtlesim:main',
            'ros2_ai_agent_nav2 = ros2_basic_agent.ros2_ai_agent_turtlebo3_nav2:main',
            'ros2_ai_agent_moveit2 = ros2_basic_agent.ros2_ai_agent_ur_moveit2:main',            
        ],
    },
)
