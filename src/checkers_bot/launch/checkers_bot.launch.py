"""
ROS2 launch file for the checkers bot.

Launches all nodes required for a game of English checkers:
    - game_manager: State machine and AI orchestrator
    - perception_node: Camera → board state (placeholder)
    - manipulation_node: UR5e motion execution (placeholder)

Usage:
    ros2 launch checkers_bot checkers_bot.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('checkers_bot')

    # Launch arguments
    search_time_arg = DeclareLaunchArgument(
        'search_time',
        default_value='5.0',
        description='AI search time per move (seconds)',
    )

    human_colour_arg = DeclareLaunchArgument(
        'human_colour',
        default_value='2',
        description='Human colour: 1=white, 2=black',
    )

    # Game manager node
    game_manager_node = Node(
        package='checkers_bot',
        executable='game_manager',
        name='game_manager',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'robot_params.yaml'),
            os.path.join(pkg_dir, 'config', 'board_params.yaml'),
        ],
    )

    # Perception node
    perception_node = Node(
        package='checkers_bot',
        executable='perception',
        name='perception_node',
        output='screen',
        parameters=[{
            'rgb_topic': '/camera/color/image_raw',
            'depth_topic': '/camera/depth/image_rect_raw',
            'publish_rate': 2.0,
        }],
    )

    # Manipulation node
    manipulation_node = Node(
        package='checkers_bot',
        executable='manipulation',
        name='manipulation_node',
        output='screen',
    )

    return LaunchDescription([
        search_time_arg,
        human_colour_arg,
        game_manager_node,
        perception_node,
        manipulation_node,
    ])
