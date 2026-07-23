"""Launch Cartographer mapping and RViz for Isaac Stage 4."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create Cartographer, occupancy-grid, optional driving, and RViz nodes."""
    package_share = get_package_share_directory("isaac_stage4_slam")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    start_drive = LaunchConfiguration("start_drive")
    start_adapter = LaunchConfiguration("start_adapter")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "start_adapter",
                default_value="true",
                description="Set false when run_training_nodes.sh already provides the Isaac adapter.",
            ),
            DeclareLaunchArgument(
                "start_drive",
                default_value="false",
                description="Start standalone wall avoidance; keep false while DRL controls /cmd_vel.",
            ),
            Node(
                package="turtlebot3_isaac_adapter",
                executable="gazebo_api_adapter",
                name="isaac_gazebo_api_adapter",
                output="screen",
                condition=IfCondition(start_adapter),
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                name="cartographer_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                remappings=[("scan", "slam_scan")],
                arguments=[
                    "-configuration_directory",
                    os.path.join(package_share, "config"),
                    "-configuration_basename",
                    "waffle_pi_2d.lua",
                ],
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_occupancy_grid_node",
                name="cartographer_occupancy_grid_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=["-resolution", "0.05", "-publish_period_sec", "1.0"],
            ),
            Node(
                package="isaac_stage4_slam",
                executable="wall_avoidance",
                name="isaac_wall_avoidance",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(start_drive),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", os.path.join(package_share, "rviz", "isaac_mapping.rviz")],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
