"""Launch the unchanged DRL environment and goal nodes with the Isaac facade."""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Start the compatibility facade followed by existing DRL nodes."""
    return LaunchDescription(
        [
            Node(
                package="turtlebot3_isaac_adapter",
                executable="gazebo_api_adapter",
                output="screen",
            ),
            # The upstream environment rejects the ``--ros-args`` suffix that
            # launch_ros adds even when no parameters are supplied.  Running
            # its normal, already-supported CLI preserves the node unchanged.
            ExecuteProcess(cmd=["ros2", "run", "turtlebot3_drl", "environment"], output="screen"),
            ExecuteProcess(cmd=["ros2", "run", "turtlebot3_drl", "gazebo_goals"], output="screen"),
        ]
    )
