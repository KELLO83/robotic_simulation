"""Expose the Gazebo-shaped API expected by the unchanged DRL nodes.

ROS callbacks stay in system Python 3.10.  Simulation controls and the goal
marker cross a local datagram socket because Isaac Sim 6 embeds Python 3.12.
"""

import json
from pathlib import Path
import socket

from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose, Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import Empty


CONTROL_SOCKET_PATH = Path("/tmp/turtlebot3_isaac_control.sock")


class GazeboApiAdapter(Node):
    """Preserve Gazebo goal service names without running Gazebo."""

    def __init__(self) -> None:
        """Create compatibility services required by the existing nodes."""
        super().__init__("isaac_gazebo_api_adapter")
        self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self._delete_service = self.create_service(DeleteEntity, "/delete_entity", self._delete_entity)
        self._spawn_service = self.create_service(SpawnEntity, "/spawn_entity", self._spawn_entity)
        self._pause_service = self.create_service(Empty, "/pause_physics", self._pause)
        self._unpause_service = self.create_service(Empty, "/unpause_physics", self._play)
        self._reset_service = self.create_service(Empty, "/reset_simulation", self._reset)

        goal_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._goal_subscription = self.create_subscription(Pose, "/goal_pose", self._goal_changed, goal_qos)
        self._cmd_vel_subscription = self.create_subscription(Twist, "/cmd_vel", self._velocity_changed, 10)
        self.get_logger().info("Isaac Gazebo and physics compatibility services are ready")

    def destroy_node(self):
        """Release the datagram socket before the ROS node."""
        self._socket.close()
        return super().destroy_node()

    def _send_command(self, command: dict) -> bool:
        try:
            self._socket.sendto(json.dumps(command).encode("utf-8"), str(CONTROL_SOCKET_PATH))
        except (FileNotFoundError, ConnectionRefusedError, OSError) as error:
            self.get_logger().error(f"Isaac control socket is unavailable: {error}")
            return False
        return True

    def _goal_changed(self, message: Pose) -> None:
        self._send_command({"command": "goal", "x": message.position.x, "y": message.position.y})

    def _velocity_changed(self, message: Twist) -> None:
        self._send_command(
            {
                "command": "drive",
                "linear": message.linear.x,
                "angular": message.angular.z,
            }
        )

    def _pause(self, _request, response):
        self._send_command({"command": "pause"})
        return response

    def _play(self, _request, response):
        self._send_command({"command": "play"})
        return response

    def _reset(self, _request, response):
        self._send_command({"command": "reset"})
        return response

    @staticmethod
    def _delete_entity(_request, response):
        response.success = True
        response.status_message = "Isaac goal marker is managed through /goal_pose"
        return response

    @staticmethod
    def _spawn_entity(_request, response):
        response.success = True
        response.status_message = "Isaac goal marker is managed through /goal_pose"
        return response


def main(args=None) -> None:
    """Run the Gazebo API compatibility node."""
    rclpy.init(args=args)
    node = GazeboApiAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
