"""Conservative dense-scan wall avoidance for autonomous map collection."""

from __future__ import annotations

import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class WallAvoidance(Node):
    """Drive the Waffle Pi through open space while avoiding Stage 4 walls."""

    def __init__(self) -> None:
        """Create scan input, velocity output, and the control timer."""
        super().__init__("isaac_wall_avoidance")
        self._command_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self._scan_subscription = self.create_subscription(
            LaserScan,
            "/slam_scan",
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self._latest_scan: LaserScan | None = None
        self._control_timer = self.create_timer(0.1, self._control_tick)

    @staticmethod
    def minimum_range(scan: LaserScan, start_degrees: float, end_degrees: float) -> float:
        """Return the nearest valid scan sample inside an angular sector."""
        values = []
        for index, value in enumerate(scan.ranges):
            outside_limits = value < scan.range_min or value > scan.range_max
            if not math.isfinite(value) or outside_limits:
                continue
            angle = scan.angle_min + index * scan.angle_increment
            degrees = math.degrees(math.atan2(math.sin(angle), math.cos(angle)))
            if start_degrees <= degrees <= end_degrees:
                values.append(value)
        return min(values, default=scan.range_max)

    def _scan_callback(self, scan: LaserScan) -> None:
        """Store the newest planar scan for the next control update."""
        self._latest_scan = scan

    def _control_tick(self) -> None:
        """Publish a Waffle Pi-safe velocity based on nearby obstacles."""
        command = Twist()
        if self._latest_scan is None:
            self._command_publisher.publish(command)
            return

        front = self.minimum_range(self._latest_scan, -25.0, 25.0)
        left = self.minimum_range(self._latest_scan, 30.0, 100.0)
        right = self.minimum_range(self._latest_scan, -100.0, -30.0)
        if front < 0.48:
            command.angular.z = 0.8 if left >= right else -0.8
        elif front < 0.8:
            command.linear.x = 0.08
            command.angular.z = 0.45 if left >= right else -0.45
        elif left < 0.4:
            command.linear.x = 0.14
            command.angular.z = -0.35
        elif right < 0.4:
            command.linear.x = 0.14
            command.angular.z = 0.35
        else:
            command.linear.x = 0.16
        self._command_publisher.publish(command)

    def stop(self) -> None:
        """Publish an explicit stop command before shutdown."""
        self._command_publisher.publish(Twist())


def main(args: list[str] | None = None) -> None:
    """Run the optional Isaac wall-avoidance node."""
    rclpy.init(args=args)
    node = WallAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
