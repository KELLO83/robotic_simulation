"""Runtime configuration for the Isaac Stage 4 simulator."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class IsaacStage4Config:
    """Values shared by the Stage 4 world, robot, and ROS adapters."""

    repository_root: Path = Path(__file__).resolve().parents[2]
    stage_number: int = 4
    physics_hz: int = 100
    scan_hz: int = 50
    slam_scan_hz: int = 10

    graph_path: str = "/World/ROS2Graph"
    robot_root_path: str = "/World/TurtleBot3"
    goal_prim_path: str = "/World/goal"
    obstacle_prim_paths: tuple[str, str] = ("/World/obstacle1", "/World/obstacle2")

    wheel_joint_names: tuple[str, str] = ("wheel_left_joint", "wheel_right_joint")
    wheel_radius: float = 0.033
    wheel_base: float = 0.288
    robot_spawn_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    lidar_translation: tuple[float, float, float] = (-0.064, 0.0, 0.132)

    scan_samples: int = 40
    slam_scan_samples: int = 360
    scan_fov_degrees: float = 360.0
    scan_min_range: float = 0.12
    scan_max_range: float = 3.5

    topic_cmd_vel: str = "cmd_vel"
    topic_scan: str = "scan"
    topic_slam_scan: str = "slam_scan"
    topic_odom: str = "odom"
    topic_joint_states: str = "joint_states"
    topic_clock: str = "/clock"
    topic_tf: str = "/tf"
    topic_tf_static: str = "/tf_static"
    topic_goal_pose: str = "goal_pose"
    topic_obstacle_odom: str = "obstacle/odom"

    @property
    def scan_publish_step(self) -> int:
        """Return the integer physics-frame interval between scans."""
        if self.physics_hz % self.scan_hz != 0:
            raise ValueError("physics_hz must be an integer multiple of scan_hz")
        return self.physics_hz // self.scan_hz

    @property
    def slam_scan_publish_step(self) -> int:
        """Return the physics-frame interval for the denser SLAM scan."""
        if self.physics_hz % self.slam_scan_hz != 0:
            raise ValueError("physics_hz must be an integer multiple of slam_scan_hz")
        return self.physics_hz // self.slam_scan_hz

    @property
    def waffle_pi_visual_usd_path(self) -> Path:
        """Return the validated Waffle Pi visual asset from the reference implementation."""
        return (
            Path("/home/rr/turtlebot3_drlnav_humble/isaac")
            / "assets"
            / "turtlebot3_waffle_pi"
            / "generated"
            / "turtlebot3_waffle_pi"
            / "waffle_pi_visual_only.usda"
        )
