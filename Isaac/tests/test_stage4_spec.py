"""Pure-Python contract tests for the Isaac Stage 4 configuration."""

import math

import pytest

from Isaac.config.runtime import IsaacStage4Config
from Isaac.robot.turtlebot3_waffle_pi import wheel_angular_velocities
from Isaac.world.stage4_spec import INNER_WALLS, MOVING_OBSTACLES, OUTER_WALLS, interpolate_obstacle_xy


def test_training_observation_contract() -> None:
    """The Isaac sensor dimensions must match the existing DRL model."""
    config = IsaacStage4Config()
    assert config.stage_number == 4
    assert config.scan_samples == 40
    assert config.scan_min_range == 0.12
    assert config.scan_max_range == 3.5
    assert config.wheel_radius == 0.033
    assert config.wheel_base == 0.288
    assert config.scan_publish_step == 2
    assert config.slam_scan_samples == 360
    assert config.slam_scan_publish_step == 10
    assert config.waffle_pi_visual_usd_path.is_file()


def test_stage4_geometry_count() -> None:
    """Stage 4 contains the four outer and seven inner Gazebo walls."""
    assert len(OUTER_WALLS) == 4
    assert len(INNER_WALLS) == 7
    assert len(MOVING_OBSTACLES) == 2


def test_obstacle_keyframes_match_gazebo_offsets() -> None:
    """Interpolation includes each obstacle's model include pose."""
    obstacle1, obstacle2 = MOVING_OBSTACLES
    assert interpolate_obstacle_xy(obstacle1, 0.0) == (2.0, 2.0)
    assert interpolate_obstacle_xy(obstacle1, 50.0) == (-1.5, 1.0)
    assert interpolate_obstacle_xy(obstacle1, 160.0) == (2.0, 2.0)
    assert interpolate_obstacle_xy(obstacle2, 40.0) == (0.5, 1.5)
    assert interpolate_obstacle_xy(obstacle2, 140.0) == (-2.0, -2.0)
    assert interpolate_obstacle_xy(obstacle2, 25.0) == pytest.approx((-0.4, -0.15))
    assert math.isclose(obstacle1.radius, 0.16)


def test_waffle_pi_differential_drive_conversion() -> None:
    """Body velocity must map to standard differential wheel targets."""
    left, right = wheel_angular_velocities(0.15, 0.0, 0.033, 0.288)
    assert left == pytest.approx(0.15 / 0.033)
    assert right == pytest.approx(0.15 / 0.033)

    left, right = wheel_angular_velocities(0.0, 1.0, 0.033, 0.288)
    assert left == pytest.approx(-0.144 / 0.033)
    assert right == pytest.approx(0.144 / 0.033)
