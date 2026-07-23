"""Simulator-independent geometry and motion specification for Stage 4."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class BoxSpec:
    """A static axis-aligned box with an optional yaw rotation."""

    name: str
    center: tuple[float, float, float]
    size: tuple[float, float, float]
    yaw: float = 0.0


@dataclass(frozen=True)
class MotionKeyframe:
    """A relative planar position at a point in a repeating animation."""

    time_seconds: float
    xy: tuple[float, float]


@dataclass(frozen=True)
class MovingObstacleSpec:
    """Geometry and repeating keyframes for one Stage 4 obstacle."""

    name: str
    base_xy: tuple[float, float]
    period_seconds: float
    keyframes: tuple[MotionKeyframe, ...]
    radius: float = 0.16
    height: float = 0.50


OUTER_WALLS = (
    BoxSpec("outer_wall_left", (-2.425, 0.0, 0.25), (5.0, 0.15, 0.5), math.pi / 2),
    BoxSpec("outer_wall_top", (0.0, 2.425, 0.25), (5.0, 0.15, 0.5)),
    BoxSpec("outer_wall_right", (2.425, 0.0, 0.25), (5.0, 0.15, 0.5), -math.pi / 2),
    BoxSpec("outer_wall_bottom", (0.0, -2.425, 0.25), (5.0, 0.15, 0.5), math.pi),
)

INNER_WALLS = (
    BoxSpec("inner_wall_1", (-2.0, -1.5, 0.25), (1.0, 0.15, 0.5)),
    BoxSpec("inner_wall_2", (-0.5, -2.0, 0.25), (1.0, 0.15, 0.5), -math.pi / 2),
    BoxSpec("inner_wall_3", (1.0, -1.0, 0.25), (1.0, 0.15, 0.5), math.pi / 2),
    BoxSpec("inner_wall_4", (1.2, 1.9, 0.25), (1.0, 0.15, 0.5), -math.pi / 2),
    BoxSpec("inner_wall_5", (1.9, 0.4, 0.25), (1.0, 0.15, 0.5)),
    BoxSpec("inner_wall_6", (-0.5, 1.5, 0.25), (1.0, 0.15, 0.5)),
    BoxSpec("inner_wall_7", (-1.2, 0.092, 0.25), (1.0, 0.15, 0.5), -math.pi / 2),
)

MOVING_OBSTACLES = (
    MovingObstacleSpec(
        name="obstacle1",
        base_xy=(2.0, 2.0),
        period_seconds=160.0,
        keyframes=tuple(
            MotionKeyframe(time_seconds, xy)
            for time_seconds, xy in (
                (0, (0.0, 0.0)),
                (10, (-0.5, -1.0)),
                (50, (-3.5, -1.0)),
                (70, (-3.7, -3.0)),
                (90, (-3.5, -1.0)),
                (130, (-0.5, -1.0)),
                (140, (0.0, 0.0)),
                (160, (0.0, 0.0)),
            )
        ),
    ),
    MovingObstacleSpec(
        name="obstacle2",
        base_xy=(-2.0, -2.0),
        period_seconds=140.0,
        keyframes=tuple(
            MotionKeyframe(time_seconds, xy)
            for time_seconds, xy in (
                (0, (0.0, 0.0)),
                (10, (0.7, 0.2)),
                (40, (2.5, 3.5)),
                (55, (0.3, 3.5)),
                (85, (3.5, 1.8)),
                (100, (3.5, 0.0)),
                (110, (2.0, 0.5)),
                (115, (1.5, 1.0)),
                (120, (1.0, 0.5)),
                (125, (0.5, 0.1)),
                (130, (0.0, 0.0)),
                (140, (0.0, 0.0)),
            )
        ),
    ),
)


def interpolate_obstacle_xy(spec: MovingObstacleSpec, elapsed_seconds: float) -> tuple[float, float]:
    """Interpolate a moving obstacle's world position at simulation time."""
    local_time = elapsed_seconds % spec.period_seconds
    frames = spec.keyframes
    for start, end in zip(frames, frames[1:]):
        if start.time_seconds <= local_time <= end.time_seconds:
            duration = end.time_seconds - start.time_seconds
            ratio = 0.0 if duration == 0 else (local_time - start.time_seconds) / duration
            relative_x = start.xy[0] + ratio * (end.xy[0] - start.xy[0])
            relative_y = start.xy[1] + ratio * (end.xy[1] - start.xy[1])
            return spec.base_xy[0] + relative_x, spec.base_xy[1] + relative_y
    return spec.base_xy
