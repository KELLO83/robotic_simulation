"""USD construction for the Gazebo-compatible Stage 4 arena."""

from __future__ import annotations

import math

from Isaac.world.stage4_spec import INNER_WALLS, MOVING_OBSTACLES, OUTER_WALLS, BoxSpec, interpolate_obstacle_xy


class Stage4World:
    """Own the Stage 4 USD geometry, goal marker, and moving obstacles."""

    def __init__(self, stage, config) -> None:
        """Build the complete arena in the supplied USD stage."""
        from pxr import Gf, UsdGeom, UsdLux

        self._stage = stage
        self._config = config
        self._episode_start_time = 0.0
        self._obstacle_translate_ops = {}

        UsdGeom.Xform.Define(stage, "/World")
        light = UsdLux.DistantLight.Define(stage, "/World/SunLight")
        light.CreateIntensityAttr(7000.0)

        self._add_box(
            BoxSpec("ground", (0.0, 0.0, -0.025), (6.0, 6.0, 0.05)),
            color=(0.55, 0.55, 0.55),
        )
        for wall in OUTER_WALLS:
            self._add_box(wall, color=(0.72, 0.36, 0.08))
        for wall in INNER_WALLS:
            self._add_box(wall, color=(0.95, 0.55, 0.12))

        for obstacle in MOVING_OBSTACLES:
            cylinder = self._add_cylinder(
                obstacle.name,
                obstacle.base_xy[0],
                obstacle.base_xy[1],
                obstacle.height / 2.0,
                obstacle.radius,
                obstacle.height,
                color=(0.95, 0.95, 0.95),
                kinematic=True,
            )
            xform = UsdGeom.Xformable(cylinder.GetPrim())
            translate_op = next(op for op in xform.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeTranslate)
            self._obstacle_translate_ops[obstacle.name] = translate_op

        goal = self._add_cylinder(
            "goal",
            0.5,
            0.0,
            0.0005,
            0.30,
            0.001,
            color=(1.0, 0.0, 0.0),
            collision=False,
        )
        goal_xform = UsdGeom.Xformable(goal.GetPrim())
        self._goal_translate_op = next(
            op for op in goal_xform.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeTranslate
        )

    @property
    def obstacle_prim_paths(self) -> tuple[str, str]:
        """Return obstacle prim paths in the environment node's expected order."""
        return tuple(f"/World/{spec.name}" for spec in MOVING_OBSTACLES)

    def update(self, simulation_time: float) -> None:
        """Advance both keyframe animations from the current episode origin."""
        from pxr import Gf

        elapsed = max(0.0, simulation_time - self._episode_start_time)
        for obstacle in MOVING_OBSTACLES:
            x, y = interpolate_obstacle_xy(obstacle, elapsed)
            self._obstacle_translate_ops[obstacle.name].Set(Gf.Vec3d(x, y, obstacle.height / 2.0))

    def reset(self, simulation_time: float) -> None:
        """Restart obstacle keyframes at their Gazebo initial positions."""
        self._episode_start_time = simulation_time
        self.update(simulation_time)

    def set_goal(self, x: float, y: float) -> None:
        """Move the visual, non-colliding goal marker."""
        from pxr import Gf

        self._goal_translate_op.Set(Gf.Vec3d(x, y, 0.0005))

    def _add_box(self, spec: BoxSpec, color: tuple[float, float, float]):
        from pxr import Gf, UsdGeom, UsdPhysics

        cube = UsdGeom.Cube.Define(self._stage, f"/World/{spec.name}")
        cube.CreateSizeAttr().Set(1.0)
        cube.CreateDisplayColorAttr().Set([Gf.Vec3f(*color)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        xform = UsdGeom.Xformable(cube.GetPrim())
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(*spec.center))
        xform.AddRotateZOp().Set(math.degrees(spec.yaw))
        xform.AddScaleOp().Set(Gf.Vec3f(*spec.size))
        return cube

    def _add_cylinder(
        self,
        name: str,
        x: float,
        y: float,
        z: float,
        radius: float,
        height: float,
        color: tuple[float, float, float],
        *,
        collision: bool = True,
        kinematic: bool = False,
    ):
        from pxr import Gf, UsdGeom, UsdPhysics

        cylinder = UsdGeom.Cylinder.Define(self._stage, f"/World/{name}")
        cylinder.CreateRadiusAttr().Set(radius)
        cylinder.CreateHeightAttr().Set(height)
        cylinder.CreateDisplayColorAttr().Set([Gf.Vec3f(*color)])
        if collision:
            UsdPhysics.CollisionAPI.Apply(cylinder.GetPrim())
        if kinematic:
            rigid_body = UsdPhysics.RigidBodyAPI.Apply(cylinder.GetPrim())
            rigid_body.CreateKinematicEnabledAttr().Set(True)
        xform = UsdGeom.Xformable(cylinder.GetPrim())
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
        return cylinder
