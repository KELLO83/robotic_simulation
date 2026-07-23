"""Native Isaac control and planar sensing for TurtleBot3 Waffle Pi."""

from __future__ import annotations

from dataclasses import dataclass
import math
import random


@dataclass(frozen=True)
class WafflePiRosHandle:
    """Prim paths and kinematics consumed by the ROS bridge."""

    robot_prim_path: str
    chassis_prim_path: str
    wheel_joint_names: tuple[str, str]
    wheel_radius: float
    wheel_base: float
    odom_frame_id: str = "odom"
    chassis_frame_id: str = "base_footprint"
    scan_frame_id: str = "base_scan"


class TurtleBot3WafflePi:
    """Build and control the reference Waffle Pi as one Isaac articulation."""

    def __init__(self, stage, config) -> None:
        """Reference the visual USD and add stable native PhysX components."""
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.core.experimental.prims import Articulation
        from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

        asset_path = config.waffle_pi_visual_usd_path
        if not asset_path.is_file():
            raise FileNotFoundError(f"Reference Waffle Pi asset was not found: {asset_path}")

        self._stage = stage
        self._config = config
        self._spawn_position = config.robot_spawn_xyz
        self._root_path = config.robot_root_path
        self._chassis_path = f"{self._root_path}/Geometry/base_footprint"
        self._left_wheel_path = f"{self._chassis_path}/wheel_left_link"
        self._right_wheel_path = f"{self._chassis_path}/wheel_right_link"

        stage_utils.add_reference_to_stage(str(asset_path), self._root_path)
        root_prim = _require_prim(stage, self._root_path)
        root_xform = UsdGeom.Xformable(root_prim)
        root_xform.ClearXformOpOrder()
        root_xform.AddTranslateOp().Set(Gf.Vec3d(*config.robot_spawn_xyz))

        chassis_prim = _require_prim(stage, self._chassis_path)
        left_wheel_prim = _require_prim(stage, self._left_wheel_path)
        right_wheel_prim = _require_prim(stage, self._right_wheel_path)

        UsdPhysics.RigidBodyAPI.Apply(chassis_prim).CreateRigidBodyEnabledAttr().Set(True)
        UsdPhysics.ArticulationRootAPI.Apply(chassis_prim)
        PhysxSchema.PhysxArticulationAPI.Apply(chassis_prim).CreateEnabledSelfCollisionsAttr().Set(False)
        chassis_mass = UsdPhysics.MassAPI.Apply(chassis_prim)
        chassis_mass.CreateMassAttr().Set(1.74300212)
        chassis_mass.CreateCenterOfMassAttr().Set(Gf.Vec3f(-0.0061, 0.0, 0.0193))
        chassis_mass.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(0.0152, 0.0161, 0.0215))

        for wheel_prim in (left_wheel_prim, right_wheel_prim):
            UsdPhysics.RigidBodyAPI.Apply(wheel_prim).CreateRigidBodyEnabledAttr().Set(True)
            wheel_mass = UsdPhysics.MassAPI.Apply(wheel_prim)
            wheel_mass.CreateMassAttr().Set(0.02849894)
            wheel_mass.CreateCenterOfMassAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            wheel_mass.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(1.1176e-5, 1.1192e-5, 2.0713e-5))

        wheel_material = _create_physics_material(stage, f"{self._root_path}/PhysicsMaterials/Wheel", 1.2, 1.0)
        caster_material = _create_physics_material(stage, f"{self._root_path}/PhysicsMaterials/Caster", 0.01, 0.01)
        body_material = _create_physics_material(stage, f"{self._root_path}/PhysicsMaterials/Body", 0.45, 0.35)

        for wheel_path in (self._left_wheel_path, self._right_wheel_path):
            inherited_collider = _require_prim(stage, f"{wheel_path}/cylinder")
            UsdPhysics.CollisionAPI(inherited_collider).CreateCollisionEnabledAttr().Set(False)
            wheel_collider = UsdGeom.Sphere.Define(stage, f"{wheel_path}/native_wheel_collision")
            wheel_collider.CreateRadiusAttr().Set(config.wheel_radius)
            wheel_collider.CreatePurposeAttr().Set(UsdGeom.Tokens.guide)
            UsdPhysics.CollisionAPI.Apply(wheel_collider.GetPrim()).CreateCollisionEnabledAttr().Set(True)
            _configure_contact(wheel_collider.GetPrim(), contact_offset=0.002)
            _bind_physics_material(wheel_collider.GetPrim(), wheel_material)

        # Sensor and imported body colliders otherwise make raycasts see the
        # robot itself. The official low-friction rear caster boxes remain.
        for inherited_name in ("box", "cylinder", "box_3"):
            inherited_collider = _require_prim(stage, f"{self._chassis_path}/{inherited_name}")
            UsdPhysics.CollisionAPI(inherited_collider).CreateCollisionEnabledAttr().Set(False)
        for caster_name in ("box_1", "box_2"):
            caster_collider = _require_prim(stage, f"{self._chassis_path}/{caster_name}")
            UsdPhysics.CollisionAPI(caster_collider).CreateCollisionEnabledAttr().Set(True)
            _configure_contact(caster_collider, contact_offset=0.002)
            _bind_physics_material(caster_collider, caster_material)

        chassis = UsdGeom.Cube.Define(stage, f"{self._chassis_path}/native_chassis_collision")
        chassis.CreateSizeAttr().Set(1.0)
        chassis.CreatePurposeAttr().Set(UsdGeom.Tokens.guide)
        chassis.AddTranslateOp().Set(Gf.Vec3d(-0.064, 0.0, 0.085))
        chassis.AddScaleOp().Set(Gf.Vec3f(0.266, 0.266, 0.07))
        UsdPhysics.CollisionAPI.Apply(chassis.GetPrim()).CreateCollisionEnabledAttr().Set(True)
        _configure_contact(chassis.GetPrim(), contact_offset=0.002)
        _bind_physics_material(chassis.GetPrim(), body_material)

        physics_path = f"{self._root_path}/Physics"
        _create_wheel_joint(
            stage,
            f"{physics_path}/{config.wheel_joint_names[0]}",
            self._chassis_path,
            self._left_wheel_path,
            Gf.Vec3f(0.0, config.wheel_base / 2.0, 0.033),
        )
        _create_wheel_joint(
            stage,
            f"{physics_path}/{config.wheel_joint_names[1]}",
            self._chassis_path,
            self._right_wheel_path,
            Gf.Vec3f(0.0, -config.wheel_base / 2.0, 0.033),
        )

        self._base_xform = UsdGeom.Xformable(chassis_prim)
        self._articulation = Articulation(self._chassis_path)
        self._left_dof_index = int(self._articulation.get_dof_indices(config.wheel_joint_names[0]).numpy()[0])
        self._right_dof_index = int(self._articulation.get_dof_indices(config.wheel_joint_names[1]).numpy()[0])

    def ros_handle(self) -> WafflePiRosHandle:
        """Return the ROS bridge view of this articulation."""
        return WafflePiRosHandle(
            robot_prim_path=self._chassis_path,
            chassis_prim_path=self._chassis_path,
            wheel_joint_names=self._config.wheel_joint_names,
            wheel_radius=self._config.wheel_radius,
            wheel_base=self._config.wheel_base,
        )

    def get_planar_scan(self, sample_count: int) -> list[float]:
        """Raycast a 360-degree scan from the Waffle Pi LDS pose."""
        import carb
        import omni.physx
        from pxr import Gf, Usd

        transform = self._base_xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        world_origin = transform.Transform(Gf.Vec3d(*self._config.lidar_translation))
        scene_query = omni.physx.get_physx_scene_query_interface()
        ranges = []
        for ray_index in range(sample_count):
            angle = 2.0 * math.pi * ray_index / sample_count
            local_direction = Gf.Vec3d(math.cos(angle), math.sin(angle), 0.0)
            world_direction = transform.TransformDir(local_direction).GetNormalized()
            hit = scene_query.raycast_closest(
                carb.Float3(*world_origin),
                carb.Float3(*world_direction),
                self._config.scan_max_range,
            )
            if not hit.get("hit"):
                ranges.append(math.inf)
                continue
            noisy_distance = float(hit["distance"]) + random.gauss(0.0, 0.01)
            if noisy_distance < self._config.scan_min_range:
                ranges.append(math.inf)
            else:
                ranges.append(min(noisy_distance, self._config.scan_max_range))
        return ranges

    def reset(self) -> None:
        """Restore the initial pose and clear chassis and wheel velocities."""
        import numpy as np

        self._articulation.set_world_poses(
            positions=np.array([self._spawn_position], dtype=np.float32),
            orientations=np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32),
        )
        zero_root_velocity = np.zeros((1, 3), dtype=np.float32)
        self._articulation.set_velocities(zero_root_velocity, zero_root_velocity)
        indices = [self._left_dof_index, self._right_dof_index]
        zero_wheel_velocity = np.zeros((1, 2), dtype=np.float32)
        self._articulation.set_dof_velocities(zero_wheel_velocity, dof_indices=indices)
        self._articulation.set_dof_velocity_targets(zero_wheel_velocity, dof_indices=indices)

    def drive(self, linear_velocity: float, angular_velocity: float) -> None:
        """Apply a planar Twist and animate both Waffle Pi wheels."""
        import numpy as np

        left, right = wheel_angular_velocities(
            linear_velocity,
            angular_velocity,
            self._config.wheel_radius,
            self._config.wheel_base,
        )
        indices = [self._left_dof_index, self._right_dof_index]
        wheel_targets = np.array([[left, right]], dtype=np.float32)
        self._articulation.set_dof_velocities(wheel_targets, dof_indices=indices)
        self._articulation.set_dof_velocity_targets(wheel_targets, dof_indices=indices)

        _, orientations = self._articulation.get_world_poses()
        quaternion = orientations.numpy()[0] if hasattr(orientations, "numpy") else np.asarray(orientations)[0]
        w, x, y, z = quaternion
        forward_x = 1.0 - 2.0 * (y * y + z * z)
        forward_y = 2.0 * (x * y + w * z)
        chassis_linear = np.array(
            [[linear_velocity * forward_x, linear_velocity * forward_y, 0.0]],
            dtype=np.float32,
        )
        chassis_angular = np.array([[0.0, 0.0, angular_velocity]], dtype=np.float32)
        self._articulation.set_velocities(chassis_linear, chassis_angular)


def wheel_angular_velocities(
    linear_velocity: float,
    angular_velocity: float,
    wheel_radius: float,
    wheel_base: float,
) -> tuple[float, float]:
    """Convert differential-drive body velocity into wheel angular velocity."""
    half_angular_component = angular_velocity * wheel_base / 2.0
    left = (linear_velocity - half_angular_component) / wheel_radius
    right = (linear_velocity + half_angular_component) / wheel_radius
    return left, right


def _create_wheel_joint(stage, path: str, chassis_path: str, wheel_path: str, position) -> None:
    from pxr import Gf, Sdf, UsdPhysics

    joint = UsdPhysics.RevoluteJoint.Define(stage, path)
    joint.CreateAxisAttr().Set(UsdPhysics.Tokens.z)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(chassis_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(wheel_path)])
    joint.CreateLocalPos0Attr().Set(position)
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(0.7073883, -0.7068252, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateTypeAttr().Set(UsdPhysics.Tokens.force)
    drive.CreateStiffnessAttr().Set(0.0)
    drive.CreateDampingAttr().Set(1.5)
    drive.CreateMaxForceAttr().Set(2.0)
    drive.CreateTargetVelocityAttr().Set(0.0)


def _create_physics_material(stage, path: str, static_friction: float, dynamic_friction: float):
    from pxr import UsdPhysics, UsdShade

    material = UsdShade.Material.Define(stage, path)
    physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_material.CreateStaticFrictionAttr().Set(static_friction)
    physics_material.CreateDynamicFrictionAttr().Set(dynamic_friction)
    physics_material.CreateRestitutionAttr().Set(0.0)
    return material


def _bind_physics_material(prim, material) -> None:
    from pxr import UsdShade

    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        material,
        bindingStrength=UsdShade.Tokens.strongerThanDescendants,
        materialPurpose="physics",
    )


def _configure_contact(prim, contact_offset: float) -> None:
    from pxr import PhysxSchema

    collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    collision.CreateContactOffsetAttr().Set(contact_offset)
    collision.CreateRestOffsetAttr().Set(0.0)


def _require_prim(stage, path: str):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise RuntimeError(f"Required Waffle Pi prim is missing: {path}")
    return prim
