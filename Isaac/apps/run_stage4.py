#!/usr/bin/env python3
"""Run TurtleBot3 DRL Stage 4 in Isaac Sim 6."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))


def parse_arguments() -> argparse.Namespace:
    """Parse arguments before creating the Isaac SimulationApp."""
    parser = argparse.ArgumentParser(description="Run the Gazebo-compatible TurtleBot3 Stage 4 in Isaac Sim.")
    parser.add_argument("--headless", action="store_true", help="Run without an Isaac GUI window.")
    parser.add_argument("--no-ros", action="store_true", help="Build and simulate the scene without ROS 2 bridges.")
    parser.add_argument(
        "--test-frames",
        type=int,
        default=0,
        help="Exit after this many app frames; zero runs until the window closes.",
    )
    return parser.parse_args()


ARGS = parse_arguments()

from isaacsim import SimulationApp


SIMULATION_APP = SimulationApp({"headless": ARGS.headless})


def main() -> None:
    """Build Stage 4, start physics and ROS bridges, then run the app loop."""
    import carb
    import isaacsim.core.experimental.utils.app as app_utils
    import isaacsim.core.experimental.utils.stage as stage_utils
    import omni.timeline
    import omni.usd
    from isaacsim.core.simulation_manager import SimulationManager

    from Isaac.config import IsaacStage4Config
    from Isaac.robot.turtlebot3_waffle_pi import TurtleBot3WafflePi
    from Isaac.ros_bridge.local_control import LocalControlServer
    from Isaac.ros_bridge.omnigraph_bridge import IsaacRos2Bridge
    from Isaac.world.stage4_world import Stage4World

    config = IsaacStage4Config()
    control_server = None
    ros_bridge = None
    drive_linear = 0.0
    drive_angular = 0.0

    Path("/tmp/drlnav_current_stage.txt").write_text(f"{config.stage_number}\n", encoding="utf-8")
    omni.usd.get_context().new_stage()
    SIMULATION_APP.update()
    SIMULATION_APP.update()
    stage_utils.set_stage_up_axis("Z")
    stage_utils.set_stage_units(meters_per_unit=1.0)
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("Isaac did not create a USD stage")

    carb.log_info("Building TurtleBot3 DRL Stage 4 world")
    world = Stage4World(stage, config)

    carb.log_info("Creating the reference TurtleBot3 Waffle Pi")
    robot = TurtleBot3WafflePi(stage, config)
    SIMULATION_APP.update()

    SimulationManager.setup_simulation(dt=1.0 / config.physics_hz, device="cpu")
    physics_scenes = SimulationManager.get_physics_scenes()
    if physics_scenes:
        physics_scenes[0].set_enabled_gpu_dynamics(False)

    if not ARGS.no_ros:
        carb.log_info("Creating ROS 2 graph and local Humble control channel")
        IsaacRos2Bridge.enable_extension()
        SIMULATION_APP.update()
        ros_bridge = IsaacRos2Bridge(config)
        ros_bridge.initialize(robot.ros_handle(), world.obstacle_prim_paths)

        control_server = LocalControlServer()

    app_utils.play()
    SIMULATION_APP.update()
    robot.reset()
    world.reset(omni.timeline.get_timeline_interface().get_current_time())
    carb.log_info("Isaac Stage 4 is ready")

    frame_count = 0
    while SIMULATION_APP.is_running():
        timeline = omni.timeline.get_timeline_interface()
        if timeline.is_playing():
            robot.drive(drive_linear, drive_angular)
            world.update(timeline.get_current_time())
            if ros_bridge is not None:
                ros_bridge.update_scan(robot)
        if control_server is not None:
            for command in control_server.poll():
                command_name = command["command"]
                if command_name == "pause":
                    app_utils.pause()
                elif command_name == "play":
                    app_utils.play()
                elif command_name == "reset":
                    drive_linear = 0.0
                    drive_angular = 0.0
                    robot.reset()
                    world.reset(timeline.get_current_time())
                elif command_name == "drive":
                    try:
                        drive_linear = float(command["linear"])
                        drive_angular = float(command["angular"])
                    except (KeyError, TypeError, ValueError):
                        carb.log_warn(f"Ignoring malformed drive command: {command!r}")
                elif command_name == "goal":
                    try:
                        world.set_goal(float(command["x"]), float(command["y"]))
                    except (KeyError, TypeError, ValueError):
                        carb.log_warn(f"Ignoring malformed goal command: {command!r}")
        SIMULATION_APP.update()
        frame_count += 1
        if ARGS.test_frames > 0 and frame_count >= ARGS.test_frames:
            break

    if control_server is not None:
        control_server.close()
    app_utils.stop()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback

        traceback.print_exc()
        raise
    finally:
        SIMULATION_APP.close()
