"""OmniGraph ROS 2 contract used by the unchanged DRL environment node."""

from __future__ import annotations


class IsaacRos2Bridge:
    """Publish simulator state and consume TurtleBot3 velocity commands."""

    def __init__(self, config) -> None:
        """Store graph configuration without importing Isaac modules early."""
        self._config = config
        self._scan_frame_count = 0
        self._scan_data_attribute = None
        self._slam_scan_data_attribute = None

    @staticmethod
    def enable_extension() -> None:
        """Enable Isaac's built-in ROS 2 bridge extension."""
        import isaacsim.core.experimental.utils.app as app_utils

        app_utils.enable_extension("isaacsim.ros2.bridge")

    def initialize(self, robot_handle, obstacle_prim_paths: tuple[str, str]) -> None:
        """Create the complete motion, clock, scan, and obstacle graph."""
        import omni.graph.core as og
        import usdrt.Sdf

        # Cartographer's SLAM input contract and the OmniGraph publishers that
        # provide it:
        #   /slam_scan <- PublishSlamLaserScan
        #   /odom      <- PublishOdometry
        #   /tf        <- PublishOdomTF
        #   /tf_static <- PublishScanTF
        create_nodes = [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
        ]
        set_values = []
        connections = []

        self._add_clock_publisher(create_nodes, set_values, connections)
        self._add_joint_state_publisher(create_nodes, set_values, connections, usdrt.Sdf, robot_handle)
        self._add_odometry_source(create_nodes, set_values, connections, usdrt.Sdf, robot_handle)
        self._add_odometry_publisher(create_nodes, set_values, connections, robot_handle)
        self._add_odom_tf_publisher(create_nodes, set_values, connections, robot_handle)
        self._add_scan_tf_publisher(create_nodes, set_values, connections, robot_handle)
        self._add_control_scan_publisher(create_nodes, set_values, connections, robot_handle)
        self._add_slam_scan_publisher(create_nodes, set_values, connections, robot_handle)

        context_nodes = (
            "PublishClock",
            "PublishJointState",
            "PublishOdometry",
            "PublishOdomTF",
            "PublishScanTF",
            "PublishLaserScan",
            "PublishSlamLaserScan",
        )
        timed_nodes = (
            "PublishClock",
            "PublishJointState",
            "PublishOdometry",
            "PublishOdomTF",
            "PublishScanTF",
            "PublishLaserScan",
            "PublishSlamLaserScan",
        )
        connections.extend(("Context.outputs:context", f"{node}.inputs:context") for node in context_nodes)
        connections.extend(("ReadSimTime.outputs:simulationTime", f"{node}.inputs:timeStamp") for node in timed_nodes)

        self._add_obstacle_odometry_publishers(
            create_nodes,
            set_values,
            connections,
            usdrt.Sdf,
            obstacle_prim_paths,
        )

        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": self._config.graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: create_nodes,
                keys.SET_VALUES: set_values,
                keys.CONNECT: connections,
            },
        )
        self._scan_data_attribute = og.Controller.attribute(
            f"{self._config.graph_path}/PublishLaserScan.inputs:linearDepthData"
        )
        self._slam_scan_data_attribute = og.Controller.attribute(
            f"{self._config.graph_path}/PublishSlamLaserScan.inputs:linearDepthData"
        )

    def _add_clock_publisher(self, create_nodes, set_values, connections) -> None:
        """Add the simulation clock publisher."""
        create_nodes.append(("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"))
        set_values.append(("PublishClock.inputs:topicName", self._config.topic_clock))
        connections.append(("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"))

    def _add_joint_state_publisher(self, create_nodes, set_values, connections, sdf, robot_handle) -> None:
        """Add the robot joint-state publisher."""
        create_nodes.append(("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"))
        set_values.extend(
            [
                ("PublishJointState.inputs:topicName", self._config.topic_joint_states),
                ("PublishJointState.inputs:targetPrim", [sdf.Path(robot_handle.robot_prim_path)]),
            ]
        )
        connections.append(("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"))

    @staticmethod
    def _add_odometry_source(create_nodes, set_values, connections, sdf, robot_handle) -> None:
        """Add the Isaac node that computes the robot's odometry state."""
        create_nodes.append(("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"))
        set_values.append(
            ("ComputeOdometry.inputs:chassisPrim", [sdf.Path(robot_handle.chassis_prim_path)])
        )
        connections.append(("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"))

    def _add_odometry_publisher(self, create_nodes, set_values, connections, robot_handle) -> None:
        """Add PublishOdometry, which provides Cartographer's /odom input."""
        create_nodes.append(("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"))
        set_values.extend(
            [
                ("PublishOdometry.inputs:topicName", self._config.topic_odom),
                ("PublishOdometry.inputs:odomFrameId", robot_handle.odom_frame_id),
                ("PublishOdometry.inputs:chassisFrameId", robot_handle.chassis_frame_id),
            ]
        )
        connections.extend(
            [
                ("ComputeOdometry.outputs:execOut", "PublishOdometry.inputs:execIn"),
                ("ComputeOdometry.outputs:position", "PublishOdometry.inputs:position"),
                ("ComputeOdometry.outputs:orientation", "PublishOdometry.inputs:orientation"),
                ("ComputeOdometry.outputs:linearVelocity", "PublishOdometry.inputs:linearVelocity"),
                ("ComputeOdometry.outputs:angularVelocity", "PublishOdometry.inputs:angularVelocity"),
            ]
        )

    @staticmethod
    def _add_odom_tf_publisher(create_nodes, set_values, connections, robot_handle) -> None:
        """Add PublishOdomTF, which provides the dynamic /tf odom-to-base transform."""
        create_nodes.append(
            ("PublishOdomTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree")
        )
        set_values.extend(
            [
                ("PublishOdomTF.inputs:parentFrameId", robot_handle.odom_frame_id),
                ("PublishOdomTF.inputs:childFrameId", robot_handle.chassis_frame_id),
            ]
        )
        connections.extend(
            [
                ("OnPlaybackTick.outputs:tick", "PublishOdomTF.inputs:execIn"),
                ("ComputeOdometry.outputs:position", "PublishOdomTF.inputs:translation"),
                ("ComputeOdometry.outputs:orientation", "PublishOdomTF.inputs:rotation"),
            ]
        )

    def _add_scan_tf_publisher(self, create_nodes, set_values, connections, robot_handle) -> None:
        """Add PublishScanTF, which provides the static /tf_static base-to-lidar transform."""
        create_nodes.append(
            ("PublishScanTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree")
        )
        set_values.extend(
            [
                ("PublishScanTF.inputs:topicName", self._config.topic_tf_static),
                ("PublishScanTF.inputs:parentFrameId", robot_handle.chassis_frame_id),
                ("PublishScanTF.inputs:childFrameId", robot_handle.scan_frame_id),
                ("PublishScanTF.inputs:translation", list(self._config.lidar_translation)),
                ("PublishScanTF.inputs:rotation", [0.0, 0.0, 0.0, 1.0]),
                ("PublishScanTF.inputs:staticPublisher", True),
            ]
        )
        connections.append(("OnPlaybackTick.outputs:tick", "PublishScanTF.inputs:execIn"))

    def _add_control_scan_publisher(self, create_nodes, set_values, connections, robot_handle) -> None:
        """Add the lower-density /scan publisher used by the DRL controller."""
        self._add_laser_scan_publisher(
            create_nodes=create_nodes,
            set_values=set_values,
            connections=connections,
            gate_name="ScanGate",
            publisher_name="PublishLaserScan",
            topic_name=self._config.topic_scan,
            frame_id=robot_handle.scan_frame_id,
            publish_step=self._config.scan_publish_step,
            publish_hz=self._config.scan_hz,
            sample_count=self._config.scan_samples,
        )

    def _add_slam_scan_publisher(self, create_nodes, set_values, connections, robot_handle) -> None:
        """Add PublishSlamLaserScan, which provides Cartographer's /slam_scan input."""
        self._add_laser_scan_publisher(
            create_nodes=create_nodes,
            set_values=set_values,
            connections=connections,
            gate_name="SlamScanGate",
            publisher_name="PublishSlamLaserScan",
            topic_name=self._config.topic_slam_scan,
            frame_id=robot_handle.scan_frame_id,
            publish_step=self._config.slam_scan_publish_step,
            publish_hz=self._config.slam_scan_hz,
            sample_count=self._config.slam_scan_samples,
        )

    def _add_laser_scan_publisher(
        self,
        *,
        create_nodes,
        set_values,
        connections,
        gate_name,
        publisher_name,
        topic_name,
        frame_id,
        publish_step,
        publish_hz,
        sample_count,
    ) -> None:
        """Add a rate-gated Isaac ROS 2 LaserScan publisher."""
        create_nodes.extend(
            [
                (gate_name, "isaacsim.core.nodes.IsaacSimulationGate"),
                (publisher_name, "isaacsim.ros2.bridge.ROS2PublishLaserScan"),
            ]
        )
        set_values.extend(
            [
                (f"{gate_name}.inputs:step", publish_step),
                (f"{publisher_name}.inputs:topicName", topic_name),
                (f"{publisher_name}.inputs:frameId", frame_id),
                (f"{publisher_name}.inputs:horizontalFov", self._config.scan_fov_degrees),
                (
                    f"{publisher_name}.inputs:horizontalResolution",
                    self._config.scan_fov_degrees / sample_count,
                ),
                (f"{publisher_name}.inputs:numCols", sample_count),
                (f"{publisher_name}.inputs:numRows", 1),
                (
                    f"{publisher_name}.inputs:depthRange",
                    [self._config.scan_min_range, self._config.scan_max_range],
                ),
                (f"{publisher_name}.inputs:rotationRate", float(publish_hz)),
                (f"{publisher_name}.inputs:azimuthRange", [0.0, 360.0]),
                (
                    f"{publisher_name}.inputs:linearDepthData",
                    [self._config.scan_max_range] * sample_count,
                ),
                (f"{publisher_name}.inputs:intensitiesData", [0.0] * sample_count),
            ]
        )
        connections.extend(
            [
                ("OnPlaybackTick.outputs:tick", f"{gate_name}.inputs:execIn"),
                (f"{gate_name}.outputs:execOut", f"{publisher_name}.inputs:execIn"),
            ]
        )

    def _add_obstacle_odometry_publishers(
        self,
        create_nodes,
        set_values,
        connections,
        sdf,
        obstacle_prim_paths,
    ) -> None:
        """Add odometry publishers for the moving Stage 4 obstacles."""
        for index, obstacle_path in enumerate(obstacle_prim_paths, start=1):
            compute_name = f"ComputeObstacle{index}Odometry"
            publish_name = f"PublishObstacle{index}Odometry"
            create_nodes.extend(
                [
                    (compute_name, "isaacsim.core.nodes.IsaacComputeOdometry"),
                    (publish_name, "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ]
            )
            set_values.extend(
                [
                    (f"{compute_name}.inputs:chassisPrim", [sdf.Path(obstacle_path)]),
                    (f"{publish_name}.inputs:topicName", self._config.topic_obstacle_odom),
                    (f"{publish_name}.inputs:odomFrameId", "world"),
                    (f"{publish_name}.inputs:chassisFrameId", f"obstacle{index}"),
                ]
            )
            connections.extend(
                [
                    ("OnPlaybackTick.outputs:tick", f"{compute_name}.inputs:execIn"),
                    (f"{compute_name}.outputs:execOut", f"{publish_name}.inputs:execIn"),
                    (f"{compute_name}.outputs:position", f"{publish_name}.inputs:position"),
                    (f"{compute_name}.outputs:orientation", f"{publish_name}.inputs:orientation"),
                    (f"{compute_name}.outputs:linearVelocity", f"{publish_name}.inputs:linearVelocity"),
                    (f"{compute_name}.outputs:angularVelocity", f"{publish_name}.inputs:angularVelocity"),
                    ("Context.outputs:context", f"{publish_name}.inputs:context"),
                    ("ReadSimTime.outputs:simulationTime", f"{publish_name}.inputs:timeStamp"),
                ]
            )

    def update_scan(self, robot) -> None:
        """Update the graph's range buffer at the configured sensor rate."""
        import omni.graph.core as og

        if self._scan_data_attribute is None or self._slam_scan_data_attribute is None:
            raise RuntimeError("IsaacRos2Bridge.initialize() must run before update_scan().")
        self._scan_frame_count += 1
        if self._scan_frame_count % self._config.scan_publish_step == 0:
            og.Controller.set(
                self._scan_data_attribute,
                robot.get_planar_scan(self._config.scan_samples),
            )
        if self._scan_frame_count % self._config.slam_scan_publish_step == 0:
            og.Controller.set(
                self._slam_scan_data_attribute,
                robot.get_planar_scan(self._config.slam_scan_samples),
            )
