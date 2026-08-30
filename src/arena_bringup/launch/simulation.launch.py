from __future__ import annotations

import json
from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


TRACK_DIRECTORIES = {
    "original": "it_arena_track",
    "experimental": "it_arena_experimental",
}


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _launch_setup(context):
    description_share = Path(get_package_share_directory("arena_description"))
    gazebo_share = Path(get_package_share_directory("arena_gazebo"))
    config_path = Path(LaunchConfiguration("vehicle_config").perform(context))
    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)["vehicle"]

    body = config["body"]
    drivetrain = config["drivetrain"]
    d435i = config["sensors"]["d435i"]
    wheel_encoders = config["sensors"]["wheel_encoders"]
    d435i_xyz = d435i["xyz_m"]
    d435i_rpy = d435i["rpy_rad"]
    xacro_path = description_share / "models" / "arena_car" / "model.sdf.xacro"
    model_xml = xacro.process_file(
        str(xacro_path),
        mappings={
            "body_length": str(body["length_m"]),
            "body_width": str(body["width_m"]),
            "body_height": str(body["height_m"]),
            "body_mass": str(body["mass_kg"]),
            "wheelbase": str(drivetrain["wheelbase_m"]),
            "track_width": str(drivetrain["track_width_m"]),
            "wheel_radius": str(drivetrain["wheel_radius_m"]),
            "wheel_width": str(drivetrain["wheel_width_m"]),
            "max_steering_angle": str(drivetrain["max_steering_angle_rad"]),
            "max_speed": str(drivetrain["max_speed_mps"]),
            "d435i_x": str(d435i_xyz[0]),
            "d435i_y": str(d435i_xyz[1]),
            "d435i_z": str(d435i_xyz[2]),
            "d435i_roll": str(d435i_rpy[0]),
            "d435i_pitch": str(d435i_rpy[1]),
            "d435i_yaw": str(d435i_rpy[2]),
            "d435i_horizontal_fov": str(d435i["rgb_horizontal_fov_rad"]),
            "d435i_width": str(d435i["width_px"]),
            "d435i_height": str(d435i["height_px"]),
            "d435i_rate": str(d435i["update_rate_hz"]),
            "d435i_near": str(d435i["minimum_depth_m"]),
            "d435i_far": str(d435i["maximum_depth_m"]),
        },
    ).toxml()

    track = LaunchConfiguration("track").perform(context)
    if track not in TRACK_DIRECTORIES:
        raise RuntimeError(f"track must be one of {sorted(TRACK_DIRECTORIES)}")
    world_directory = gazebo_share / "worlds" / TRACK_DIRECTORIES[track]
    scene = json.loads((world_directory / "scene.json").read_text(encoding="utf-8"))
    slots = {int(slot["index"]): slot for slot in scene["starting_grid"]["slots"]}
    slot_index = int(LaunchConfiguration("grid_slot").perform(context))
    if slot_index not in slots:
        raise RuntimeError(f"grid_slot must be one of {sorted(slots)}")
    slot = slots[slot_index]
    spawn_x, spawn_y, spawn_yaw = slot["x"], slot["y"], slot["yaw_rad"]

    world_path = world_directory / "world.sdf"
    headless = _as_bool(LaunchConfiguration("headless").perform(context))
    # Keep the server and GUI as direct child processes. The upstream combined
    # launcher uses a shell wrapper, which can leave the actual Gazebo process
    # orphaned when ROS launch is interrupted.
    gazebo_server = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-s", "-v", "3", str(world_path)],
        output="screen",
    )
    gazebo_gui = ExecuteProcess(
        cmd=["gz", "sim", "-g", "-v", "3"],
        output="screen",
    )

    spawn_vehicle = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world",
            "it_arena_track",
            "-name",
            "arena_car",
            "-allow_renaming",
            "false",
            "-string",
            model_xml,
            "-x",
            str(spawn_x),
            "-y",
            str(spawn_y),
            "-z",
            "0.01",
            "-Y",
            str(spawn_yaw),
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/model/arena_car/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/model/arena_car/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/model/arena_car/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            "/model/arena_car/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/camera/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
        ],
        remappings=[
            ("/model/arena_car/cmd_vel", "/sim/cmd_vel"),
            ("/model/arena_car/odometry", "/odom"),
            ("/model/arena_car/tf", "/tf"),
            ("/model/arena_car/joint_state", "/sim/joint_states_raw"),
            ("/camera/image", "/camera/color/image_raw"),
            ("/camera/camera_info", "/camera/color/camera_info"),
            ("/camera/depth_image", "/camera/depth/image_rect_raw"),
            ("/camera/points", "/camera/depth/color/points"),
        ],
    )

    vehicle_interface = Node(
        package="arena_vehicle_interface",
        executable="ackermann_to_twist",
        output="screen",
        parameters=[
            {
                "wheelbase_m": float(drivetrain["wheelbase_m"]),
                "max_speed_mps": float(drivetrain["max_speed_mps"]),
                "max_steering_angle_rad": float(
                    drivetrain["max_steering_angle_rad"]
                ),
                "use_sim_time": True,
            }
        ],
    )

    actions = [
        LogInfo(msg=f"Track: {track}; main width: {scene['track']['width_m']} m; "
                    f"shortcut widths: {[branch['width_m'] for branch in scene['branches']]} m. "
                    "Not a confirmed official course."),
        gazebo_server,
        bridge,
        vehicle_interface,
        TimerAction(period=2.0, actions=[spawn_vehicle]),
    ]
    if not headless:
        actions.insert(2, gazebo_gui)

    if wheel_encoders["enabled"]:
        actions.append(
            Node(
                package="arena_vehicle_interface",
                executable="sim_wheel_encoder",
                output="screen",
                parameters=[
                    {
                        "left_joint_name": wheel_encoders["left_joint_name"],
                        "right_joint_name": wheel_encoders["right_joint_name"],
                        "ticks_per_revolution": int(
                            wheel_encoders["ticks_per_revolution"]
                        ),
                        "sample_rate_hz": float(wheel_encoders["sample_rate_hz"]),
                        "latency_ms": float(wheel_encoders["latency_ms"]),
                        "dropout_probability": float(
                            wheel_encoders["dropout_probability"]
                        ),
                        "use_sim_time": True,
                    }
                ],
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    description_share = Path(get_package_share_directory("arena_description"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "track",
                default_value="experimental",
                choices=list(TRACK_DIRECTORIES),
                description="experimental: 45/25 cm test track; original: preserved 35/12 cm output.",
            ),
            DeclareLaunchArgument(
                "vehicle_config",
                default_value=str(description_share / "config" / "vehicle.yaml"),
                description="Vehicle parameter YAML file.",
            ),
            DeclareLaunchArgument(
                "grid_slot",
                default_value="0",
                description="Starting grid slot index (0-5).",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run the Gazebo server without its GUI.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
