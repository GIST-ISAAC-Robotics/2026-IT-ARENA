from __future__ import annotations

import json
import math
from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


TRACK_DIRECTORIES = {
    "official": "it_arena_official",
    "original": "it_arena_track",
    "experimental": "it_arena_experimental",
}
D435I_STREAM_PROFILES = ("high_speed_async", "synchronized_60", "low_load_30")
TOF_PROFILES = ("low_latency_4x4_60", "tracking_8x8_15")
TOF_MODULE_NAMES = {"front", "front_left", "rear_left", "rear", "rear_right", "front_right"}
DIFFERENTIAL_PROFILES = ("ideal_open", "lossy_open", "viscous_lsd")
SPEED_PROFILES = {
    "cautious": {"max_speed_mps": .35, "min_speed_mps": .14, "acceleration_mps2": .5},
    "brisk": {"max_speed_mps": .70, "min_speed_mps": .18, "acceleration_mps2": .8},
    "exploratory": {"max_speed_mps": 1.40, "min_speed_mps": .20, "acceleration_mps2": 1.0},
    "hardware_target": {"max_speed_mps": 5.5555555556, "min_speed_mps": .20, "acceleration_mps2": 1.5},
}


def _dynamics_mappings(config, drive_mode="configured", differential="configured"):
    drive = config["drivetrain"]
    mode = drive["simulation_drive_mode"] if drive_mode == "configured" else drive_mode
    if mode not in {"single_motor", "legacy_velocity"}:
        raise ValueError("알 수 없는 구동 모드입니다.")
    diff = drive["mechanical_differential"]
    profile = diff["active_profile"] if differential == "configured" else differential
    if profile not in diff["profiles"]:
        raise ValueError("알 수 없는 차동 프로필입니다.")
    selected, motor, servo, tire = diff["profiles"][profile], drive["motor"], drive["steering_servo"], drive["tire_contact"]
    values = {
        "body_cg_z_offset": config["body"].get("center_of_mass_z_offset_m", 0),
        "gear_ratio": motor["gear_ratio"], "gear_efficiency": selected["gear_efficiency"],
        "motor_torque_limit": motor["torque_limit_nm"], "motor_brake_torque_limit": motor["brake_torque_limit_nm"],
        "motor_free_speed": motor["free_speed_rad_s"], "motor_response_time": motor["response_time_s"],
        "motor_speed_kp": motor["speed_kp"], "motor_speed_ki": motor["speed_ki"],
        "motor_acceleration": motor["acceleration_limit_mps2"], "motor_timeout": motor["command_timeout_s"],
        "carrier_drag": selected["carrier_drag_nm_per_rad_s"],
        "differential_viscosity": selected["coupling_nm_per_rad_s"],
        "differential_torque_limit": selected["coupling_limit_nm"],
        "servo_position_gain": servo["position_gain"], "servo_velocity_gain": servo["velocity_gain"],
        "servo_integral_gain": servo["integral_gain"],
        "servo_torque_limit": servo["torque_limit_nm"], "servo_rate_limit": servo["rate_limit_rad_s"],
        "tire_mu_lateral": tire["friction_lateral"], "tire_mu_longitudinal": tire["friction_longitudinal"],
        "tire_slip_lateral": tire["slip_compliance_lateral"],
        "tire_slip_longitudinal": tire["slip_compliance_longitudinal"],
        "tire_normal_force": tire["nominal_wheel_normal_force_n"],
        "wheel_damping": tire["joint_viscous_damping_nm_per_rad_s"],
        "rear_left_friction_scale": tire.get("rear_left_friction_scale", 1.0),
    }
    for name, value in values.items():
        if not math.isfinite(float(value)) or (name != "body_cg_z_offset" and float(value) < 0):
            raise ValueError(f"잘못된 구동 매개변수: {name}")
    if not 0 < float(values["gear_efficiency"]) <= 1:
        raise ValueError("기어 효율은 (0, 1] 범위여야 합니다.")
    return {"drive_mode": mode, **{key: str(value) for key, value in values.items()}}


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _nominal_intrinsics(stream: dict) -> dict[str, float]:
    width = int(stream["width_px"])
    height = int(stream["height_px"])
    horizontal_fov = math.radians(float(stream["horizontal_fov_deg"]))
    vertical_fov = math.radians(float(stream["vertical_fov_deg"]))
    if width <= 0 or height <= 0 or not 0 < horizontal_fov < math.pi or not 0 < vertical_fov < math.pi:
        raise RuntimeError(f"Invalid D435i nominal stream geometry: {stream}")
    return {
        "horizontal_fov_rad": horizontal_fov,
        "fx_px": (width / 2.0) / math.tan(horizontal_fov / 2.0),
        "fy_px": (height / 2.0) / math.tan(vertical_fov / 2.0),
        # 실제 principal point와 RGB-깊이 외부 파라미터는 카메라 개체별 보정값으로 교체합니다.
        "cx_px": width / 2.0,
        "cy_px": height / 2.0,
    }


def _resolve_d435i_profile(config: dict, requested: str) -> tuple[str, dict]:
    name = config["active_stream_profile"] if requested == "configured" else requested
    if name not in config["stream_profiles"]:
        raise RuntimeError(f"d435i_profile must be one of {sorted(config['stream_profiles'])}")
    profile = config["stream_profiles"][name]
    color_rate = float(profile["color_rate_hz"])
    depth_rate = float(profile["depth_rate_hz"])
    if not math.isfinite(color_rate) or not math.isfinite(depth_rate) or min(color_rate, depth_rate) <= 0:
        raise RuntimeError("D435i stream rates must be finite and positive")
    if profile["hardware_sync_compatible"] and color_rate != depth_rate:
        raise RuntimeError("A hardware-sync-compatible D435i profile must use equal RGB and depth rates")
    return name, profile


def _resolve_tof_profile(config: dict, requested: str) -> tuple[str, dict]:
    name = config["active_profile"] if requested == "configured" else requested
    if name not in config["profiles"]:
        raise RuntimeError(f"tof_profile must be one of {sorted(config['profiles'])}")
    profile = config["profiles"][name]
    horizontal = profile["horizontal_zones"]
    vertical = profile["vertical_zones"]
    rate = float(profile["update_rate_hz"])
    if horizontal not in {4, 8} or vertical not in {4, 8}:
        raise RuntimeError("VL53L7CX ToF profile zones must be 4 or 8")
    if horizontal != vertical:
        raise RuntimeError("VL53L7CX ToF simulation requires a square zone grid")
    maximum_rate = 60.0 if horizontal == 4 else 15.0
    if not math.isfinite(rate) or not 0 < rate <= maximum_rate:
        raise RuntimeError(
            f"VL53L7CX {horizontal}x{vertical} profile rate must be in (0, {maximum_rate}] Hz"
        )
    return name, profile


def _validate_tof_ring(config: dict, body: dict) -> None:
    modules = config["modules"]
    if len(modules) != 6 or {module["name"] for module in modules} != TOF_MODULE_NAMES:
        raise RuntimeError(f"ToF ring requires these six module names: {sorted(TOF_MODULE_NAMES)}")
    for key in ("frame_id", "topic"):
        if len({module[key] for module in modules}) != len(modules):
            raise RuntimeError(f"ToF modules require unique {key} values")
    for key in ("horizontal_fov_deg", "vertical_fov_deg"):
        value = float(config[key])
        if not math.isfinite(value) or not 0 < value < 180:
            raise RuntimeError(f"Invalid ToF {key}: {value}")
    minimum, maximum = float(config["range_min_m"]), float(config["range_max_m"])
    if not math.isfinite(minimum) or not math.isfinite(maximum) or not 0 < minimum < maximum:
        raise RuntimeError("Invalid ToF range limits")
    half_length = float(body["length_m"]) / 2.0
    half_width = float(body["width_m"]) / 2.0
    yaws = []
    for module in modules:
        xyz = [float(value) for value in module["xyz_m"]]
        rpy = [float(value) for value in module["rpy_rad"]]
        if len(xyz) != 3 or len(rpy) != 3 or not all(math.isfinite(value) for value in [*xyz, *rpy]):
            raise RuntimeError(f"Invalid ToF module pose: {module}")
        if abs(xyz[0]) > half_length or abs(xyz[1]) > half_width or xyz[2] <= 0:
            raise RuntimeError(f"ToF optical center is outside the vehicle envelope: {module['name']}")
        yaws.append(rpy[2] % (2.0 * math.pi))
    yaws.sort()
    gaps = [
        (yaws[(index + 1) % len(yaws)] - yaws[index]) % (2.0 * math.pi)
        for index in range(len(yaws))
    ]
    horizontal_fov = math.radians(float(config["horizontal_fov_deg"]))
    if max(gaps) > horizontal_fov + 1e-6:
        raise RuntimeError("Nominal ToF ring has an angular coverage gap")


def _launch_setup(context):
    bringup_share = Path(get_package_share_directory("arena_bringup"))
    description_share = Path(get_package_share_directory("arena_description"))
    gazebo_share = Path(get_package_share_directory("arena_gazebo"))
    config_path = Path(LaunchConfiguration("vehicle_config").perform(context))
    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)["vehicle"]

    body = config["body"]
    drivetrain = config["drivetrain"]
    rear_marker = config["identification_marker"]
    rear_marker_xyz = [float(value) for value in rear_marker["xyz_m"]]
    rear_marker_rpy = [float(value) for value in rear_marker["rpy_rad"]]
    if (
        rear_marker["dictionary"] != "DICT_4X4_50"
        or int(rear_marker["id"]) != 10
        or not math.isclose(float(rear_marker["printed_board_size_m"]), .05, abs_tol=1e-9)
        or not math.isclose(
            float(rear_marker["black_code_size_m"])
            + 2 * float(rear_marker["quiet_zone_each_side_m"]),
            float(rear_marker["printed_board_size_m"]),
            abs_tol=1e-9,
        )
        or any(not math.isfinite(value) for value in [*rear_marker_xyz, *rear_marker_rpy])
        or not math.isclose(rear_marker_rpy[0], 0.0, abs_tol=1e-9)
        or not math.isclose(rear_marker_rpy[1], 0.0, abs_tol=1e-9)
    ):
        raise RuntimeError("현재 차량 형상은 5 cm DICT_4X4_50 ID 10 후면 마커 설정만 지원합니다.")
    d435i = config["sensors"]["d435i"]
    wheel_encoders = config["sensors"]["wheel_encoders"]
    drive_mappings = _dynamics_mappings(config, LaunchConfiguration("drive_mode").perform(context),
                                       LaunchConfiguration("differential_profile").perform(context))
    safety_enabled = _as_bool(LaunchConfiguration("tof_safety").perform(context))
    render_sensors = _as_bool(LaunchConfiguration("render_sensors").perform(context))
    if safety_enabled and not render_sensors:
        raise RuntimeError("광학 센서를 끈 동역학 전용 시험에서는 tof_safety:=false를 명시해야 합니다.")
    lidar = config["sensors"]["lidar_2d"]
    tof = config["sensors"]["tof_ring"]
    if lidar["enabled"]:
        if int(lidar["samples_per_scan"]) < 10 or not math.isclose(float(lidar["field_of_view_deg"]), 360.0):
            raise RuntimeError("기초 C1 모델은 360도 스캔과 10개 이상의 표본을 요구합니다.")
        if not 0 < float(lidar["range_min_m"]) < float(lidar["range_max_m"]) or float(lidar["scan_rate_hz"]) <= 0:
            raise RuntimeError("라이다 거리·주기 설정이 올바르지 않습니다.")
    d435i_xyz = d435i["xyz_m"]
    d435i_rpy = d435i["rpy_rad"]
    d435i_profile_name, d435i_profile = _resolve_d435i_profile(
        d435i, LaunchConfiguration("d435i_profile").perform(context)
    )
    tof_profile_name, tof_profile = _resolve_tof_profile(
        tof, LaunchConfiguration("tof_profile").perform(context)
    )
    if tof["enabled"]:
        _validate_tof_ring(tof, body)
    color = d435i["color"]
    depth = d435i["depth"]
    depth_enabled = _as_bool(LaunchConfiguration("depth_camera").perform(context))
    color_intrinsics = _nominal_intrinsics(color)
    depth_intrinsics = _nominal_intrinsics(depth)
    xacro_path = description_share / "models" / "arena_car" / "model.sdf.xacro"
    model_xml = xacro.process_file(
        str(xacro_path),
        mappings={
            **drive_mappings,
            "render_sensors": str(render_sensors).lower(),
            "body_length": str(body["length_m"]),
            "body_width": str(body["width_m"]),
            "body_height": str(body["height_m"]),
            "body_mass": str(body["mass_kg"]),
            "rear_marker_enabled": str(rear_marker["enabled"]).lower(),
            "rear_marker_x": str(rear_marker_xyz[0]),
            "rear_marker_y": str(rear_marker_xyz[1]),
            "rear_marker_z": str(rear_marker_xyz[2]),
            "rear_marker_yaw": str(rear_marker_rpy[2]),
            "rear_marker_board_size": str(rear_marker["printed_board_size_m"]),
            "rear_marker_code_size": str(rear_marker["black_code_size_m"]),
            "wheelbase": str(drivetrain["wheelbase_m"]),
            "track_width": str(drivetrain["track_width_m"]),
            "wheel_radius": str(drivetrain["wheel_radius_m"]),
            "wheel_width": str(drivetrain["wheel_width_m"]),
            "max_steering_angle": str(drivetrain["max_steering_angle_rad"]),
            "max_speed": str(drivetrain["max_speed_mps"]),
            "steering_p_gain": str(drivetrain.get("simulation_steering_p_gain", 12.0)),
            "acceleration_limit": str(drivetrain.get("simulation_acceleration_limit_mps2", 4.0)),
            "lidar_enabled": str(lidar["enabled"]).lower(),
            **{f"lidar_{axis}": str(value) for axis, value in zip(("x", "y", "z"), lidar["xyz_m"])},
            **{f"lidar_{axis}": str(value) for axis, value in zip(("roll", "pitch", "yaw"), lidar["rpy_rad"])},
            "lidar_rate": str(lidar["scan_rate_hz"]),
            "lidar_samples": str(lidar["samples_per_scan"]),
            "lidar_min": str(lidar["range_min_m"]),
            "lidar_max": str(lidar["range_max_m"]),
            "lidar_resolution": str(lidar["nominal_range_resolution_m"]),
            "lidar_noise": str(lidar["simulated_noise_stddev_m"]),
            "lidar_mass": str(lidar["mass_kg"]),
            "lidar_radius": str(lidar["proxy_radius_m"]),
            "lidar_height": str(lidar["proxy_height_m"]),
            "tof_enabled": str(tof["enabled"]).lower(),
            "tof_rate": str(tof_profile["update_rate_hz"]),
            "tof_horizontal_zones": str(tof_profile["horizontal_zones"]),
            "tof_vertical_zones": str(tof_profile["vertical_zones"]),
            "tof_horizontal_fov": str(math.radians(float(tof["horizontal_fov_deg"]))),
            "tof_vertical_fov": str(math.radians(float(tof["vertical_fov_deg"]))),
            "tof_min": str(tof["range_min_m"]),
            "tof_max": str(tof["range_max_m"]),
            "tof_resolution": str(tof["nominal_depth_resolution_m"]),
            "tof_noise": str(tof["simulated_noise_stddev_m"]),
            "tof_carrier_size_x": str(tof["carrier_size_m"][0]),
            "tof_carrier_size_y": str(tof["carrier_size_m"][1]),
            "tof_carrier_size_z": str(tof["carrier_size_m"][2]),
            "tof_carrier_mass": str(tof["carrier_mass_kg"]),
            **{
                f"tof_{module['name']}_pose": " ".join(
                    str(value) for value in [*module["xyz_m"], *module["rpy_rad"]]
                )
                for module in tof["modules"]
            },
            **{f"tof_{module['name']}_topic": str(module["topic"]) for module in tof["modules"]},
            "d435i_x": str(d435i_xyz[0]),
            "d435i_y": str(d435i_xyz[1]),
            "d435i_z": str(d435i_xyz[2]),
            "d435i_roll": str(d435i_rpy[0]),
            "d435i_pitch": str(d435i_rpy[1]),
            "d435i_yaw": str(d435i_rpy[2]),
            "d435i_color_horizontal_fov": str(color_intrinsics["horizontal_fov_rad"]),
            "d435i_color_width": str(color["width_px"]),
            "d435i_color_height": str(color["height_px"]),
            "d435i_color_rate": str(d435i_profile["color_rate_hz"]),
            "d435i_color_near": str(color["render_near_clip_m"]),
            "d435i_color_far": str(color["render_far_clip_m"]),
            "d435i_color_fx": str(color_intrinsics["fx_px"]),
            "d435i_color_fy": str(color_intrinsics["fy_px"]),
            "d435i_color_cx": str(color_intrinsics["cx_px"]),
            "d435i_color_cy": str(color_intrinsics["cy_px"]),
            "d435i_depth_horizontal_fov": str(depth_intrinsics["horizontal_fov_rad"]),
            "d435i_depth_enabled": str(depth_enabled).lower(),
            "d435i_depth_width": str(depth["width_px"]),
            "d435i_depth_height": str(depth["height_px"]),
            "d435i_depth_rate": str(d435i_profile["depth_rate_hz"]),
            "d435i_depth_near": str(depth["minimum_depth_m"]),
            "d435i_depth_far": str(depth["simulation_far_clip_m"]),
            "d435i_depth_fx": str(depth_intrinsics["fx_px"]),
            "d435i_depth_fy": str(depth_intrinsics["fy_px"]),
            "d435i_depth_cx": str(depth_intrinsics["cx_px"]),
            "d435i_depth_cy": str(depth_intrinsics["cy_px"]),
            "d435i_imu_rate": str(d435i["imu"]["update_rate_hz"]),
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
    world_override = LaunchConfiguration("world_override").perform(context)
    if world_override:
        world_path = Path(world_override).resolve()
        if not world_path.is_file():
            raise RuntimeError(f"시험 월드를 찾을 수 없습니다: {world_path}")
        spawn_x = float(LaunchConfiguration("test_spawn_x").perform(context))
        spawn_y = float(LaunchConfiguration("test_spawn_y").perform(context))
        spawn_yaw = float(LaunchConfiguration("test_spawn_yaw").perform(context))
        if not all(math.isfinite(v) for v in (spawn_x, spawn_y, spawn_yaw)):
            raise ValueError("시험 시작 위치는 유한한 수여야 합니다.")
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
            "/model/arena_car/steer_angle@std_msgs/msg/Float64]gz.msgs.Double",
            "/model/arena_car/drivetrain@std_msgs/msg/String[gz.msgs.StringMsg",
            "/model/arena_car/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/model/arena_car/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            "/model/arena_car/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model",
        ],
        remappings=[
            ("/model/arena_car/cmd_vel", "/sim/cmd_vel"),
            ("/model/arena_car/steer_angle", "/sim/steering_angle"),
            ("/model/arena_car/drivetrain", "/sim/drivetrain"),
            ("/model/arena_car/odometry", "/odom"),
            ("/model/arena_car/tf", "/tf"),
            ("/model/arena_car/joint_state", "/sim/joint_states_raw"),
        ],
    )

    sensor_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="d435i_sensor_bridge",
        output="screen",
        parameters=[{"config_file": str(bringup_share / "config" /
                                        ("d435i_sensor_bridge.yaml" if depth_enabled else "d435i_rgb_imu_bridge.yaml"))}],
    )

    # 대용량 포인트 클라우드가 RGB/깊이 영상 브리지를 지연시키지 않게 분리하고,
    # 실제 ROS 구독자가 있을 때만 ROS 변환·전송합니다. 토픽 이름은 유지합니다.
    pointcloud_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="d435i_pointcloud_bridge",
        output="screen",
        parameters=[{"config_file": str(bringup_share / "config" / "d435i_pointcloud_bridge.yaml")}],
    )

    vehicle_interface = Node(
        package="arena_vehicle_interface",
        executable="ackermann_to_twist",
        output="screen",
        remappings=[("/drive", "/drive/safe")] if safety_enabled else [],
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
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", [
            str(Path(get_package_prefix("arena_gazebo")) / "lib"), ":",
            EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value="")]),
        LogInfo(msg=f"Drive: {drive_mappings['drive_mode']}; differential: "
                    f"{LaunchConfiguration('differential_profile').perform(context)}; "
                    f"ToF safety={safety_enabled}. Motor/tire properties are provisional."),
        LogInfo(msg=f"Track: {track}; main width: {scene['track']['width_m']} m; "
                    f"shortcut widths: {[branch['width_m'] for branch in scene['branches']]} m. "
                    + ("Official v2026.09.02 road/grass/wall geometry and filled grid visuals are preserved; "
                     "course-marker angle, signal, bump profile and checker finish are team-test provisional."
                     if track == "official" else "Historical reproduction / experimental course.")),
        LogInfo(msg=f"Rear identification marker: enabled={rear_marker['enabled']}; "
                    f"{rear_marker['dictionary']} ID {rear_marker['id']}; "
                    f"board={rear_marker['printed_board_size_m']} m (team provisional)."),
        LogInfo(msg=f"D435i profile: {d435i_profile_name}; "
                    f"RGB {color['width_px']}x{color['height_px']} @ "
                    f"{d435i_profile['color_rate_hz']} Hz; depth "
                    f"{depth['width_px']}x{depth['height_px']} @ "
                    f"{d435i_profile['depth_rate_hz']} Hz (enabled={depth_enabled})."),
        LogInfo(msg=f"ToF ring: enabled={tof['enabled']}; profile={tof_profile_name}; "
                    f"{tof_profile['horizontal_zones']}x{tof_profile['vertical_zones']} @ "
                    f"{tof_profile['update_rate_hz']} Hz; modules={len(tof['modules'])}."),
        gazebo_server,
        bridge,
        sensor_bridge,
        vehicle_interface,
        TimerAction(period=2.0, actions=[spawn_vehicle]),
    ]
    if not render_sensors:
        actions.remove(sensor_bridge)
    if depth_enabled and render_sensors:
        actions.append(pointcloud_bridge)
    if not headless:
        actions.append(gazebo_gui)

    if safety_enabled:
        if not tof["enabled"] or not wheel_encoders["enabled"]:
            raise RuntimeError("ToF 안전층은 ToF 링과 좌우 바퀴 엔코더가 필요합니다.")
        actions.append(Node(package="arena_autonomy", executable="tof_safety", output="screen",
                            parameters=[str(bringup_share / "config/tof_safety.yaml"),
                                        {"use_sim_time": True, "vehicle_config": str(config_path)}]))

    if lidar["enabled"] and render_sensors:
        actions.extend([
            Node(package="ros_gz_bridge", executable="parameter_bridge", name="lidar_bridge",
                 arguments=["/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
                 parameters=[{"override_frame_id": str(lidar["frame_id"])}], output="screen"),
            Node(package="tf2_ros", executable="static_transform_publisher", name="lidar_transform",
                 arguments=["--x", str(lidar["xyz_m"][0]), "--y", str(lidar["xyz_m"][1]),
                            "--z", str(lidar["xyz_m"][2]), "--roll", str(lidar["rpy_rad"][0]),
                            "--pitch", str(lidar["rpy_rad"][1]), "--yaw", str(lidar["rpy_rad"][2]),
                            "--frame-id", "base_link", "--child-frame-id", str(lidar["frame_id"])],
                 parameters=[{"use_sim_time": True}], output="screen"),
        ])

    if tof["enabled"] and render_sensors:
        for module in tof["modules"]:
            name = str(module["name"])
            topic = str(module["topic"])
            xyz = module["xyz_m"]
            rpy = module["rpy_rad"]
            actions.extend([
                Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    name=f"tof_{name}_pointcloud_bridge",
                    arguments=[
                        f"{topic}/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
                    ],
                    parameters=[{"override_frame_id": str(module["frame_id"])}],
                    output="screen",
                ),
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name=f"tof_{name}_transform",
                    arguments=[
                        "--x", str(xyz[0]), "--y", str(xyz[1]), "--z", str(xyz[2]),
                        "--roll", str(rpy[0]), "--pitch", str(rpy[1]), "--yaw", str(rpy[2]),
                        "--frame-id", "base_link", "--child-frame-id", str(module["frame_id"]),
                    ],
                    parameters=[{"use_sim_time": True}],
                    output="screen",
                ),
            ])

    autonomy = _as_bool(LaunchConfiguration("autonomy").perform(context))
    traffic = _as_bool(LaunchConfiguration("traffic_light").perform(context))
    if (autonomy or traffic) and (world_override or not render_sensors):
        raise RuntimeError("시험 월드/광학 센서 비활성화는 본선 자율주행·신호등과 함께 쓰지 않습니다.")
    if (autonomy or traffic) and track not in {"official", "experimental"}:
        raise RuntimeError("신호등·본선 벽 추종 데모는 official/experimental 지도에서만 지원합니다.")
    if autonomy and not lidar["enabled"]:
        raise RuntimeError("벽 추종에는 lidar_2d.enabled=true가 필요합니다.")
    if traffic:
        actions.append(Node(package="arena_gazebo", executable="traffic_light_controller.py",
                            parameters=[{"use_sim_time": True,
                                         "red_duration_s": float(LaunchConfiguration("red_duration_s").perform(context)),
                                         "yellow_duration_s": 2.0}], output="screen"))
    if autonomy:
        speed_profile = SPEED_PROFILES[LaunchConfiguration("speed_profile").perform(context)]
        actions.append(Node(package="arena_autonomy", executable="wall_follow",
                            parameters=[str(bringup_share / "config" / "wall_follow.yaml"),
                                        {"use_sim_time": True, **speed_profile, "wheelbase_m": float(drivetrain["wheelbase_m"]),
                                         "lidar_x_m": float(lidar["xyz_m"][0]),
                                         "max_steering_angle_rad": float(drivetrain["max_steering_angle_rad"])}],
                            output="screen"))

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
            # 이번 launch의 ROS 하위 프로세스에만 적용합니다. 별도 수신 터미널도
            # 같은 모드를 사용해야 하며, 기존 사용자 지정 환경변수는 덮어쓰지 않습니다.
            SetEnvironmentVariable(
                "FASTDDS_BUILTIN_TRANSPORTS",
                EnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", default_value="LARGE_DATA"),
            ),
            DeclareLaunchArgument(
                "track",
                default_value="official",
                choices=list(TRACK_DIRECTORIES),
                description="official: v2026.09.02 45/20 cm team-test runtime; experimental: legacy 45/25 cm test; original: preserved 35/12 cm.",
            ),
            DeclareLaunchArgument(
                "vehicle_config",
                default_value=str(description_share / "config" / "vehicle.yaml"),
                description="Vehicle parameter YAML file.",
            ),
            DeclareLaunchArgument(
                "d435i_profile",
                default_value="configured",
                choices=["configured", *D435I_STREAM_PROFILES],
                description="D435i stream profile. configured follows active_stream_profile in vehicle_config.",
            ),
            DeclareLaunchArgument(
                "tof_profile",
                default_value="configured",
                choices=["configured", *TOF_PROFILES],
                description="ToF zone/rate profile. configured follows active_profile in vehicle_config.",
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
            DeclareLaunchArgument("autonomy", default_value="false", description="RGB 출발 신호 + 라이다 본선 벽 추종"),
            DeclareLaunchArgument("render_sensors", default_value="true", description="동역학 전용 시험에서 광학 센서만 끕니다. 링크·질량은 유지합니다."),
            DeclareLaunchArgument("world_override", default_value="", description="시험 전용 SDF. 원본/실험 트랙 파일은 덮어쓰지 않습니다."),
            DeclareLaunchArgument("test_spawn_x", default_value="0"),
            DeclareLaunchArgument("test_spawn_y", default_value="0"),
            DeclareLaunchArgument("test_spawn_yaw", default_value="0"),
            DeclareLaunchArgument("drive_mode", default_value="configured", choices=["configured", "single_motor", "legacy_velocity"]),
            DeclareLaunchArgument("differential_profile", default_value="configured", choices=["configured", *DIFFERENTIAL_PROFILES]),
            DeclareLaunchArgument("tof_safety", default_value="true", description="/drive를 ToF 안전층에서 제한하여 /drive/safe로 전달"),
            DeclareLaunchArgument("speed_profile", default_value="cautious", choices=list(SPEED_PROFILES),
                                  description="명령 속도 상한이며 실제 속도/완주 보증이 아닙니다. ToF 안전층이 추가 제한합니다."),
            DeclareLaunchArgument("depth_camera", default_value="true", description="깊이 센서와 점군을 켭니다. RGB·IMU는 유지합니다."),
            DeclareLaunchArgument("traffic_light", default_value="false", description="빨강-노랑-초록 출발 신호와 수동 제어"),
            DeclareLaunchArgument("red_duration_s", default_value="8.0", description="영상 준비 이후 빨간 신호 유지 시간"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
