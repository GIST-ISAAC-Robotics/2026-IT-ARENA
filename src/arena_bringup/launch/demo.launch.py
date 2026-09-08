"""빨간 신호 대기 후 본선을 계속 도는 단일 차량 기초 데모."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include_simulation(context):
    launch_file = Path(get_package_share_directory("arena_bringup")) / "launch/simulation.launch.py"
    mode = LaunchConfiguration("autonomy_mode").perform(context)
    depth = LaunchConfiguration("depth_camera").perform(context)
    if depth == "auto":
        depth = "true" if mode == "stereo" else "false"
    if mode == "stereo" and depth != "true":
        raise RuntimeError("stereo 모드는 depth_camera:=true가 필요합니다.")
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_file)), launch_arguments={
        "headless": LaunchConfiguration("headless"), "grid_slot": LaunchConfiguration("grid_slot"),
        "render_backend": LaunchConfiguration("render_backend"),
        "collision_detector": LaunchConfiguration("collision_detector"),
        "scene_broadcaster": LaunchConfiguration("scene_broadcaster"),
        "red_duration_s": LaunchConfiguration("red_duration_s"), "track": "official",
        "depth_camera": depth,
        "tof_profile": LaunchConfiguration("tof_profile"),
        "lidar_rate_hz": LaunchConfiguration("lidar_rate_hz"),
        "lidar_acquisition": LaunchConfiguration("lidar_acquisition"),
        "lidar_compensation": LaunchConfiguration("lidar_compensation"),
        "autonomy_control_rate_hz": LaunchConfiguration("autonomy_control_rate_hz"),
        "sensor_wall_timeout_s": LaunchConfiguration("sensor_wall_timeout_s"),
        "speed_profile": LaunchConfiguration("speed_profile"),
        "differential_profile": LaunchConfiguration("differential_profile"),
        "tof_safety": LaunchConfiguration("tof_safety"),
        "drive_mode": LaunchConfiguration("drive_mode"),
        "autonomy_mode": mode,
        "chase_camera": LaunchConfiguration("chase_camera"),
        "d435i_profile": "low_load_30", "autonomy": "true", "traffic_light": "true",
    }.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("render_backend", default_value="system", choices=["auto", "system", "wsl_nvidia", "software"]),
        DeclareLaunchArgument("collision_detector", default_value="configured", choices=["configured", "ode", "bullet", "fcl"]),
        DeclareLaunchArgument("scene_broadcaster", default_value="true"),
        DeclareLaunchArgument("grid_slot", default_value="0"),
        DeclareLaunchArgument("red_duration_s", default_value="8.0"),
        DeclareLaunchArgument("autonomy_mode", default_value="lidar", choices=["lidar", "stereo"],
                              description="lidar=C1+ToF6, stereo=전방 D435i 깊이+측면 ToF4"),
        DeclareLaunchArgument("depth_camera", default_value="auto", choices=["auto", "true", "false"],
                              description="auto는 stereo에서 켜고 lidar에서 끕니다."),
        DeclareLaunchArgument("chase_camera", default_value="false", description="검증 영상 전용 3인칭 카메라"),
        DeclareLaunchArgument("tof_profile", default_value="configured", description="하부 ToF 영역·주기 프로필"),
        DeclareLaunchArgument("lidar_rate_hz", default_value="configured", description="라이다 주기 비교용 실행 덮어쓰기"),
        DeclareLaunchArgument("lidar_acquisition", default_value="snapshot", choices=["snapshot", "sequential"]),
        DeclareLaunchArgument("lidar_compensation", default_value="none", choices=["none", "deskew", "shift", "both"]),
        DeclareLaunchArgument("autonomy_control_rate_hz", default_value="20.0", description="단독 고속 시험용 제어 주기; 기본 20 Hz 유지"),
        DeclareLaunchArgument("sensor_wall_timeout_s", default_value="3.0", description="느린 오프라인 시뮬레이션용 벽시계 감시 한도; 센서 취득 시각 제한은 유지"),
        DeclareLaunchArgument("speed_profile", default_value="cautious"),
        DeclareLaunchArgument("differential_profile", default_value="configured"),
        DeclareLaunchArgument("tof_safety", default_value="true"),
        DeclareLaunchArgument("drive_mode", default_value="configured"),
        OpaqueFunction(function=_include_simulation),
    ])
