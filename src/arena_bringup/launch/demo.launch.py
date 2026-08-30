"""빨간 신호 대기 후 본선을 계속 도는 단일 차량 기초 데모."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_file = Path(get_package_share_directory("arena_bringup")) / "launch/simulation.launch.py"
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("grid_slot", default_value="0"),
        DeclareLaunchArgument("red_duration_s", default_value="8.0"),
        DeclareLaunchArgument("depth_camera", default_value="false", description="이 데모는 깊이를 사용하지 않으므로 기본값은 끕니다."),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_file)), launch_arguments={
            "headless": LaunchConfiguration("headless"), "grid_slot": LaunchConfiguration("grid_slot"),
            "red_duration_s": LaunchConfiguration("red_duration_s"), "track": "experimental",
            "depth_camera": LaunchConfiguration("depth_camera"),
            "d435i_profile": "low_load_30", "autonomy": "true", "traffic_light": "true",
        }.items()),
    ])
