"""새 RGB+C1+독립 IMU 구성의 재현 가능한 기본 실행. 기존 데모는 보존."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    defaults = {"headless": "false", "speed_profile": "local_fast", "speed_bump": "false",
                "chase_camera": "false", "render_backend": "system", "collision_detector": "configured",
                "sensor_wall_timeout_s": "3.0", "batch_static_visuals": "true",
                "lidar_test_profile": "", "motion_test_profile": "", "actuation_mode": "legacy",
                "timing_probe": "false"}
    demo = Path(get_package_share_directory("arena_bringup"))/"launch/demo.launch.py"
    return LaunchDescription([
        DeclareLaunchArgument('g1_parallel_hold_trial', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('g1_pass_release_trial', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('g1_guard_curve_trial', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('partial_observation_policy', default_value='strict',
                              choices=['strict', 'bounded_side_v1', 'bounded_local_v2', 'bounded_local_v3', 'bounded_local_v4']),
        DeclareLaunchArgument('evaluation_course_markers', default_value='visible', choices=['visible','blank_30','blank_all']),
        *[DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()],
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(demo)), launch_arguments={
            'g1_parallel_hold_trial': LaunchConfiguration('g1_parallel_hold_trial'),
            'g1_pass_release_trial': LaunchConfiguration('g1_pass_release_trial'),
            'g1_guard_curve_trial': LaunchConfiguration('g1_guard_curve_trial'),
            'partial_observation_policy': LaunchConfiguration('partial_observation_policy'),
            'evaluation_course_markers': LaunchConfiguration('evaluation_course_markers'),
            **{key: LaunchConfiguration(key) for key in defaults},
            "autonomy_mode": "local_pursuit", "lidar_acquisition": "sequential",
            "lidar_compensation": "both", "autonomy_control_rate_hz": "50.0",
            "depth_camera": "false", "lidar_rate_hz": "10.0",
        }.items())])
