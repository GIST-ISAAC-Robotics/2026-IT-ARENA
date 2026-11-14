"""명시 선택형 R0+ 비교. 단독/G1 기본값과 독립 보호층은 바꾸지 않는다."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter


def _trace_path_parameters(context):
    # 빈 global parameter를 name:=로 보내지 않고 소비자의 기록 OFF 기본값을 쓴다.
    if not LaunchConfiguration('g1_trace_path').perform(context):return []
    return [SetParameter(name='g1_trace_path', value=LaunchConfiguration('g1_trace_path'))]


def generate_launch_description():
    source = Path(get_package_share_directory('arena_bringup'))/'launch/local_pursuit.launch.py'
    return LaunchDescription([GroupAction(scoped=False, actions=[
        DeclareLaunchArgument('g1_trace_path', default_value=''),
        OpaqueFunction(function=_trace_path_parameters),
        SetParameter(name='r0_plus_enabled', value=True),
        SetParameter(name='g1_follow_enabled', value=False),
        SetParameter(name='g1_boundary_evidence_mode', value='connected'),
        SetParameter(name='g1_occupancy_timing_mode', value='per_ray_candidate'),
        SetParameter(name='g1_corridor_fallback_enabled', value=False),
        SetParameter(name='g1_recent_observation_enabled', value=False),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(source)))])])
