"""다음 평가 사본에서만 쓰는 마커 OFF/ON 매개변수 연결. 기본 OFF."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from launch_ros.parameter_descriptions import ParameterValue

A=Path(__file__).resolve().parent
BASE=A.parent/'field_wall_resource_drive_r530_v1/course.launch.py'

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('r537_marker_profile',default_value=''),
        SetParameter(name='vehicle_marker_profile_path',
                     value=ParameterValue(LaunchConfiguration('r537_marker_profile'),value_type=str)),
        SetParameter(name='vehicle_marker_observer_enabled',value=False),
        SetParameter(name='vision_execution',value='process'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(BASE))),
    ])
