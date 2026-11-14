"""다음 평가 사본에서만 쓰는 마커 OFF/ON 매개변수 연결. 기본 OFF."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from launch_ros.parameter_descriptions import ParameterValue

A=Path(__file__).resolve().parent
BASE=A/'course.launch.py'

def _marker_profile_parameters(context):
    # 빈 global parameter는 ROS CLI name:=로 직렬화되어 init 단계에서 거절된다.
    profile=LaunchConfiguration('r537_marker_profile').perform(context)
    if not profile:return []  # 소비자의 기존 빈 경로 기본값: 마커 보조 OFF
    return [SetParameter(name='vehicle_marker_profile_path',
                         value=ParameterValue(profile,value_type=str))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('r537_marker_profile',default_value=''),
        OpaqueFunction(function=_marker_profile_parameters),
        SetParameter(name='vehicle_marker_observer_enabled',value=False),
        SetParameter(name='vision_execution',value='process'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(BASE))),
    ])
