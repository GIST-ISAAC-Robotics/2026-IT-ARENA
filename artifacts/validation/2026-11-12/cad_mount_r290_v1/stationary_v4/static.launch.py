from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    path=get_package_share_directory('arena_bringup')+'/launch/simulation.launch.py'
    return LaunchDescription([SetParameter(name='partial_observation_policy', value='bounded_local_v4'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(path))])
