from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter
def generate_launch_description():
    return LaunchDescription([SetParameter(name='g1_follow_enabled',value=True),
        SetParameter(name='g2_entry_simulation_trial',value=True),
        SetParameter(name='g2_trace_path',value='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-10-29/g2_entry_left_v1/run1/g2_decisions.jsonl'),
        SetParameter(name='start_signal_mode',value='official_4red_local_candidate'),
        SetParameter(name='race_control_mode',value='legacy'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource('/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-10-29/g2_entry_left_v1/fixture/simulation.launch.py'),launch_arguments={'track': 'official', 'grid_slot': '3', 'headless': 'true', 'autonomy': 'true', 'traffic_light': 'false', 'autonomy_mode': 'local_pursuit', 'lidar_acquisition': 'sequential', 'lidar_compensation': 'both', 'lidar_rate_hz': '10.0', 'autonomy_control_rate_hz': '50.0', 'depth_camera': 'false', 'speed_bump': 'false', 'chase_camera': 'false', 'batch_static_visuals': 'true', 'scene_broadcaster': 'true', 'speed_profile': 'g1_compare_1mps', 'render_backend': 'wsl_nvidia', 'sensor_wall_timeout_s': '30', 'actuation_mode': 'virtual_mcu', 'runtime_artifacts': '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-10-29/g2_entry_left_v1/run1', 'd435i_profile': 'low_load_30', 'evaluation_actor_sdf': '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-10-29/g2_entry_left_v1/run1/evaluation_actor.sdf'}.items()),
        ExecuteProcess(cmd=['/usr/bin/python3', '-B', '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-10-27/official_departure_v2/signal_driver_v2.py', '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-10-29/g2_entry_left_v1/run1'],output="screen")])
