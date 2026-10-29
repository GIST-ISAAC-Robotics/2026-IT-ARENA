from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter
def generate_launch_description():
    return LaunchDescription([SetParameter(name='g1_follow_enabled',value=True),
        SetParameter(name='g1_boundary_evidence_mode',value='corner_distance_candidate'),
        SetParameter(name='g1_occupancy_timing_mode',value='per_ray_candidate'),
        SetParameter(name='g1_corridor_fallback_enabled',value=True),
        SetParameter(name='g1_short_boundary_prefix_enabled',value=True),
        SetParameter(name='g1_recent_observation_enabled',value=False),
        SetParameter(name='partial_observation_policy',value='bounded_local_v4'),
        SetParameter(name='g1_parallel_hold_trial',value=True),
        SetParameter(name='g1_pass_release_trial',value=True),
        SetParameter(name='g1_guard_curve_trial',value=True),
        SetParameter(name='g1_trace_path',value='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-10-28/bump_traffic_v1/run4/follow_decisions.jsonl'),
        SetParameter(name='start_signal_mode',value='official_4red_local_candidate'),
        SetParameter(name='race_control_mode',value='legacy'),
        SetParameter(name='g1_traffic_recovery_trial',value=True),
        SetParameter(name='g1_traffic_path_recovery_trial',value=True),
        IncludeLaunchDescription(PythonLaunchDescriptionSource('/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-10-27/official_departure_v2/fixture/simulation.launch.py'),launch_arguments={'track': 'official', 'grid_slot': '3', 'headless': 'true', 'autonomy': 'true', 'traffic_light': 'false', 'autonomy_mode': 'local_pursuit', 'lidar_acquisition': 'sequential', 'lidar_compensation': 'both', 'lidar_rate_hz': '10.0', 'autonomy_control_rate_hz': '50.0', 'depth_camera': 'false', 'speed_bump': 'true', 'chase_camera': 'false', 'batch_static_visuals': 'true', 'scene_broadcaster': 'true', 'speed_profile': 'g1_compare_1mps', 'render_backend': 'wsl_nvidia', 'sensor_wall_timeout_s': '30', 'actuation_mode': 'virtual_mcu', 'runtime_artifacts': '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-10-28/bump_traffic_v1/run4', 'd435i_profile': 'low_load_30', 'evaluation_actor_sdf': '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-10-28/bump_traffic_v1/run4/evaluation_actor.sdf'}.items()),
        ExecuteProcess(cmd=['/usr/bin/python3', '-B', '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-10-27/official_departure_v2/signal_driver_v2.py', '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-10-28/bump_traffic_v1/run4'],output="screen")])
