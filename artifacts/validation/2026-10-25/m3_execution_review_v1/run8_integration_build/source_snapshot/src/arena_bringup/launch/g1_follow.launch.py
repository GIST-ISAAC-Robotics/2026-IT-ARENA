"""G1-a 선택 실행. 기존 단독 모드·독립 보호층·신호 허가를 유지한다."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter


def generate_launch_description():
    # 관측 보완 bool은 계획기와 독립 보호층이 각각 소비한다. 구동/센서 물성은 유지한다.
    launch_file = Path(get_package_share_directory('arena_bringup'))/'launch/local_pursuit.launch.py'
    # 종료 시 TimerAction도 하위 launch의 timeout 설정을 읽어야 한다.
    # 독립 진입점이므로 문맥을 조기에 pop하는 scoped group은 사용하지 않는다.
    return LaunchDescription([GroupAction(scoped=False, actions=[
        DeclareLaunchArgument('g1_boundary_evidence_mode', default_value='connected',
                              choices=['connected', 'fragmented_candidate', 'corner_anchor_candidate', 'corner_distance_candidate']),
        SetParameter(name='g1_boundary_evidence_mode', value=LaunchConfiguration('g1_boundary_evidence_mode')),
        DeclareLaunchArgument('g1_occupancy_timing_mode', default_value='oldest',
                              choices=['oldest', 'per_ray_candidate']),
        SetParameter(name='g1_occupancy_timing_mode', value=LaunchConfiguration('g1_occupancy_timing_mode')),
        DeclareLaunchArgument('g1_corridor_fallback_enabled', default_value='false', choices=['true', 'false'],
                              description='명시 선택형 회랑 후보; 기본 비활성, 거리/점별 시각 후보 필요'),
        SetParameter(name='g1_corridor_fallback_enabled', value=LaunchConfiguration('g1_corridor_fallback_enabled')),
        DeclareLaunchArgument('g1_recent_observation_enabled', default_value='false', choices=['true', 'false'],
                              description='선택형 측방 관측 보완; 계획기와 독립 보호 모두 별도 검사'),
        SetParameter(name='g1_recent_observation_enabled', value=LaunchConfiguration('g1_recent_observation_enabled')),
        DeclareLaunchArgument('g1_m3_execution_trial', default_value='false', choices=['true', 'false'],
                              description='M3 제한 시뮬레이션 후보; per-ray 및 판단 기록 필수, 기본 비활성'),
        SetParameter(name='g1_m3_execution_trial', value=LaunchConfiguration('g1_m3_execution_trial')),
        DeclareLaunchArgument('g1_trace_path', default_value='', description='선택형 판단 단계 JSONL 기록; 기본 꺼짐'),
        SetParameter(name='g1_trace_path', value=LaunchConfiguration('g1_trace_path')),
        SetParameter(name='g1_follow_enabled', value=True),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_file))),
    ])])
