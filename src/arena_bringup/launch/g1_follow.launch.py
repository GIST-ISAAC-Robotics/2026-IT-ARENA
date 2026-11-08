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
        DeclareLaunchArgument('race_control_mode', default_value='legacy', choices=['legacy', 'b3_candidate'],
                              description='명시 선택형 경기 운영; 기본 legacy 유지'),
        SetParameter(name='race_control_mode', value=LaunchConfiguration('race_control_mode')),
        DeclareLaunchArgument('g1_marker_late_start_trial', default_value='false', choices=['true', 'false'],
                              description='B3 준비 뒤 현재 ID45/0과 두 녹색 광원으로 늦은 출발 확인'),
        SetParameter(name='g1_marker_late_start_trial', value=LaunchConfiguration('g1_marker_late_start_trial')),
        DeclareLaunchArgument('g1_solo_straight_budget_trial', default_value='false', choices=['true', 'false'],
                              description='단독 직선 시험: 양쪽 벽을 현재 조향의 속도 거리 근거로만 사용'),
        SetParameter(name='g1_solo_straight_budget_trial', value=LaunchConfiguration('g1_solo_straight_budget_trial')),
        DeclareLaunchArgument('g1_solo_corner_approach_trial', default_value='false', choices=['true', 'false'],
                              description='현재 횡단면 거리로 단독 코너 진입 감속을 앞당기는 시험'),
        SetParameter(name='g1_solo_corner_approach_trial', value=LaunchConfiguration('g1_solo_corner_approach_trial')),
        DeclareLaunchArgument('g1_oblique_gap_trial', default_value='false', choices=['true', 'false'],
                              description='단독 사선 공백 저속 후보. 계획기와 독립 보호층에서 각각 검사'),
        SetParameter(name='g1_oblique_gap_trial', value=LaunchConfiguration('g1_oblique_gap_trial')),
        DeclareLaunchArgument('g1_piecewise_wall_trial', default_value='false', choices=['true', 'false'],
                              description='선택형 관측 벽 두 직선+원호 경로; 점유·독립 보호 유지'),
        SetParameter(name='g1_piecewise_wall_trial', value=LaunchConfiguration('g1_piecewise_wall_trial')),
        DeclareLaunchArgument('g1_standing_wall_reacquire', default_value='false', choices=['true', 'false'],
                              description='재개 후보: 기억 없는 낮은 속도에서 선호 벽의 긴 현재 관측으로 재획득; 기본 비활성'),
        SetParameter(name='g1_standing_wall_reacquire', value=LaunchConfiguration('g1_standing_wall_reacquire')),
        DeclareLaunchArgument('g1_traffic_boundary_refresh_trial', default_value='false', choices=['true', 'false'],
                              description='교통 선택형: 가려진 벽 조각 경로와 이미 승인한 벽의 현재 관측 갱신'),
        SetParameter(name='g1_traffic_boundary_refresh_trial', value=LaunchConfiguration('g1_traffic_boundary_refresh_trial')),
        DeclareLaunchArgument('g1_boundary_evidence_mode', default_value='connected',
                              choices=['connected', 'fragmented_candidate', 'corner_anchor_candidate', 'corner_distance_candidate']),
        SetParameter(name='g1_boundary_evidence_mode', value=LaunchConfiguration('g1_boundary_evidence_mode')),
        DeclareLaunchArgument('g1_occupancy_timing_mode', default_value='oldest',
                              choices=['oldest', 'per_ray_candidate']),
        SetParameter(name='g1_occupancy_timing_mode', value=LaunchConfiguration('g1_occupancy_timing_mode')),
        DeclareLaunchArgument('g1_corridor_fallback_enabled', default_value='false', choices=['true', 'false'],
                              description='명시 선택형 회랑 후보; 기본 비활성, 거리/점별 시각 후보 필요'),
        SetParameter(name='g1_corridor_fallback_enabled', value=LaunchConfiguration('g1_corridor_fallback_enabled')),
        DeclareLaunchArgument('g1_short_boundary_prefix_enabled',default_value='false',choices=['true','false'],
                              description='저속의 짧은 관측 경로 후보; 기본 비활성, 회랑/기록 필요'),
        SetParameter(name='g1_short_boundary_prefix_enabled',value=LaunchConfiguration('g1_short_boundary_prefix_enabled')),
        DeclareLaunchArgument('g1_recent_observation_enabled', default_value='false', choices=['true', 'false'],
                              description='선택형 측방 관측 보완; 계획기와 독립 보호 모두 별도 검사'),
        SetParameter(name='g1_recent_observation_enabled', value=LaunchConfiguration('g1_recent_observation_enabled')),
        DeclareLaunchArgument('g1_m3_execution_trial', default_value='false', choices=['true', 'false'],
                              description='M3 제한 시뮬레이션 후보; per-ray 및 판단 기록 필수, 기본 비활성'),
        SetParameter(name='g1_m3_execution_trial', value=LaunchConfiguration('g1_m3_execution_trial')),
        DeclareLaunchArgument('g1_m3_command_fallback', default_value='false', choices=['true', 'false'],
                              description='선택형 M3 실제 명령 검사 뒤 다음 후보 비교; 기본 비활성'),
        SetParameter(name='g1_m3_command_fallback', value=LaunchConfiguration('g1_m3_command_fallback')),
        DeclareLaunchArgument('g1_m3_release_to_baseline', default_value='false', choices=['true', 'false'],
                              description='선택형 M3 옆차가 앞으로 지나간 뒤 기본 G1 복귀 비교; 기본 비활성'),
        SetParameter(name='g1_m3_release_to_baseline', value=LaunchConfiguration('g1_m3_release_to_baseline')),
        DeclareLaunchArgument('g1_m3_small_center_straight', default_value='false', choices=['true', 'false'],
                              description='선택형 M3 작은 중앙 보정 대신 직진 먼저 검사(관측 복귀 필요); 기본 비활성'),
        SetParameter(name='g1_m3_small_center_straight', value=LaunchConfiguration('g1_m3_small_center_straight')),
        DeclareLaunchArgument('g1_m3_straight_surface_commitment', default_value='false', choices=['true', 'false'],
                              description='선택형 M3 직진 노면 검사를 확약/정지 구간에 맞춤; 기본 비활성'),
        SetParameter(name='g1_m3_straight_surface_commitment', value=LaunchConfiguration('g1_m3_straight_surface_commitment')),
        DeclareLaunchArgument('g1_m3_rounded_occupancy', default_value='false', choices=['true', 'false'],
                              description='선택형 M3 고정 차체 여유와 원판 불확실성 점유; 기본 비활성'),
        SetParameter(name='g1_m3_rounded_occupancy', value=LaunchConfiguration('g1_m3_rounded_occupancy')),
        DeclareLaunchArgument('g1_m3_straight_with_threat', default_value='false', choices=['true', 'false'],
                              description='선택형 M3 가능한 양보 우선, 평행 직진은 hold 전에 검사; 관측/간격 검사 유지'),
        SetParameter(name='g1_m3_straight_with_threat', value=LaunchConfiguration('g1_m3_straight_with_threat')),
        DeclareLaunchArgument('g1_m3_defer_center_on_straight', default_value='false', choices=['true', 'false'],
                              description='선택형 M3 위협 부재 확인/복귀 중 평행 직진 먼저 검사; 기본 비활성'),
        SetParameter(name='g1_m3_defer_center_on_straight', value=LaunchConfiguration('g1_m3_defer_center_on_straight')),
        DeclareLaunchArgument('g1_trace_path', default_value='', description='선택형 판단 단계 JSONL 기록; 기본 꺼짐'),
        DeclareLaunchArgument('pilot_marker_z', default_value='', description='파일럿 전용 ID49 판 중심 높이 덮어쓰기'),
        SetParameter(name='g1_trace_path', value=LaunchConfiguration('g1_trace_path')),
        SetParameter(name='g1_follow_enabled', value=True),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_file)),
                                 launch_arguments={'pilot_marker_z': LaunchConfiguration('pilot_marker_z')}.items()),
    ])])
