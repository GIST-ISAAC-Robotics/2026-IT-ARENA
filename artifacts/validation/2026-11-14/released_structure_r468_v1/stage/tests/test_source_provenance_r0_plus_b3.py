"""10/24 R0+·B3 후속 소스 출처 계약. 해시만 맞추지 않고 조상·이전 바이트·변경 이유를 대조한다.

이 계약은 소스 출처 기록이며 기본 채택·주행 안전·M3 해결 승인을 뜻하지 않는다.
"""
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile
import zipfile

import pytest

ROOT = Path(__file__).resolve().parents[1]
CHAIN = ('dense_stop_evidence_r0_sha256.json', 'lidar_stress_eval_r0_sha256.json', 'per_ray_occupancy_r0_sha256.json',
         'corner_anchor_r0_sha256.json', 'corner_distance_r0_sha256.json', 'corridor_fallback_r0_sha256.json',
         'recent_observation_r0_sha256.json')
R0E3, B3 = 'config/tests/r0_plus_e3_r0_sha256.json', 'config/tests/b3_race_session_r0_sha256.json'
GROUPS = ('approved_changes', 'approved_evaluation_scripts')


def test_short_boundary_prefix_successor_preserves_before_and_current_bytes():
    name='config/tests/short_boundary_prefix_r0_sha256.json'
    child=load(name)
    assert child['parent_contract']=='config/tests/g1_course_evaluation_r0_sha256.json'
    assert sha((ROOT/child['parent_contract']).read_bytes())==child['parent_contract_sha256']
    snapshot=ROOT/child['before_snapshot']
    assert sha(snapshot.read_bytes())==child['before_snapshot_sha256']
    assert len(child['approved_changes'])==5
    assert set(child['approved_evaluation_scripts'])=={'scripts/validate_g1_traffic.py'}
    with zipfile.ZipFile(snapshot) as archive:
        for group in GROUPS:
            for path,row in child[group].items():
                assert row['reason'].strip()
                assert sha(archive.read(path))==row['before']!=row['after']
                assert m3_revision_digest(path, newer_than=name)==row['after']


def sha(data):
    return hashlib.sha256(data).hexdigest()


def load(name):
    return json.loads((ROOT/name).read_text(encoding='utf-8'))


def blob(commit, path):
    return subprocess.run(['git', 'show', f'{commit}:{path}'], cwd=ROOT, capture_output=True, check=True).stdout


def previous_after():
    """이전 계약 사슬에서 경로별 마지막 after. 이전 계약 파일은 부모 해시로 불변을 확인한다."""
    latest, parent = {}, None
    for name in CHAIN:
        contract = load('config/tests/'+name)
        if parent:
            assert contract['parent_contract'] == parent
            assert sha((ROOT/parent).read_bytes()) == contract['parent_contract_sha256']
        for group in GROUPS:
            for path, row in contract.get(group, {}).items():
                latest[path] = row['after']
        parent = 'config/tests/'+name
    return latest


def test_r0_plus_e3_successor_links_head_bytes_to_recorded_run_bytes():
    contract = load(R0E3)
    assert contract['parent_contract'] == 'config/tests/recent_observation_r0_sha256.json'
    assert sha((ROOT/contract['parent_contract']).read_bytes()) == contract['parent_contract_sha256']
    assert '승인 아님' in contract['scope']
    commit = contract['before_source']['git_commit']
    report = json.loads((ROOT/contract['after_source']['report']).read_text(encoding='utf-8'))['source_sha256']
    latest = previous_after()
    rows = [(path, row) for group in GROUPS for path, row in contract[group].items()]
    assert {path for path, _ in rows} == {'src/arena_autonomy/arena_autonomy/local_pursuit.py',
        'scripts/validate_g1_traffic.py', 'scripts/g1_traffic_eval.py', 'scripts/validate_basic_autonomy.py'}
    for path, row in rows:
        assert row['reason'].strip()
        assert row['before'] == sha(blob(commit, path))  # 이전 바이트는 고정 커밋에 그대로 있다
        assert row['before'] == latest[path]  # 이전 계약의 마지막 after와 이어진다
        assert row['after'] == report[path] != row['before']
    for path, value in contract['new_dependencies'].items():
        assert report[path] == value == sha((ROOT/path).read_bytes())


def test_r0_plus_e3_after_bytes_are_preserved_in_local_run_snapshot():
    contract = load(R0E3)
    snapshot = ROOT/contract['after_source']['snapshot']
    if not snapshot.is_file():
        pytest.skip('local-only E3 source snapshot is absent (Git-excluded); report hashes are checked separately')
    archive = zipfile.ZipFile(snapshot)
    for group in GROUPS:
        for path, row in contract[group].items():
            assert sha(archive.read(path)) == row['after']


def b3_revision_digest(path):
    """후속 M3 변경이 있는 경로는 보존된 B3 바이트와 현재 M3 출처를 함께 확인한다."""
    successor = load('config/tests/m3_execution_r0_sha256.json')
    if path not in successor['changes']:
        return sha((ROOT/path).read_bytes())
    row = successor['changes'][path]
    assert m3_revision_digest(path) == row['after']
    snapshot = ROOT/successor['before_snapshot']
    if snapshot.is_file():  # Git 제외 원시 snapshot이 없는 체크아웃에서도 출처 사슬은 확인한다.
        with tarfile.open(snapshot) as archive:
            assert sha(archive.extractfile(path).read()) == row['before']
    return row['before']


def test_b3_successor_continues_r0_plus_and_preserves_b3_bytes():
    contract, parent = load(B3), load(R0E3)
    assert contract['parent_contract'] == R0E3
    assert sha((ROOT/R0E3).read_bytes()) == contract['parent_contract_sha256']
    assert '승인 아님' in contract['scope']
    for group in GROUPS:
        for path, row in contract[group].items():
            assert row['reason'].strip() and row['before'] == parent[group][path]['after'] != row['after']
            assert b3_revision_digest(path) == row['after']
    commit = contract['newly_tracked_before_source']['git_commit']
    report = json.loads((ROOT/contract['newly_tracked_before_source']['e3_report']).read_text(encoding='utf-8'))
    for path, row in contract['newly_tracked_changes'].items():
        # E3 실행 때까지 HEAD와 같았고 B3에서만 바뀌었다.
        assert row['before'] == sha(blob(commit, path)) == report['source_sha256'][path]
        assert row['reason'].strip() and m3_revision_digest(path) == row['after'] != row['before']
    for path, value in contract['new_dependencies'].items():
        assert m3_revision_digest(path) == value


M3_SUCCESSORS = ('config/tests/m3_command_selection_r0_sha256.json', 'config/tests/m3_baseline_release_r0_sha256.json', 'config/tests/m3_release_gate_r0_sha256.json',
                 'config/tests/m3_small_center_r0_sha256.json', 'config/tests/m3_surface_commitment_r0_sha256.json', 'config/tests/m3_retained_threats_r0_sha256.json',
                 'config/tests/m3_sensor_stress_r0_sha256.json', 'config/tests/m4_operation_r0_sha256.json',
                 'config/tests/m4_operation_clock_r0_sha256.json', 'config/tests/m4_new_race_r0_sha256.json',
                 'config/tests/m3_guarded_return_r0_sha256.json', 'config/tests/m4_second_start_r0_sha256.json', 'config/tests/m2_merge_r0_sha256.json', 'config/tests/m3_exact_cost_r0_sha256.json', 'config/tests/m2_motion_stress_r0_sha256.json', 'config/tests/m2_count_fault_r0_sha256.json', 'config/tests/m2_motion_config_r0_sha256.json', 'config/tests/m5_publication_r0_sha256.json', 'config/tests/partial_observation_r0_sha256.json',
                 'config/tests/profile_bundle_selector_r0_sha256.json', 'config/tests/g1_course_evaluation_r0_sha256.json', 'config/tests/short_boundary_prefix_r0_sha256.json',
                 'config/tests/local_start_signal_r0_sha256.json', 'config/tests/traffic_recovery_trial_r0_sha256.json', 'config/tests/traffic_recovery_release_fringe_r1_sha256.json', 'config/tests/traffic_path_recovery_r0_sha256.json', 'config/tests/traffic_anchor_set_r1_sha256.json', 'config/tests/traffic_speed_recheck_r1_sha256.json', 'config/tests/traffic_positive_clearance_r1_sha256.json', 'config/tests/traffic_positive_bundle_r1_sha256.json', 'config/tests/traffic_side_scope_r1_sha256.json', 'config/tests/traffic_manifest_r1_sha256.json', 'config/tests/short_prefix_origin_r1_sha256.json', 'config/tests/rotation_reobservation_r1_sha256.json', 'config/tests/actor_tail_capture_r1_sha256.json', 'config/tests/small_alignment_r1_sha256.json', 'config/tests/rolling_clearance_r1_sha256.json', 'config/tests/fresh_scan_wait_r1_sha256.json', 'config/tests/g2_handoff_manifest_r1_sha256.json', 'config/tests/rear_lane_keep_r1_sha256.json', 'config/tests/observed_lane_preference_r1_sha256.json', 'config/tests/measured_lane_preference_r1_sha256.json', 'config/tests/fresh_confirmation_r2_sha256.json', 'config/tests/g2_corridor_r1_sha256.json', 'config/tests/g2_signed_hold_r1_sha256.json', 'config/tests/g2_handoff_corridor_r1_sha256.json', 'config/tests/g2_preparation_hold_r1_sha256.json', 'config/tests/g2_initial_reference_r1_sha256.json', 'config/tests/g2_initial_reference_r2_sha256.json', 'config/tests/g1_same_scan_history_r1_sha256.json', 'config/tests/g2_directional_reference_r1_sha256.json', 'config/tests/g1_forward_same_path_r1_sha256.json', 'config/tests/g2_motion_pause_r1_sha256.json', 'config/tests/g2_relative_yield_r1_sha256.json', 'config/tests/g2_relative_yield_r2_sha256.json', 'config/tests/continuous_prefix_r1_sha256.json', 'config/tests/signed_forward_r1_sha256.json', 'config/tests/race_continuity_r1_sha256.json', 'config/tests/race_course_profile_r1_sha256.json', 'config/tests/race_selector_speed_r1_sha256.json', 'config/tests/corner_approach_r1_sha256.json', 'config/tests/race_follow_pair_r1_sha256.json', 'config/tests/race_follow_slow_r1_sha256.json', 'config/tests/prelap_signal_r1_sha256.json', 'config/tests/race_follow_closing_r1_sha256.json', 'config/tests/turn_preview_r1_sha256.json', 'config/tests/team_signal_course_r1_sha256.json', 'config/tests/race_acceleration_r1_sha256.json', 'config/tests/rolling_lead_r1_sha256.json', 'config/tests/qualifying_profile_r1_sha256.json', 'config/tests/recent_forward_r1_sha256.json', 'config/tests/follow_continuity_r1_sha256.json', 'config/tests/yield_progress_r1_sha256.json', 'config/tests/released_rear_sweep_r1_sha256.json',
                 'config/tests/low_speed_arc_reacquire_r1_sha256.json', 'config/tests/solo_route_r1_sha256.json', 'config/tests/solo_headway_r1_sha256.json', 'config/tests/sweep_batch_r1_sha256.json', 'config/tests/follow_bundle_refresh_r1_sha256.json', 'config/tests/released_fringe_budget_r1_sha256.json', 'config/tests/solo_corridor_arc_r1_sha256.json', 'config/tests/ray_wall_continuity_r1_sha256.json', 'config/tests/turn_preview_gate_r1_sha256.json', 'config/tests/boundary_bridge_r1_sha256.json',
                 'config/tests/official_facility_r1_sha256.json', 'config/tests/official_start_region_r1_sha256.json',
                 'config/tests/official_forward_r1_sha256.json', 'config/tests/observed_rejoin_r2_sha256.json', 'config/tests/observed_forward_pace_r3_sha256.json', 'config/tests/official_traffic_r1_sha256.json', 'config/tests/official_traffic_eval_r2_sha256.json', 'config/tests/official_ops_r1_sha256.json', 'config/tests/official_ops_cleanup_r2_sha256.json', 'config/tests/official_ops_traffic_r3_sha256.json', 'config/tests/official_ops_browser_r4_sha256.json', 'config/tests/official_ops_route_r5_sha256.json', 'config/tests/solo_alternative_r1_sha256.json', 'config/tests/handoff_profile_r1_sha256.json', 'config/tests/same_call_reference_r1_sha256.json', 'config/tests/entry_obstruction_r1_sha256.json', 'config/tests/solo_ray_combination_r1_sha256.json', 'config/tests/local_sensor_mount_r1_sha256.json', 'config/tests/standing_steering_r1_sha256.json', 'config/tests/observed_gap_recovery_r1_sha256.json', 'config/tests/post_free_facility_r1_sha256.json', 'config/tests/ordinary_resume_r1_sha256.json', 'config/tests/actor_outline_r1_sha256.json', 'config/tests/traffic_rear_grid_r1_sha256.json', 'config/tests/side_departure_r1_sha256.json', 'config/tests/side_departure_rear_r2_sha256.json', 'config/tests/side_departure_selector_r3_sha256.json', 'config/tests/current_lane_preference_r1_sha256.json', 'config/tests/lidar_mount_trial_r1_sha256.json', 'config/tests/lidar_mount_adapted_r2_sha256.json', 'config/tests/lidar_mount_pilot_r3_sha256.json', 'config/tests/lidar_mount_speed_r4_sha256.json', 'config/tests/lidar_mount_launch_record_r5_sha256.json', 'config/tests/lidar_mount_traffic_marker_r6_sha256.json', 'config/tests/lidar_mount_structure_gate_r7_sha256.json', 'config/tests/continuous_forward_pace_r8_sha256.json', 'config/tests/moving_pass_r9_sha256.json')


M3_SUCCESSORS += ('config/tests/moving_pass_witness_r10_sha256.json','config/tests/moving_pass_width_r11_sha256.json',
                  'config/tests/moving_pass_rejoin_r12_sha256.json',
                  'config/tests/moving_pass_withdrawal_r13_sha256.json',
                  'config/tests/moving_pass_yield_r14_sha256.json')
# 11/5 단독 직선 속도 후보 재적용(기본 OFF). 일반 후속 계약 검사가 r14 다음으로 확인한다.
M3_SUCCESSORS += ('config/tests/solo_straight_budget_r15_sha256.json',)
M3_SUCCESSORS += ('config/tests/moving_pass_handoff_r16_sha256.json',)
# 11/7 공식 v2026.11.06 별도 실행 월드 등록·명시 선택(기본 official·제어기 유지).
M3_SUCCESSORS += ('config/tests/track_migration_v2026_11_06_r1_sha256.json',)
M3_SUCCESSORS += ('config/tests/b0390_migration_r2_sha256.json',)
M3_SUCCESSORS += ('config/tests/course_section_r3_sha256.json',)
M3_SUCCESSORS += ('config/tests/piecewise_corner_r4_sha256.json',)
# 11/7 교통·운영 평가 도구 명시 track 선택(기본 official·제어기 유지).
M3_SUCCESSORS += ('config/tests/traffic_course_track_r5_sha256.json',)
# 11/7 G1/G2 launch 가드 track 조건을 두 공식 2×5 파생 track으로 확장(나머지 조건 불변).
M3_SUCCESSORS += ('config/tests/handoff_track_guard_r6_sha256.json',)
M3_SUCCESSORS += ('config/tests/grass_alignment_r7_sha256.json',)
M3_SUCCESSORS += ('config/tests/paired_prefix_r8_sha256.json',)
M3_SUCCESSORS += ('config/tests/return_guide_front_r9_sha256.json',)
M3_SUCCESSORS += ('config/tests/continuous_confirmation_r10_sha256.json',)
M3_SUCCESSORS += ('config/tests/standing_reacquire_r11_sha256.json',)
M3_SUCCESSORS += ('config/tests/marker_late_start_r12_sha256.json',)
M3_SUCCESSORS += ('config/tests/traffic_piecewise_r13_sha256.json',)
M3_SUCCESSORS += ('config/tests/oblique_gap_r14_sha256.json',)
M3_SUCCESSORS += ('config/tests/moving_pass_medium_r15_sha256.json',)
M3_SUCCESSORS += ('config/tests/stationary_wall_witness_r16_sha256.json',)
M3_SUCCESSORS += ('config/tests/stationary_rejoin_state_r17_sha256.json',)
M3_SUCCESSORS += ('config/tests/rear_face_motion_r18_sha256.json',)
M3_SUCCESSORS += ('config/tests/solo_corner_approach_r19_sha256.json',)
M3_SUCCESSORS += ('config/tests/official_grid_multi_r20_sha256.json',)
# 11/8 기본 OFF 한 모서리 경로의 관측 탐색 예산 보완(local_path.py 한 파일).
M3_SUCCESSORS += ('config/tests/corner_path_budget_r21_sha256.json',)
# 11/8 기본 OFF 한 모서리 경로의 앵커 오차를 생성 경로 거리로 판정(local_path.py·local_follow.py).
M3_SUCCESSORS += ('config/tests/corner_path_anchor_r22_sha256.json',)
# 11/8 기본 OFF 선택형 일반 곡선 반환에도 같은 경로 앵커 지표 적용(local_path.py·local_follow.py).
M3_SUCCESSORS += ('config/tests/corner_path_anchor_transition_r23_sha256.json',)
M3_SUCCESSORS += ('config/tests/parallel_measured_continuity_r24_sha256.json',)
M3_SUCCESSORS += ('config/tests/oblique_sweep_radius_r25_sha256.json',)
M3_SUCCESSORS += ('config/tests/operator_link_handoff_r26_sha256.json',)
M3_SUCCESSORS += ('config/tests/traffic_standing_reacquire_r27_sha256.json',)
M3_SUCCESSORS += ('config/tests/side_corner_actor_r28_sha256.json',)
M3_SUCCESSORS += ('config/tests/traffic_boundary_refresh_r29_sha256.json',)
M3_SUCCESSORS += ('config/tests/fragmented_wall_refresh_r30_sha256.json',)
M3_SUCCESSORS += ('config/tests/moving_pass_faster_r31_sha256.json',)
M3_SUCCESSORS += ('config/tests/moving_pass_wall_mask_r32_sha256.json',)
M3_SUCCESSORS += ('config/tests/stopped_entry_centering_r33_sha256.json',)
M3_SUCCESSORS += ('config/tests/g1g2_lidar_stress_r34_sha256.json',)
M3_SUCCESSORS += ('config/tests/piecewise_noisy_corner_r35_sha256.json',)
M3_SUCCESSORS += ('config/tests/adaptive_entry_preview_r36_sha256.json',)
M3_SUCCESSORS += ('config/tests/query_local_side_scope_r37_sha256.json',)
M3_SUCCESSORS += ('config/tests/masked_motion_speed_r38_sha256.json',)
M3_SUCCESSORS += ('config/tests/short_entry_transition_r39_sha256.json',)
M3_SUCCESSORS += ('config/tests/adaptive_corridor_offset_r40_sha256.json',)
M3_SUCCESSORS += ('config/tests/current_corner_support_r41_sha256.json',)
M3_SUCCESSORS += ('config/tests/follow_bundle_closure_r42_sha256.json',)
M3_SUCCESSORS += ('config/tests/grid_cost_r43_sha256.json',)
M3_SUCCESSORS += ('config/tests/faster_entry_consistency_r44_sha256.json',)
M3_SUCCESSORS += ('config/tests/corner_occlusion_r45_sha256.json',)
M3_SUCCESSORS += ('config/tests/retained_corner_agreement_r46_sha256.json',)
M3_SUCCESSORS += ('config/tests/current_corner_fallback_r47_sha256.json',)
M3_SUCCESSORS += ('config/tests/observed_corner_continuity_r48_sha256.json',)
M3_SUCCESSORS += ('config/tests/p22_entry_wiring_r49_sha256.json',)
M3_SUCCESSORS += ('config/tests/p22_route_connected_r50_sha256.json',)
M3_SUCCESSORS += ('config/tests/p22_exit_alignment_r51_sha256.json',)
# 11/9 신호 양지주 실행 사본 선택(기본 none)·갈림길 마커 누락 후보(기본 OFF).
M3_SUCCESSORS += ('config/tests/signal_posts_markers_r52_sha256.json',)
M3_SUCCESSORS += ('config/tests/signal_posts_marker_launch_r53_sha256.json',)
M3_SUCCESSORS += ('config/tests/junction_priority_r54_sha256.json',)
M3_SUCCESSORS += ('config/tests/junction_divergence_r55_sha256.json',)
M3_SUCCESSORS += ('config/tests/nearby_face_motion_r56_sha256.json',)
M3_SUCCESSORS += ('config/tests/nearby_long_window_r57_sha256.json',)
M3_SUCCESSORS += ('config/tests/tracked_wall_occluder_r58_sha256.json',)
M3_SUCCESSORS += ('config/tests/forward_rejoin_posts_r59_sha256.json',)
M3_SUCCESSORS += ('config/tests/current_guard_path_r60_sha256.json',)
M3_SUCCESSORS += ('config/tests/confirmed_passage_fringe_r61_sha256.json',)
M3_SUCCESSORS += ('config/tests/confirmed_fringe_restore_r62_sha256.json',)
M3_SUCCESSORS += ('config/tests/observed_entry_return_r63_sha256.json',)
M3_SUCCESSORS += ('config/tests/released_straight_recovery_r64_sha256.json',)
M3_SUCCESSORS += ('config/tests/released_straight_restore_r65_sha256.json',)
M3_SUCCESSORS += ('config/tests/c1_native_sampling_r66_sha256.json',)
M3_SUCCESSORS += ('config/tests/c1_native_time_glue_r67_sha256.json',)
M3_SUCCESSORS += ('config/tests/partial_request_r68_sha256.json',)
M3_SUCCESSORS += ('config/tests/forward_metric_r69_sha256.json',)
M3_SUCCESSORS += ('config/tests/native_contract_r70_sha256.json',)
M3_SUCCESSORS += ('config/tests/imu_corner_r71_sha256.json',)
M3_SUCCESSORS += ('config/tests/native_cpu_r72_sha256.json',)
M3_SUCCESSORS += ('config/tests/gearing_81t_r73_sha256.json',)
M3_SUCCESSORS += ('config/tests/forward_prefix_r74_sha256.json',)
M3_SUCCESSORS += ('config/tests/longitudinal_surface_r75_sha256.json',)
M3_SUCCESSORS += ('config/tests/rear_structure_memory_r77_sha256.json',)
M3_SUCCESSORS += ('config/tests/confirmed_rear_fringe_r78_sha256.json',)
M3_SUCCESSORS += ('config/tests/native_traffic_r79_sha256.json',)
M3_SUCCESSORS += ('config/tests/native_nonreturn_r81_sha256.json',)
M3_SUCCESSORS += ('config/tests/path_sampling_r82_sha256.json',)
M3_SUCCESSORS += ('config/tests/wall_occluder_r83_sha256.json',)
M3_SUCCESSORS += ('config/tests/guard_masked_speed_r84_sha256.json', 'config/tests/traffic_cpu_r85_sha256.json',)
M3_SUCCESSORS += ('config/tests/retained_structure_r86_sha256.json',)
M3_SUCCESSORS += ('config/tests/occluder_cpu_r89_sha256.json',)
M3_SUCCESSORS += ('config/tests/control_cadence_r91_sha256.json',)
M3_SUCCESSORS += ('config/tests/control_rate_launch_r93_sha256.json',)
M3_SUCCESSORS += ('config/tests/diagnostic_window_r95_sha256.json',)
M3_SUCCESSORS += ('config/tests/vision_completion_r96_sha256.json',)
M3_SUCCESSORS += ('config/tests/query_set_r103_sha256.json',)
M3_SUCCESSORS += ('config/tests/async_handoff_r104_sha256.json',)
M3_SUCCESSORS += ('config/tests/short_corner_bevel_r105_sha256.json',)
M3_SUCCESSORS += ('config/tests/scan_start_phase_r107_sha256.json',)
M3_SUCCESSORS += ('config/tests/forward_gap_scope_r109_sha256.json',)
# 실행 등록의 누적 목록과 선형 변경 사슬을 구분한다. 원 등록 파일을 고치지
# 않고 r126 보존 바이트와 현재 네 파일의 정확한 차이를 한 단계로 연결한다.
M3_SUCCESSORS += ('config/tests/review_checkpoint_r157_sha256.json',)
M3_SUCCESSORS += ('config/tests/development_speed_limit_r162_sha256.json',)
M3_SUCCESSORS += ('config/tests/side_view_cost_r172_sha256.json',)
M3_SUCCESSORS += ('config/tests/current_path_recheck_r177_sha256.json',)
M3_SUCCESSORS += ('config/tests/absent_guard_signed_r179_sha256.json',)
M3_SUCCESSORS += ('config/tests/fresh_submission_r184_sha256.json',)
M3_SUCCESSORS += ('config/tests/guard_policy_r190_sha256.json',)
M3_SUCCESSORS += ('config/tests/guard_policy_node_r191_sha256.json',)
M3_SUCCESSORS += ('config/tests/execution_trace_metadata_r193_sha256.json',)
M3_SUCCESSORS += ('config/tests/depth_batch_r197_sha256.json',)
M3_SUCCESSORS += ('config/tests/released_body_gap_r199_sha256.json',)
M3_SUCCESSORS += ('config/tests/released_mount_gap_r202_sha256.json',)
M3_SUCCESSORS += ('config/tests/static_side_elapsed_r209_sha256.json',)
M3_SUCCESSORS += ('config/tests/expiry_reference_r218_sha256.json',)
M3_SUCCESSORS += ('config/tests/traffic_solo_gap_r224_sha256.json',)
M3_SUCCESSORS += ('config/tests/motion_query_batch_r227_sha256.json',)
M3_SUCCESSORS += ('config/tests/stale_command_observation_r229_sha256.json',)
M3_SUCCESSORS += ('config/tests/selected_trace_r233_sha256.json',)
M3_SUCCESSORS += ('config/tests/receiver_lease_host_r238_sha256.json',)
M3_SUCCESSORS += ('config/tests/recheck_delivery_r242_sha256.json',)
M3_SUCCESSORS += ('config/tests/numeric_backend_r247_sha256.json',)
M3_SUCCESSORS += ('config/tests/hardware_shadow_r249_sha256.json',)
M3_SUCCESSORS += ('config/tests/mcu_status_r254_sha256.json',)
M3_SUCCESSORS += ('config/tests/imu_fifo_clock_r268_sha256.json',)
M3_SUCCESSORS += ('config/tests/estimated_time_r269_sha256.json',)
M3_SUCCESSORS += ('config/tests/c1_tracking_r270_sha256.json',)
M3_SUCCESSORS += ('config/tests/paired_clock_r277_sha256.json',)
M3_SUCCESSORS += ('config/tests/reusable_preview_r280_sha256.json',)
M3_SUCCESSORS += ('config/tests/motor_feedback_r285_sha256.json',)
M3_SUCCESSORS += ('config/tests/cad_mount_r290_sha256.json',)
M3_SUCCESSORS += ('config/tests/runtime_callback_r312_sha256.json',)
M3_SUCCESSORS += ('config/tests/runtime_source_r320_sha256.json',)


def test_moving_pass_revisions_link_registered_sources_and_before_bytes():
    previous='config/tests/continuous_forward_pace_r8_sha256.json'
    for name in ('config/tests/moving_pass_r9_sha256.json','config/tests/moving_pass_witness_r10_sha256.json',
                 'config/tests/moving_pass_width_r11_sha256.json','config/tests/moving_pass_rejoin_r12_sha256.json',
                 'config/tests/moving_pass_withdrawal_r13_sha256.json',
                 'config/tests/moving_pass_yield_r14_sha256.json',
                 'config/tests/moving_pass_handoff_r16_sha256.json'):
        child=load(name)
        previous=M3_SUCCESSORS[M3_SUCCESSORS.index(name)-1]
        assert child['parent_contract']==previous
        assert sha((ROOT/previous).read_bytes())==child['parent_contract_sha256']
        before=ROOT/child['before_snapshot'];after=ROOT/child['after_registration']
        assert sha(before.read_bytes())==child['before_snapshot_sha256']
        assert sha(after.read_bytes())==child['after_registration_sha256']
        sources=load(child['after_registration'])['sources']
        with zipfile.ZipFile(before) as archive:
            for p,row in {**child['approved_changes'],**child['approved_evaluation_scripts']}.items():
                assert sha(archive.read(p))==row['before']!=row['after']==sources[p]
                assert m3_revision_digest(p,newer_than=name)==row['after']
            for p,h in child['new_dependencies'].items():
                assert h==sources[p]==m3_revision_digest(p,newer_than=name)
        previous=name


def test_local_signal_successor_preserves_failed_five_car_source():
    name = 'config/tests/local_start_signal_r0_sha256.json'
    child = load(name)
    assert child['parent_contract'] == 'config/tests/short_boundary_prefix_r0_sha256.json'
    assert sha((ROOT/child['parent_contract']).read_bytes()) == child['parent_contract_sha256']
    assert sha((ROOT/child['before_snapshot']).read_bytes()) == child['before_snapshot_sha256']
    assert set(child['approved_evaluation_scripts']) == {'scripts/validate_g1_traffic.py', 'scripts/validate_pipeline_faults_ros.py'}
    assert set(child['new_dependencies']) == {'src/arena_autonomy/arena_autonomy/race_start_local.py'}
    with zipfile.ZipFile(ROOT/child['before_snapshot']) as archive:
        for group in GROUPS:
            for path, row in child[group].items():
                assert row['reason'].strip()
                assert sha(archive.read(path)) == row['before'] != row['after']
                assert m3_revision_digest(path, newer_than=name) == row['after']
    for path, digest in child['new_dependencies'].items():
        assert m3_revision_digest(path, newer_than=name) == digest


def m3_revision_digest(path, newer_than=None):
    """현재 바이트를 M3 후속 계약 사슬(최신→과거)로 되돌린 해시. 각 단계의 실행 사본 바이트도 대조한다.

    newer_than를 주면 그 계약보다 나중 계약만 되돌린다(그 계약의 after 시점).
    """
    digest = sha((ROOT/path).read_bytes())
    for name in reversed(M3_SUCCESSORS):
        if name == newer_than:
            break
        successor = load(name)
        rows = {**successor['approved_changes'], **successor['approved_evaluation_scripts']}
        if path not in rows:
            continue
        row = rows[path]
        assert digest == row['after']
        snapshot = ROOT/successor['before_snapshot']
        if snapshot.is_file():
            if tarfile.is_tarfile(snapshot):
                with tarfile.open(snapshot) as archive:
                    assert sha(archive.extractfile(path).read()) == row['before']
            else:
                with zipfile.ZipFile(snapshot) as archive:
                    assert sha(archive.read(path)) == row['before']
        digest = row['before']
    return digest


def test_m3_successor_records_current_bytes_without_rewriting_previous_contract():
    contract = load('config/tests/m3_execution_r0_sha256.json')
    assert contract['changes'] == {**contract['approved_changes'], **contract['approved_evaluation_scripts']}
    assert contract['parent_contract'] == B3
    assert sha((ROOT/B3).read_bytes()) == contract['parent_contract_sha256']
    snapshot = ROOT/contract['before_snapshot']
    if snapshot.is_file():
        assert sha(snapshot.read_bytes()) == contract['before_snapshot_sha256']
        with tarfile.open(snapshot) as archive:
            for path, row in contract['changes'].items():
                assert sha(archive.extractfile(path).read()) == row['before']
    for path, row in contract['changes'].items():
        assert row['reason'].strip()
        assert m3_revision_digest(path) == row['after'] != row['before']
    for path, value in contract['dependencies'].items():
        assert m3_revision_digest(path) == value


def test_command_selection_continues_m3_recorded_run_bytes():
    child = load('config/tests/m3_command_selection_r0_sha256.json')
    parent = load(child['parent_contract'])
    assert sha((ROOT/child['parent_contract']).read_bytes()) == child['parent_contract_sha256']
    snapshot = ROOT/child['before_snapshot']
    if snapshot.is_file():
        assert sha(snapshot.read_bytes()) == child['before_snapshot_sha256']
    for group in GROUPS:
        for path,row in child[group].items():
            expected = parent['changes'][path]['after'] if path in parent['changes'] else parent['dependencies'][path]
            assert m3_revision_digest(path) == row['before'] == expected
            assert m3_revision_digest(path, newer_than=M3_SUCCESSORS[0]) == row['after'] != row['before']


def test_baseline_release_continues_command_selection_run13_bytes():
    child, parent = load(M3_SUCCESSORS[1]), load(M3_SUCCESSORS[0])
    assert child['parent_contract'] == M3_SUCCESSORS[0] and '승인 아님' in child['scope']
    assert sha((ROOT/M3_SUCCESSORS[0]).read_bytes()) == child['parent_contract_sha256']
    snapshot = ROOT/child['before_snapshot']
    if snapshot.is_file():
        assert sha(snapshot.read_bytes()) == child['before_snapshot_sha256']
    parent_rows = {**parent['approved_changes'], **parent['approved_evaluation_scripts']}
    for group in GROUPS:
        for path, row in child[group].items():
            assert row['reason'].strip()
            assert row['before'] == parent_rows[path]['after'] == m3_revision_digest(path, newer_than=M3_SUCCESSORS[0])
            assert m3_revision_digest(path, newer_than=M3_SUCCESSORS[1]) == row['after'] != row['before']


def test_release_gate_continues_preserved_rejected_release_bytes():
    child, parent = load(M3_SUCCESSORS[2]), load(M3_SUCCESSORS[1])
    assert child['parent_contract'] == M3_SUCCESSORS[1]
    assert sha((ROOT/M3_SUCCESSORS[1]).read_bytes()) == child['parent_contract_sha256']
    snapshot = ROOT/child['before_snapshot']
    if snapshot.is_file():
        assert sha(snapshot.read_bytes()) == child['before_snapshot_sha256']
    parent_rows = {**parent['approved_changes'], **parent['approved_evaluation_scripts']}
    for group in GROUPS:
        for path, row in child[group].items():
            assert row['reason'].strip()
            assert row['before'] == parent_rows[path]['after']
            assert m3_revision_digest(path, newer_than=M3_SUCCESSORS[1]) == row['before']
            # 10/25 작은 중앙 보정 비교 계약이 뒤에 이어진다. 현재 바이트는 그 계약을 되돌려 대조한다.
            assert m3_revision_digest(path, newer_than=M3_SUCCESSORS[2]) == row['after'] != row['before']


def test_small_center_continues_release_gate_run19_bytes():
    """run17/19/21/24 실행 사본 바이트에서 작은 중앙 보정 비교 옵션으로. 이전 계약·실행 사본은 수정하지 않는다."""
    child, parent = load(M3_SUCCESSORS[3]), load(M3_SUCCESSORS[2])
    assert child['parent_contract'] == M3_SUCCESSORS[2] and '승인 아님' in child['scope']
    assert sha((ROOT/M3_SUCCESSORS[2]).read_bytes()) == child['parent_contract_sha256']
    snapshot = ROOT/child['before_snapshot']
    if snapshot.is_file():
        assert sha(snapshot.read_bytes()) == child['before_snapshot_sha256']
    parent_rows = {**parent['approved_changes'], **parent['approved_evaluation_scripts']}
    for group in GROUPS:
        for path, row in child[group].items():
            assert row['reason'].strip()
            if path in parent_rows:
                assert row['before'] == parent_rows[path]['after']
            assert m3_revision_digest(path, newer_than=M3_SUCCESSORS[2]) == row['before']
            assert m3_revision_digest(path, newer_than=M3_SUCCESSORS[3]) == row['after'] != row['before']


def test_surface_commitment_continues_run32_source():
    child, parent = load(M3_SUCCESSORS[4]), load(M3_SUCCESSORS[3])
    assert child['parent_contract'] == M3_SUCCESSORS[3]
    assert sha((ROOT/M3_SUCCESSORS[3]).read_bytes()) == child['parent_contract_sha256']
    parent_rows = {**parent['approved_changes'], **parent['approved_evaluation_scripts']}
    for group in GROUPS:
        for path, row in child[group].items():
            assert row['before'] == parent_rows[path]['after'] and row['reason'].strip()
            assert m3_revision_digest(path, newer_than=M3_SUCCESSORS[3]) == row['before']
            assert m3_revision_digest(path, newer_than=M3_SUCCESSORS[4]) == row['after'] != row['before']


def test_retained_threats_continues_run35_source():
    child, parent = load(M3_SUCCESSORS[5]), load(M3_SUCCESSORS[4])
    assert child['parent_contract'] == M3_SUCCESSORS[4]
    assert sha((ROOT/M3_SUCCESSORS[4]).read_bytes()) == child['parent_contract_sha256']
    rows = {**parent['approved_changes'], **parent['approved_evaluation_scripts']}
    for group in GROUPS:
        for path, row in child[group].items():
            assert row['before'] == rows[path]['after'] and row['reason'].strip()
            assert m3_revision_digest(path, newer_than=M3_SUCCESSORS[4]) == row['before']
            assert m3_revision_digest(path, newer_than=M3_SUCCESSORS[5]) == row['after'] != row['before']


def test_sensor_stress_only_extends_evaluator_option_after_run55():
    child, parent = load(M3_SUCCESSORS[6]), load(M3_SUCCESSORS[5])
    assert child['parent_contract'] == M3_SUCCESSORS[5]
    assert sha((ROOT/M3_SUCCESSORS[5]).read_bytes()) == child['parent_contract_sha256']
    assert child['approved_changes'] == {}
    assert set(child['approved_evaluation_scripts']) == {'scripts/validate_g1_traffic.py'}
    path = 'scripts/validate_g1_traffic.py'
    row = child['approved_evaluation_scripts'][path]
    assert row['reason'].strip()
    assert row['before'] == parent['approved_evaluation_scripts'][path]['after']
    assert m3_revision_digest(path, newer_than=M3_SUCCESSORS[6]) == row['after'] != row['before']


def test_m4_operation_successor_preserves_run59_and_changes_no_controller():
    child = load(M3_SUCCESSORS[7])
    assert child['parent_contract'] == M3_SUCCESSORS[6]
    assert sha((ROOT/M3_SUCCESSORS[6]).read_bytes()) == child['parent_contract_sha256']
    assert set(child['approved_changes']) == {'src/arena_bringup/launch/g1_follow.launch.py'}
    assert set(child['approved_evaluation_scripts']) == {'scripts/validate_g1_traffic.py'}
    with zipfile.ZipFile(ROOT/child['before_snapshot']) as archive:
        for group in GROUPS:
            for path,row in child[group].items():
                assert sha(archive.read(path)) == row['before'] != row['after']
                assert m3_revision_digest(path, newer_than=M3_SUCCESSORS[7]) == row['after']
    for path,value in child['new_dependencies'].items():
        assert m3_revision_digest(path, newer_than=M3_SUCCESSORS[7]) == value


def test_m4_clock_boundary_preserves_original_evaluator_bytes():
    child, parent = load(M3_SUCCESSORS[8]), load(M3_SUCCESSORS[7])
    assert child['parent_contract'] == M3_SUCCESSORS[7]
    assert sha((ROOT/M3_SUCCESSORS[7]).read_bytes()) == child['parent_contract_sha256']
    assert child['approved_changes'] == {}
    assert set(child['approved_evaluation_scripts']) == {'scripts/m4_operation_eval.py'}
    for path,row in child['approved_evaluation_scripts'].items():
        assert row['before'] == parent['new_dependencies'][path]
        assert m3_revision_digest(path, newer_than=M3_SUCCESSORS[8]) == row['after'] != row['before']
        with zipfile.ZipFile(ROOT/child['before_snapshot']) as archive:
            assert sha(archive.read(path)) == row['before']


def test_m4_new_race_only_extends_test_harness():
    child = load(M3_SUCCESSORS[9])
    assert child['parent_contract'] == M3_SUCCESSORS[8]
    assert sha((ROOT/M3_SUCCESSORS[8]).read_bytes()) == child['parent_contract_sha256']
    assert child['approved_changes'] == {}
    assert set(child['approved_evaluation_scripts']) == {'scripts/validate_g1_traffic.py',
        'scripts/m4_operation_trial.py', 'scripts/m4_operation_eval.py'}
    with zipfile.ZipFile(ROOT/child['before_snapshot']) as archive:
        for path,row in child['approved_evaluation_scripts'].items():
            assert sha(archive.read(path)) == row['before']
            assert m3_revision_digest(path, newer_than=M3_SUCCESSORS[9]) == row['after'] != row['before']


@pytest.mark.parametrize('index', [*range(10,18),
    *range(M3_SUCCESSORS.index('config/tests/signed_forward_r1_sha256.json'),len(M3_SUCCESSORS))])
def test_registered_oct26_and_nov01_successors_have_preserved_source_chain(index):
    child = load(M3_SUCCESSORS[index])
    assert child['parent_contract'] == M3_SUCCESSORS[index-1]
    assert sha((ROOT/child['parent_contract']).read_bytes()) == child['parent_contract_sha256']
    assert sha((ROOT/child['before_snapshot']).read_bytes()) == child['before_snapshot_sha256']
    with zipfile.ZipFile(ROOT/child['before_snapshot']) as archive:
        for group in GROUPS:
            for path, row in child[group].items():
                assert row['reason'].strip()
                assert sha(archive.read(path)) == row['before'] != row['after']
                assert m3_revision_digest(path, newer_than=M3_SUCCESSORS[index]) == row['after']
    for path, digest in child['new_dependencies'].items():
        assert m3_revision_digest(path, newer_than=M3_SUCCESSORS[index]) == digest
    if index in (11, 12):
        assert child['approved_changes'] == {}
        assert set(child['approved_evaluation_scripts']) == {'scripts/validate_g1_traffic.py'}
