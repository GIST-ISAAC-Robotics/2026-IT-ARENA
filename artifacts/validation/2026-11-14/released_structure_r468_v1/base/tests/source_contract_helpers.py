"""과거 소스 계약과 명시 후속 변경의 연결. 과거 JSON/해시를 덮어쓰지 않는다."""
import hashlib
import json
import zipfile


RECOVERY_CONTRACTS = ('fresh_confirmation_r2_sha256.json', 'measured_lane_preference_r1_sha256.json', 'observed_lane_preference_r1_sha256.json', 'rear_lane_keep_r1_sha256.json', 'g2_handoff_manifest_r1_sha256.json', 'fresh_scan_wait_r1_sha256.json', 'rolling_clearance_r1_sha256.json', 'small_alignment_r1_sha256.json', 'actor_tail_capture_r1_sha256.json', 'rotation_reobservation_r1_sha256.json', 'short_prefix_origin_r1_sha256.json', 'traffic_manifest_r1_sha256.json', 'traffic_side_scope_r1_sha256.json', 'traffic_positive_bundle_r1_sha256.json', 'traffic_positive_clearance_r1_sha256.json', 'traffic_speed_recheck_r1_sha256.json', 'traffic_anchor_set_r1_sha256.json', 'traffic_path_recovery_r0_sha256.json', 'traffic_recovery_release_fringe_r1_sha256.json', 'traffic_recovery_trial_r0_sha256.json')

def hash_before_observation_extension(root, path):
    """새 보호층 bytes까지 확인한 뒤 이전 계약 시점의 값을 반환한다.

    현재 hash를 검사하지 않고 과거 값을 돌려주는 우회가 아니다. 새 관측 계약과
    명시 모터 카운트 시험의 launch 변경만 검증해 되돌리며 다른 파일은 실제 hash다.
    """
    current=hashlib.sha256((root/path).read_bytes()).hexdigest()
    for name in RECOVERY_CONTRACTS:
        recovery=json.loads((root/'config/tests'/name).read_text(encoding='utf-8'))
        change={**recovery['approved_changes'],**recovery['approved_evaluation_scripts']}.get(path)
        if change:
            assert hashlib.sha256((root/recovery['parent_contract']).read_bytes()).hexdigest()==recovery['parent_contract_sha256']
            assert current==change['after'],path
            assert hashlib.sha256((root/recovery['before_snapshot']).read_bytes()).hexdigest()==recovery['before_snapshot_sha256']
            with zipfile.ZipFile(root/recovery['before_snapshot']) as archive:
                assert hashlib.sha256(archive.read(path)).hexdigest()==change['before']
            current=change['before']
    partial=root/'config/tests/partial_observation_r0_sha256.json'
    if partial.exists():
        extension=json.loads(partial.read_text())
        change=extension['approved_changes'].get(path)
        if change:
            assert hashlib.sha256((root/extension['parent_contract']).read_bytes()).hexdigest()==extension['parent_contract_sha256']
            assert current==change['after'],path
            with zipfile.ZipFile(root/extension['before_snapshot']) as archive:
                assert hashlib.sha256(archive.read(path)).hexdigest()==change['before']
            current=change['before']
    if path == 'src/arena_bringup/launch/simulation.launch.py':
        display=json.loads((root/'config/tests/m5_publication_r0_sha256.json').read_text())
        assert hashlib.sha256((root/display['parent_contract']).read_bytes()).hexdigest()==display['parent_contract_sha256']
        change=display['approved_changes'][path]
        assert current==change['after'], path
        with zipfile.ZipFile(root/display['before_snapshot']) as archive:
            assert hashlib.sha256(archive.read(path)).hexdigest()==change['before']
        current=change['before']
        counts=json.loads((root/'config/tests/m2_count_fault_r0_sha256.json').read_text())
        assert hashlib.sha256((root/counts['parent_contract']).read_bytes()).hexdigest()==counts['parent_contract_sha256']
        change=counts['approved_changes'][path]
        assert current==change['after'], path
        snapshot=root/counts['before_snapshot']
        if snapshot.is_file():
            assert hashlib.sha256(snapshot.read_bytes()).hexdigest()==counts['before_snapshot_sha256']
            with zipfile.ZipFile(snapshot) as archive:
                assert hashlib.sha256(archive.read(path)).hexdigest()==change['before']
        current=change['before']
    contract=json.loads((root/'config/tests/recent_observation_r0_sha256.json').read_text())
    change=contract['newly_tracked_changes'].get(path)
    if change is None:
        return current
    parent=root/contract['parent_contract']
    assert hashlib.sha256(parent.read_bytes()).hexdigest()==contract['parent_contract_sha256']
    assert current==change['after'], path
    return change['before']
