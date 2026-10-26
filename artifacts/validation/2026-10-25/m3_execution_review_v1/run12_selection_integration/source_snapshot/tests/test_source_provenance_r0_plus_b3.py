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
        assert row['reason'].strip() and sha((ROOT/path).read_bytes()) == row['after'] != row['before']
    for path, value in contract['new_dependencies'].items():
        assert sha((ROOT/path).read_bytes()) == value


def m3_revision_digest(path):
    successor = load('config/tests/m3_command_selection_r0_sha256.json')
    rows = {**successor['approved_changes'], **successor['approved_evaluation_scripts']}
    if path not in rows:
        return sha((ROOT/path).read_bytes())
    row = rows[path]
    assert sha((ROOT/path).read_bytes()) == row['after']
    snapshot = ROOT/successor['before_snapshot']
    if snapshot.is_file():
        with zipfile.ZipFile(snapshot) as archive:
            assert sha(archive.read(path)) == row['before']
    return row['before']


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
            assert sha((ROOT/path).read_bytes()) == row['after'] != row['before']
