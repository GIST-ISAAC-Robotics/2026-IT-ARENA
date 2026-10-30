"""observed_measured_v1 연결의 후속 출처 계약. 주행 성공 판정은 별도다."""
import hashlib
import json
from pathlib import Path
import zipfile

from test_source_provenance_r0_plus_b3 import m3_revision_digest

ROOT = Path(__file__).resolve().parents[1]
NAME = 'config/tests/measured_lane_preference_r1_sha256.json'


def test_measured_lane_preference_contract_preserves_previous_bytes_and_offline_origin():
    sha = lambda data: hashlib.sha256(data).hexdigest()
    child = json.loads((ROOT/NAME).read_text(encoding='utf-8'))
    assert child['parent_contract'] == 'config/tests/observed_lane_preference_r1_sha256.json'
    assert sha((ROOT/child['parent_contract']).read_bytes()) == child['parent_contract_sha256']
    snapshot = ROOT/child['before_snapshot']
    assert sha(snapshot.read_bytes()) == child['before_snapshot_sha256']
    assert list(child['approved_changes']) == ['src/arena_autonomy/arena_autonomy/local_follow.py']
    assert len(child['approved_evaluation_scripts']) == 4
    with zipfile.ZipFile(snapshot) as previous:
        for group in ('approved_changes', 'approved_evaluation_scripts'):
            for path, row in child[group].items():
                assert row['reason'].strip()
                assert sha(previous.read(path)) == row['before'] != row['after']
                assert m3_revision_digest(path, newer_than=NAME) == row['after']
        # local_pursuit.py는 정책 문자열을 그대로 전달하므로 이번 연결에서 바뀌지 않았다.
        path = 'src/arena_autonomy/arena_autonomy/local_pursuit.py'
        assert sha(previous.read(path)) == sha((ROOT/path).read_bytes())
    assert sorted(child['new_dependencies']) == [
        f'src/arena_autonomy/arena_autonomy/{name}.py' for name in
        ('measured_lane_preference_trial', 'measured_parallel_speed_trial', 'measured_parallel_trial')]
    for path, digest in child['new_dependencies'].items():
        assert sha((ROOT/path).read_bytes()) == digest
    # 10/29 오프라인 v4 원본은 수정하지 않고 그대로 보존한다.
    for path, digest in child['offline_origin'].items():
        assert sha((ROOT/path).read_bytes()) == digest
    archive = child['offline_origin_archive']
    assert sha((ROOT/archive['path']).read_bytes()) == archive['sha256']
    with zipfile.ZipFile(ROOT/archive['path']) as offline:
        for path, digest in child['offline_origin'].items():
            assert sha(offline.read(path)) == digest
