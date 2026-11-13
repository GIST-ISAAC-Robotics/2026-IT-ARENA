"""후방 차선 유지 후보의 후속 출처 계약. 주행 성공 판정은 별도다."""
import hashlib
import json
from pathlib import Path
import zipfile

from test_source_provenance_r0_plus_b3 import m3_revision_digest

ROOT = Path(__file__).resolve().parents[1]
NAME = 'config/tests/rear_lane_keep_r1_sha256.json'


def test_rear_lane_keep_successor_preserves_previous_bytes_and_current_sources():
    sha = lambda data: hashlib.sha256(data).hexdigest()
    child = json.loads((ROOT/NAME).read_text(encoding='utf-8'))
    assert child['parent_contract'] == 'config/tests/g2_handoff_manifest_r1_sha256.json'
    assert sha((ROOT/child['parent_contract']).read_bytes()) == child['parent_contract_sha256']
    snapshot = ROOT/child['before_snapshot']
    assert sha(snapshot.read_bytes()) == child['before_snapshot_sha256']
    assert len(child['approved_changes']) == 2 and len(child['approved_evaluation_scripts']) == 4
    with zipfile.ZipFile(snapshot) as previous:
        for group in ('approved_changes', 'approved_evaluation_scripts'):
            for path, row in child[group].items():
                assert row['reason'].strip()
                assert sha(previous.read(path)) == row['before'] != row['after']
                assert m3_revision_digest(path, newer_than=NAME) == row['after']
                # 현재 파일은 위 m3_revision_digest가 후속 계약과 함께 검증한다.
    assert list(child['new_dependencies']) == ['src/arena_autonomy/arena_autonomy/rear_lane_keep_trial.py']
    for path, digest in child['new_dependencies'].items():
        assert sha((ROOT/path).read_bytes()) == digest
