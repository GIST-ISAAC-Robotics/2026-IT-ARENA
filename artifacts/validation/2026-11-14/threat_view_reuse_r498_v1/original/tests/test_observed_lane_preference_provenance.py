"""관측 직선 선호의 이전 소스와 명시 후속 연결을 함께 확인한다."""
import hashlib
import json
from pathlib import Path
import zipfile

from test_source_provenance_r0_plus_b3 import m3_revision_digest

NAME = 'config/tests/observed_lane_preference_r1_sha256.json'


def test_observed_lane_preference_contract_preserves_original_and_current():
    root = Path(__file__).resolve().parents[1]
    sha = lambda b: hashlib.sha256(b).hexdigest()
    contract = json.loads((root/'config/tests/observed_lane_preference_r1_sha256.json').read_text(encoding='utf-8'))
    assert contract['parent_contract'] == 'config/tests/rear_lane_keep_r1_sha256.json'
    assert sha((root/contract['parent_contract']).read_bytes()) == contract['parent_contract_sha256']
    snapshot = root/contract['before_snapshot']
    assert sha(snapshot.read_bytes()) == contract['before_snapshot_sha256']
    assert len(contract['approved_changes']) == 2 and len(contract['approved_evaluation_scripts']) == 4
    with zipfile.ZipFile(snapshot) as previous:
        for group in ('approved_changes', 'approved_evaluation_scripts'):
            for path, row in contract[group].items():
                assert row['reason'].strip()
                assert sha(previous.read(path)) == row['before'] != row['after']
                # 10/30 observed_measured_v1 후속 계약까지 되돌린 이 계약의 after 시점을 대조한다.
                assert m3_revision_digest(path, newer_than=NAME) == row['after']
    assert len(contract['new_dependencies']) == 2
    for path, digest in contract['new_dependencies'].items():
        assert sha((root/path).read_bytes()) == digest
