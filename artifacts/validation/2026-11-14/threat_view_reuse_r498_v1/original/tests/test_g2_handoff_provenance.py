"""G2 후속 소스와 이관 의존성 보존. 주행 성공 판정은 별도다."""
import hashlib
import json
from pathlib import Path
import zipfile

from test_source_provenance_r0_plus_b3 import m3_revision_digest

ROOT = Path(__file__).resolve().parents[1]
NAME = 'config/tests/g2_handoff_manifest_r1_sha256.json'


def test_g2_handoff_successor_preserves_previous_run_and_current_bytes():
    sha = lambda data: hashlib.sha256(data).hexdigest()
    child = json.loads((ROOT/NAME).read_text(encoding='utf-8'))
    assert child['parent_contract'] == 'config/tests/fresh_scan_wait_r1_sha256.json'
    assert sha((ROOT/child['parent_contract']).read_bytes()) == child['parent_contract_sha256']
    snapshot = ROOT/child['before_snapshot']
    assert sha(snapshot.read_bytes()) == child['before_snapshot_sha256']
    assert len(child['approved_changes']) == 3
    assert len(child['approved_evaluation_scripts']) == 5
    with zipfile.ZipFile(snapshot) as previous:
        for group in ('approved_changes', 'approved_evaluation_scripts'):
            for path, row in child[group].items():
                assert row['reason'].strip()
                assert sha(previous.read(path)) == row['before'] != row['after']
                assert m3_revision_digest(path, newer_than=NAME) == row['after']
    assert len(child['new_dependencies']) == 6
    for path, digest in child['new_dependencies'].items():
        assert m3_revision_digest(path, newer_than=NAME) == digest
