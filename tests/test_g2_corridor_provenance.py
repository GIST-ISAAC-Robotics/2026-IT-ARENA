"""G2 통로 명시 연결의 출처를 확인한다. 주행 성공 판정은 별도다."""
import hashlib
import json
from pathlib import Path
import zipfile

from test_source_provenance_r0_plus_b3 import m3_revision_digest

ROOT = Path(__file__).resolve().parents[1]
NAME = 'config/tests/g2_corridor_r1_sha256.json'


def test_corridor_successor_preserves_previous_bytes():
    sha = lambda data: hashlib.sha256(data).hexdigest()
    contract = json.loads((ROOT/NAME).read_text(encoding='utf-8'))
    assert contract['parent_contract'] == 'config/tests/fresh_confirmation_r2_sha256.json'
    assert sha((ROOT/contract['parent_contract']).read_bytes()) == contract['parent_contract_sha256']
    assert sha((ROOT/contract['before_snapshot']).read_bytes()) == contract['before_snapshot_sha256']
    assert len(contract['approved_changes']) == 3 and not contract['approved_evaluation_scripts']
    with zipfile.ZipFile(ROOT/contract['before_snapshot']) as archive:
        for path, row in contract['approved_changes'].items():
            assert row['reason'].strip()
            assert sha(archive.read(path)) == row['before'] != row['after']
            assert m3_revision_digest(path, newer_than=NAME) == row['after']
    for path, digest in contract['new_dependencies'].items():
        assert sha((ROOT/path).read_bytes()) == digest
