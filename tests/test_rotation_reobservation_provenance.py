"""회전 오차 보완의 현재 바이트·실패 당시 소스·부모 사슬을 함께 확인한다."""
import hashlib
import json
from pathlib import Path
import zipfile
from test_source_provenance_r0_plus_b3 import m3_revision_digest


def test_revision_preserves_original_failure_and_complete_new_dependency():
    root = Path(__file__).resolve().parents[1]
    sha = lambda data: hashlib.sha256(data).hexdigest()
    contract = json.loads((root/'config/tests/rotation_reobservation_r1_sha256.json').read_text(encoding='utf-8'))
    assert contract['parent_contract'] == 'config/tests/short_prefix_origin_r1_sha256.json'
    assert sha((root/contract['parent_contract']).read_bytes()) == contract['parent_contract_sha256']
    source = root/contract['before_snapshot']
    assert sha(source.read_bytes()) == contract['before_snapshot_sha256']
    assert len(contract['approved_changes']) == 4
    assert len(contract['approved_evaluation_scripts']) == 5
    with zipfile.ZipFile(source) as archive:
        for group in ('approved_changes', 'approved_evaluation_scripts'):
            for path, item in contract[group].items():
                assert item['reason'].strip()
                assert sha(archive.read(path)) == item['before']
                assert m3_revision_digest(path, newer_than='config/tests/rotation_reobservation_r1_sha256.json') == item['after'] != item['before']
    assert set(contract['new_dependencies']) == {'src/arena_autonomy/arena_autonomy/surface_reobservation.py'}
    for path, value in contract['new_dependencies'].items():
        assert m3_revision_digest(path, newer_than='config/tests/rotation_reobservation_r1_sha256.json') == value
