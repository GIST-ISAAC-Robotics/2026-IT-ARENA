import hashlib,json,zipfile
from pathlib import Path
from test_source_provenance_r0_plus_b3 import m3_revision_digest

def test_same_scan_history_preserves_preceding_source():
    root=Path(__file__).resolve().parents[1];name='config/tests/g1_same_scan_history_r1_sha256.json'
    c=json.loads((root/name).read_text(encoding='utf-8'));sha=lambda b:hashlib.sha256(b).hexdigest()
    assert c['parent_contract']=='config/tests/g2_initial_reference_r2_sha256.json'
    assert sha((root/c['parent_contract']).read_bytes())==c['parent_contract_sha256']
    assert sha((root/c['before_snapshot']).read_bytes())==c['before_snapshot_sha256']
    assert set(c['approved_changes'])=={'src/arena_autonomy/arena_autonomy/traffic_path_recovery_trial.py'}
    assert not c['approved_evaluation_scripts'] and not c['new_dependencies']
    with zipfile.ZipFile(root/c['before_snapshot']) as z:
        for path,row in c['approved_changes'].items():
            assert sha(z.read(path))==row['before']!=row['after']
            assert m3_revision_digest(path,newer_than=name)==row['after']
