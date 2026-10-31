import ast,hashlib,json,zipfile
from pathlib import Path
from test_source_provenance_r0_plus_b3 import m3_revision_digest


def test_forward_same_path_preserves_old_source_and_matches_saved_candidate():
    root=Path(__file__).resolve().parents[1];name='config/tests/g1_forward_same_path_r1_sha256.json'
    c=json.loads((root/name).read_text(encoding='utf-8'));sha=lambda b:hashlib.sha256(b).hexdigest()
    assert c['parent_contract']=='config/tests/g2_directional_reference_r1_sha256.json'
    assert sha((root/c['parent_contract']).read_bytes())==c['parent_contract_sha256']
    assert sha((root/c['before_snapshot']).read_bytes())==c['before_snapshot_sha256']
    path='src/arena_autonomy/arena_autonomy/same_path_gap_trial.py'
    assert set(c['approved_changes'])=={path} and not c['new_dependencies']
    with zipfile.ZipFile(root/c['before_snapshot']) as z:
        row=c['approved_changes'][path]
        assert sha(z.read(path))==row['before']!=row['after']
        assert m3_revision_digest(path,newer_than=name)==row['after']
    def functions(source):
        return [ast.dump(n) for n in ast.parse(source).body if isinstance(n,ast.FunctionDef)]
    candidate=root/'artifacts/validation/2026-10-31/g1_forward_same_path_v1/candidate.py'
    assert functions(candidate.read_text(encoding='utf-8'))==functions((root/path).read_text(encoding='utf-8'))
