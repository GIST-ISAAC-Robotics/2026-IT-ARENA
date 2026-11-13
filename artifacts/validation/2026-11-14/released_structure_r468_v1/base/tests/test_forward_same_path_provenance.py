import ast,hashlib,json,zipfile
from pathlib import Path
from test_source_provenance_r0_plus_b3 import m3_revision_digest, M3_SUCCESSORS


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
    # 비교 대상은 당시 승인된 함수다. 후속 수정이 있으면 그 수정 직전의
    # 보존 바이트를 읽고, 해당 계약/스냅숏 해시까지 확인한다.
    recorded=(root/path).read_bytes()
    for successor in M3_SUCCESSORS[M3_SUCCESSORS.index(name)+1:]:
        newer=json.loads((root/successor).read_bytes())
        if path not in newer.get('approved_changes',{}):
            continue
        snapshot=root/newer['before_snapshot']
        assert sha(snapshot.read_bytes())==newer['before_snapshot_sha256']
        with zipfile.ZipFile(snapshot) as archive:
            recorded=archive.read(path)
        assert sha(recorded)==newer['approved_changes'][path]['before']
        break
    assert sha(recorded)==row['after']
    assert functions(candidate.read_text(encoding='utf-8'))==functions(recorded.decode('utf-8'))
