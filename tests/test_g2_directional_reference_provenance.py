import ast,hashlib,json,zipfile
from pathlib import Path
from test_source_provenance_r0_plus_b3 import m3_revision_digest


def test_directional_contract_preserves_v1():
    root=Path(__file__).resolve().parents[1];name='config/tests/g2_directional_reference_r1_sha256.json'
    c=json.loads((root/name).read_text(encoding='utf-8'));sha=lambda b:hashlib.sha256(b).hexdigest()
    assert c['parent_contract']=='config/tests/g1_same_scan_history_r1_sha256.json'
    assert sha((root/c['parent_contract']).read_bytes())==c['parent_contract_sha256']
    assert sha((root/c['before_snapshot']).read_bytes())==c['before_snapshot_sha256']
    assert len(c['approved_changes'])==2 and not c['new_dependencies']
    with zipfile.ZipFile(root/c['before_snapshot']) as z:
        for path,row in c['approved_changes'].items():
            assert sha(z.read(path))==row['before']!=row['after']
            assert m3_revision_digest(path,newer_than=name)==row['after']
        name='src/arena_autonomy/arena_autonomy/bypass_initial_reference.py'
        def body(source,cls):
            return ast.dump(next(n for n in ast.parse(source).body if isinstance(n,ast.ClassDef) and n.name==cls))
        # 10/31의 클래스 대조는 그 뒤 계산 공유 변경 전 보존본에 대해 수행한다.
        later=json.loads((root/'config/tests/same_call_reference_r1_sha256.json').read_text(encoding='utf-8'))
        with zipfile.ZipFile(root/later['before_snapshot']) as preserved:
            current=preserved.read(name).decode('utf-8')
        assert sha(current.encode('utf-8'))==m3_revision_digest(name,newer_than='config/tests/g2_directional_reference_r1_sha256.json')
        assert body(z.read(name),'CurrentPairReference')==body(current,'CurrentPairReference')
    candidate=root/'artifacts/validation/2026-10-31/g2_directional_reference_v1/candidate.py'
    assert body(candidate.read_text(encoding='utf-8'),'DirectionalPairReference')==body(current,'DirectionalPairReference')
