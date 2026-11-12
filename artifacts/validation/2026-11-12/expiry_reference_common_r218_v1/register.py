from pathlib import Path
import hashlib,json,zipfile
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
digest=lambda b:hashlib.sha256(b).hexdigest()
parent='config/tests/static_side_elapsed_r209_sha256.json'
changes={}
with zipfile.ZipFile(A/'before_sources.zip') as archive:
    for name in archive.namelist():
        changes[name]=dict(before=digest(archive.read(name)),after=digest((ROOT/name).read_bytes()),
            reason='r216 최근 경로/선택 벽 비교와 원천 만료 원인 전달을 기본 OFF 공통 선택으로 이관. 명령/STOP/기간/거리 불변.')
new='src/arena_autonomy/arena_autonomy/expiry_reference.py'
record=dict(schema_version=1,scope='r218 명시 만료 참조 연결. 기본 OFF, 평가 지연/실물 허가 없음.',
    parent_contract=parent,parent_contract_sha256=digest((ROOT/parent).read_bytes()),
    before_snapshot=(A/'before_sources.zip').relative_to(ROOT).as_posix(),
    before_snapshot_sha256=digest((A/'before_sources.zip').read_bytes()),
    approved_changes=changes,approved_evaluation_scripts={},new_dependencies={new:digest((ROOT/new).read_bytes())})
with (ROOT/'config/tests/expiry_reference_r218_sha256.json').open('x',encoding='utf8') as stream:
    json.dump(record,stream,ensure_ascii=False,indent=2);stream.write('\n')
print('Registered six changes and one module with preserved source chain.')
