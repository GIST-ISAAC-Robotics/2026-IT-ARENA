"""새 변경만 이전 실행 사본과 대조해 후속 계약으로 기록한다."""
import hashlib
import json
from pathlib import Path
import zipfile
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
def digest(data):return hashlib.sha256(data).hexdigest()
paths={f'src/arena_autonomy/arena_autonomy/{name}.py':BASE/f'{name}_before.py'
       for name in ('follow_trace','partial_observation','traffic_recovery_trial')}
previous=BASE.parent/'traffic_path_recovery_connection_v1/run1_source_snapshot.zip'
with zipfile.ZipFile(previous) as archive:
    for path,before in paths.items():assert archive.read(path)==before.read_bytes(),path
out=BASE/'before_source.zip'
with zipfile.ZipFile(out,'x',zipfile.ZIP_DEFLATED) as archive:
    for path,before in paths.items():archive.writestr(path,before.read_bytes())
parent='config/tests/traffic_path_recovery_r0_sha256.json'
changes={path:dict(before=digest(before.read_bytes()),after=digest((ROOT/path).read_bytes()),
    reason=('명시한 표면 정합 진단 위치 두 곳의 벽시계 비용만 재생 비교에서 분리. 원시 기록 유지.'
        if path.endswith('follow_trace.py') else
        '진단 예시와 전체 위협 상태를 분리하고 원래 스캔 기준의 배경·위협점을 함께 보존. 기존 거리/속도/보호 기준 유지.'))
    for path,before in paths.items()}
contract=dict(schema_version=1,scope='선택형 교통 회복 위협 상태와 앵커 기준 수정. 기본 활성화/주행 안전 승인 아님',
    parent_contract=parent,parent_contract_sha256=digest((ROOT/parent).read_bytes()),
    before_snapshot=out.relative_to(ROOT).as_posix(),before_snapshot_sha256=digest(out.read_bytes()),
    approved_changes=changes,approved_evaluation_scripts={},new_dependencies={})
with (ROOT/'config/tests/traffic_anchor_set_r1_sha256.json').open('x',encoding='utf-8') as stream:
    json.dump(contract,stream,ensure_ascii=False,indent=2)
print(json.dumps({p:c['after'] for p,c in changes.items()}))
