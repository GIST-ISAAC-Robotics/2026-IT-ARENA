"""r23 출처 계약 생성: before.zip(수정 전 r22 after) → 현재 두 소스 파일. 과거 계약은 수정하지 않는다."""
import hashlib, json, zipfile
from pathlib import Path
ROOT = Path(__file__).resolve().parents[4]
A = 'artifacts/validation/2026-11-08/corner_path_anchor_transition_v1'
PARENT = 'config/tests/corner_path_anchor_r22_sha256.json'
OUT = ROOT/'config/tests/corner_path_anchor_transition_r23_sha256.json'
sha = lambda b: hashlib.sha256(b).hexdigest()
REASONS = {
    'src/arena_autonomy/arena_autonomy/local_path.py':
        '기본 OFF 선택형 호출(piecewise_corner=True)의 일반 매개곡선 반환에도 같은 진단 path_anchor_error_m(원점에서 반환 경로 선분까지 최소 거리) 추가. 경로 계산·선분·wall_distance_m·벽 모델·기본 OFF 출력 불변',
    'src/arena_autonomy/arena_autonomy/local_follow.py':
        'anchor_error가 piecewise_wall_trial일 때 wall_model None(일반 곡선) 또는 piecewise_corner이고 필드가 있으면 경로 거리 사용. 다른 모델·필드 없음·OFF는 벽 거리 대조. 0.12·0.08·0.65·TTL·확인·점유·속도·STOP 불변',
}
parent = json.loads((ROOT/PARENT).read_text(encoding='utf-8'))
with zipfile.ZipFile(ROOT/A/'before.zip') as z:
    changes = {p: {'before': sha(z.read(p)), 'after': sha((ROOT/p).read_bytes()), 'reason': r} for p, r in REASONS.items()}
    assert sha(z.read(PARENT)) == sha((ROOT/PARENT).read_bytes())
assert all(r['before'] != r['after'] for r in changes.values())
assert all(changes[p]['before'] == parent['approved_changes'][p]['after'] for p in REASONS)
contract = {
    'schema_version': 1,
    'scope': '기본 OFF 선택형 경로의 표현 전환(한 모서리↔일반 곡선) 시 앵커 지표 적용 누락 보완; 기본 채택/실차 승인 아님',
    'parent_contract': PARENT,
    'parent_contract_sha256': sha((ROOT/PARENT).read_bytes()),
    'before_snapshot': f'{A}/before.zip',
    'before_snapshot_sha256': sha((ROOT/A/'before.zip').read_bytes()),
    'approved_changes': changes,
    'approved_evaluation_scripts': {},
    'new_dependencies': {},
}
with open(OUT, 'w', encoding='utf-8', newline='\n') as f:
    f.write(json.dumps(contract, indent=2, ensure_ascii=False)+'\n')
print(json.dumps(contract, indent=1, ensure_ascii=False))
