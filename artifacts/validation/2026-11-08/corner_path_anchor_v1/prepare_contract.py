"""r22 출처 계약 생성: before.zip(수정 전) → 현재 두 소스 파일. 과거 계약은 수정하지 않는다."""
import hashlib, json, zipfile
from pathlib import Path
ROOT = Path(__file__).resolve().parents[4]
A = 'artifacts/validation/2026-11-08/corner_path_anchor_v1'
PARENT = 'config/tests/corner_path_budget_r21_sha256.json'
OUT = ROOT/'config/tests/corner_path_anchor_r22_sha256.json'
sha = lambda b: hashlib.sha256(b).hexdigest()
REASONS = {
    'src/arena_autonomy/arena_autonomy/local_path.py':
        '기본 OFF piecewise_corner 성공 반환에만 진단 path_anchor_error_m(차량 원점에서 반환 경로 선분까지 최소 거리) 추가. 경로·wall_distance_m·기본/직선/곡선 출력 불변',
    'src/arena_autonomy/arena_autonomy/local_follow.py':
        '앵커 오차 공통 helper anchor_error: piecewise_wall_trial이고 piecewise_corner 결과에 path_anchor_error_m가 있으면 그 값(비유한/음수는 거부), 그 밖은 기존 벽 거리 대조. 0.12·0.08·0.65·TTL·확인·점유·속도·STOP 불변',
}
with zipfile.ZipFile(ROOT/A/'before.zip') as z:
    changes = {p: {'before': sha(z.read(p)), 'after': sha((ROOT/p).read_bytes()), 'reason': r} for p, r in REASONS.items()}
assert all(r['before'] != r['after'] for r in changes.values())
contract = {
    'schema_version': 1,
    'scope': '기본 OFF 한 모서리 경로의 앵커 오차 계산 보완(코너 원호 위 벽 y거리 대신 생성 경로 거리); 기본 채택/실차 승인 아님',
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
