"""후방 차선 유지 후보의 후속 출처 계약. HEAD(직전 게시 5f8ae55) 바이트를 이전 사본으로 보존한다.

과거 계약·ZIP·실행 결과를 고치지 않는다. 변경된 추적 파일만 before/after/이유로 남긴다.
"""
import hashlib, json, subprocess, sys, zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
BASE = Path(__file__).resolve().parent
OUT = ROOT/'config/tests/rear_lane_keep_r1_sha256.json'
SNAP = BASE/'before_rear_lane_keep.zip'
PARENT = 'config/tests/g2_handoff_manifest_r1_sha256.json'
REASON = '후방 소실 직후 차선 유지 선택형 후보(rear_lane_keep_trial, 기본 OFF) 연결.'
CHANGES = {
 'src/arena_autonomy/arena_autonomy/local_follow.py': 'FollowConfig 선택 플래그와 선택기 연결. '+REASON,
 'src/arena_autonomy/arena_autonomy/local_pursuit.py': 'ROS 매개변수 g1_rear_lane_keep_trial(기본 false) 연결. '+REASON}
SCRIPTS = {name: '새 정적 import 모듈 rear_lane_keep_trial을 보존/이관 목록에 추가. 제어 판정 변경 없음.' for name in (
 'scripts/validate_pipeline_faults_ros.py', 'scripts/prepare_jetson_bundle.py',
 'scripts/profile_follow_trace.py', 'scripts/validate_g1_traffic.py')}
NEW = ['src/arena_autonomy/arena_autonomy/rear_lane_keep_trial.py']
sha = lambda b: hashlib.sha256(b).hexdigest()

def head(path):
    return subprocess.run(['git', 'show', 'HEAD:'+path], cwd=ROOT, check=True, capture_output=True).stdout

if __name__ == '__main__':
    if OUT.exists() or SNAP.exists():
        sys.exit('contract or snapshot already exists')
    with zipfile.ZipFile(SNAP, 'x', zipfile.ZIP_DEFLATED) as archive:
        for path in [*CHANGES, *SCRIPTS]:
            archive.writestr(path, head(path))
    rows = lambda group: {p: {'before': sha(head(p)), 'after': sha((ROOT/p).read_bytes()), 'reason': r} for p, r in group.items()}
    contract = {'schema_version': 1,
        'scope': '후방 소실 직후 차선 유지 후보의 연결과 보존 목록. 기본 OFF·운영 채택 아님. 합성 비교 결과는 rear_reapproach_v1 참고.',
        'parent_contract': PARENT, 'parent_contract_sha256': sha((ROOT/PARENT).read_bytes()),
        'before_snapshot': SNAP.relative_to(ROOT).as_posix(), 'before_snapshot_sha256': sha(SNAP.read_bytes()),
        'before_source': {'commit': subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT, check=True, capture_output=True, text=True).stdout.strip(),
                          'scope': '직전 main 게시 바이트. 이후 Gazebo 실행 소스 사본은 없음'},
        'approved_changes': rows(CHANGES), 'approved_evaluation_scripts': rows(SCRIPTS),
        'new_dependencies': {p: sha((ROOT/p).read_bytes()) for p in NEW}}
    for group in ('approved_changes', 'approved_evaluation_scripts'):
        for p, row in contract[group].items():
            assert row['before'] != row['after'], p
    OUT.write_bytes((json.dumps(contract, ensure_ascii=False, indent=1)+'\n').encode('utf-8'))
    print(json.dumps({'contract': str(OUT.relative_to(ROOT)), 'changes': len(CHANGES), 'scripts': len(SCRIPTS)}))
