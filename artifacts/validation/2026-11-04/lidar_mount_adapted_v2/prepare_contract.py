"""v1 뒤에 구조 가림 적응 출처 계약을 연결한다. 등록 후 덮어쓰지 않는다."""
from pathlib import Path
import hashlib
import json
import zipfile

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
PARENT='config/tests/lidar_mount_trial_r1_sha256.json'
TARGET=ROOT/'config/tests/lidar_mount_adapted_r2_sha256.json'
FILES={
 'src/arena_autonomy/arena_autonomy/partial_observation.py':'알려진 구조와 예기치 않은 결측 구분. 기본 경로 유지',
 'src/arena_autonomy/arena_autonomy/local_follow.py':'선택형 구조 그림자/숨은 반환 기억의 독립 평가',
 'src/arena_autonomy/arena_autonomy/local_pursuit.py':'구조 정책 노드 파라미터, 기본 off',
 'src/arena_autonomy/arena_autonomy/lidar_safety.py':'독립 보호층에서 같은 구조 정책 자체 계산',
 'src/arena_bringup/launch/simulation.launch.py':'structure_aware 프로필에서만 두 노드에 명시 선택 전달',
}
NEW='src/arena_autonomy/arena_autonomy/known_structure.py'
def sha(data):return hashlib.sha256(data).hexdigest()
def main():
 snapshot=HERE/'runtime_before.zip'
 with zipfile.ZipFile(snapshot) as z:
  rows={p:dict(before=sha(z.read(p)),after=sha((ROOT/p).read_bytes()),reason=reason) for p,reason in FILES.items()}
 result=dict(schema_version=1,scope='C/JR 동일 광축 구조 적응 선택 시험. 기본 채택·실차 승인 아님.',
  parent_contract=PARENT,parent_contract_sha256=sha((ROOT/PARENT).read_bytes()),
  before_snapshot=snapshot.relative_to(ROOT).as_posix(),before_snapshot_sha256=sha(snapshot.read_bytes()),
  approved_changes=rows,approved_evaluation_scripts={},new_dependencies={NEW:sha((ROOT/NEW).read_bytes())})
 content=json.dumps(result,ensure_ascii=False,indent=2)+'\n'
 if TARGET.exists(): assert TARGET.read_text(encoding='utf8')==content,'등록 이후 바이트 변경'
 else: TARGET.write_text(content,encoding='utf8',newline='\n')
 print(TARGET.relative_to(ROOT))
if __name__=='__main__':main()
