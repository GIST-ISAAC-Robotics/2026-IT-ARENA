"""released_rear_sweep_r1의 후속 계약을 만든다. 기존 계약과 실패 기록은 고치지 않는다.

소스가 다시 바뀌면 이 스크립트로 이 계약 하나만 다시 생성한다.
"""
import hashlib
import json
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
PARENT = 'config/tests/released_rear_sweep_r1_sha256.json'
TARGET = ROOT/'config/tests/low_speed_arc_reacquire_r1_sha256.json'
BEFORE = HERE/'before.zip'
RUNTIME = ('src/arena_autonomy/arena_autonomy/local_follow.py', 'src/arena_autonomy/arena_autonomy/local_pursuit.py',
           'src/arena_bringup/launch/simulation.launch.py')
EVALUATION = ('scripts/validate_basic_autonomy.py',)
REASON = '결합 단독 실행의 정지 뒤 미재개를 선택형 낮은 측정 속도 양측 호길이 재획득으로 비교한다. 측정값은 실제 정지를 증명하지 않는다.'


def sha(data):
    return hashlib.sha256(data).hexdigest()


def rows(paths, archive):
    out = {}
    for path in paths:
        before, after = sha(archive.read(path)), sha((ROOT/path).read_bytes())
        if before == after:
            raise ValueError(f'변경 없음: {path}')
        out[path] = dict(before=before, after=after, reason=REASON)
    return out


with zipfile.ZipFile(BEFORE) as archive:
    contract = dict(
        schema_version=1,
        scope='선택형 단독 프리랩 낮은 측정 속도 양측 호길이 재획득. 기존 허용치·기억·점유·독립 보호·STOP·기본 OFF 보존. 교통 구성 미연결.',
        parent_contract=PARENT, parent_contract_sha256=sha((ROOT/PARENT).read_bytes()),
        before_snapshot=BEFORE.relative_to(ROOT).as_posix(), before_snapshot_sha256=sha(BEFORE.read_bytes()),
        approved_changes=rows(RUNTIME, archive), approved_evaluation_scripts=rows(EVALUATION, archive), new_dependencies={})
TARGET.write_bytes((json.dumps(contract, ensure_ascii=False, indent=2)+'\n').encode('utf-8'))
print(TARGET.relative_to(ROOT), sha(TARGET.read_bytes()))
