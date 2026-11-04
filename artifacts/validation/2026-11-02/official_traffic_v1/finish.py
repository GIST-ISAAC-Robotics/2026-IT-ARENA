"""소스 후속 계약. 이미 있으면 다시 만들지 않는다."""
from hashlib import sha256
import json
from pathlib import Path
import subprocess
import zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
PARENT = 'config/tests/observed_forward_pace_r3_sha256.json'
OUT = ROOT/'config/tests/official_traffic_r1_sha256.json'
SNAPSHOT = HERE.parent/'official_forward_v1/a4/source_snapshot.zip'
REASON = '앞차 근접 정지 뒤 조향 가능한 등록 전이를 후보로 고르는 기본 OFF 선택과, 잔디 시작 합류 평가 장면. 점유·보호층·물성 유지.'
CHANGES = ('src/arena_autonomy/arena_autonomy/bypass_corridor_trial.py', 'src/arena_autonomy/arena_autonomy/bypass_entry_follow.py',
           'src/arena_autonomy/arena_autonomy/bypass_handoff_corridor_trial.py', 'src/arena_autonomy/arena_autonomy/g1_g2_node.py')
SCRIPTS = ('scripts/traffic_merge.py', 'scripts/prepare_jetson_bundle.py', 'scripts/profile_follow_trace.py')
HEAD_ONLY = SCRIPTS[1:]  # a4 스냅숏에 없고 이번 묶음 전까지 HEAD와 같다


def digest(data):
    return sha256(data).hexdigest()


def contract():
    before = HERE/'before.zip'
    with zipfile.ZipFile(SNAPSHOT) as source, zipfile.ZipFile(before, 'x', zipfile.ZIP_DEFLATED) as archive:
        for path in (*CHANGES, *SCRIPTS):
            data = (subprocess.run(['git', 'show', f'HEAD:{path}'], cwd=ROOT, check=True, stdout=subprocess.PIPE).stdout
                    if path in HEAD_ONLY else source.read(path))
            archive.writestr(zipfile.ZipInfo(path, (2026, 11, 2, 0, 0, 0)), data)
    with zipfile.ZipFile(before) as archive:
        row = lambda path: dict(before=digest(archive.read(path)), after=digest((ROOT/path).read_bytes()), reason=REASON)
        data = dict(schema_version=1, scope=REASON, parent_contract=PARENT, parent_contract_sha256=digest((ROOT/PARENT).read_bytes()),
                    before_snapshot=before.relative_to(ROOT).as_posix(), before_snapshot_sha256=digest(before.read_bytes()),
                    approved_changes={p: row(p) for p in CHANGES}, approved_evaluation_scripts={p: row(p) for p in SCRIPTS},
                    new_dependencies={})
    OUT.write_bytes((json.dumps(data, ensure_ascii=False, indent=2)+'\n').encode('utf-8'))


if __name__ == '__main__':
    contract()
