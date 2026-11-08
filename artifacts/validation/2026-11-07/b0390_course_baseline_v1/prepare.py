"""B0390 후속 출처 계약과 첫 주행 소스 등록. 기존 등록은 덮지 않는다."""
import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import zipfile

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
REL=HERE.relative_to(ROOT).as_posix()
SENSOR='src/arena_description/config/b0390_c1_jr60p20_sim.yaml'
PARENT='config/tests/track_migration_v2026_11_06_r1_sha256.json'
TARGET=ROOT/'config/tests/b0390_migration_r2_sha256.json'
def sha(data): return hashlib.sha256(data).hexdigest()
def write_once(path,data):
    text=json.dumps(data,ensure_ascii=False,indent=2)+'\n'
    if path.exists(): assert path.read_text(encoding='utf8')==text, str(path)
    else: path.write_text(text,encoding='utf8',newline='\n')

if __name__=='__main__':
    snapshot=HERE/'before_camera_migration.zip'
    with zipfile.ZipFile(snapshot) as z:
        row=lambda p,reason: dict(before=sha(z.read(p)),after=sha((ROOT/p).read_bytes()),reason=reason)
        contract=dict(schema_version=1,scope='B0390 선택형 카메라 프로필과 명시 새 코스 단독 실행. 기존 기본 인자·조향 정책 유지.',
            parent_contract=PARENT,parent_contract_sha256=sha((ROOT/PARENT).read_bytes()),
            before_snapshot=snapshot.relative_to(ROOT).as_posix(),before_snapshot_sha256=sha(snapshot.read_bytes()),
            approved_changes={'src/arena_bringup/launch/simulation.launch.py':row('src/arena_bringup/launch/simulation.launch.py','카메라·IMU 로그에 선택 프로필 모델명 사용. 동작 변경 없음')},
            approved_evaluation_scripts={'scripts/run_current_solo.py':row('scripts/run_current_solo.py','--track/--local-sensor-config 명시 선택 전달. 기존 인자 생략 재현 유지')},
            new_dependencies={SENSOR:sha((ROOT/SENSOR).read_bytes())})
    write_once(TARGET,contract)
    if '--register' not in sys.argv: raise SystemExit(0)
    registered=HERE/'registered.json'
    assert not registered.exists(),'이미 등록됨. 덮어쓰지 않는다.'
    base=['scripts/run_current_solo.py',f'{REL}/run1','--track','official_v2026_11_06','--local-sensor-config',SENSOR]
    args=json.loads(subprocess.check_output([sys.executable,'-B',*base,'--print-args'],cwd=ROOT,text=True))
    listed=subprocess.check_output(['git','ls-files','-co','--exclude-standard','src','scripts','config'],cwd=ROOT,text=True).splitlines()
    files=sorted(set(p for p in listed if (ROOT/p).is_file()) | {f'{REL}/prepare.py',f'{REL}/run.sh'})
    hashes={p:sha((ROOT/p).read_bytes()) for p in files}
    with zipfile.ZipFile(HERE/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in files: z.write(ROOT/p,p)
    write_once(registered,dict(registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        scope='신규 v2026.11.06+B0390 단독 기준 1회, 새 코너·출발 실패 파악; 지름길/고속 후보 기본 미선택',
        entry=base,args=args,input_sha256=hashes,snapshot_sha256=sha((HERE/'source_snapshot.zip').read_bytes())))
    print(f'registered {len(files)} inputs')
