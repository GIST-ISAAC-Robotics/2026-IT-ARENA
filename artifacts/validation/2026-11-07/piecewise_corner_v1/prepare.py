"""새 코스 본선 단면 매개변수 후속 계약과 동일 조건 주행 등록."""
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
PARENT='config/tests/course_section_r3_sha256.json'
TARGET=ROOT/'config/tests/piecewise_corner_r4_sha256.json'
def sha(data):return hashlib.sha256(data).hexdigest()
def once(path,data):
    text=json.dumps(data,ensure_ascii=False,indent=2)+'\n'
    if path.exists():assert path.read_text(encoding='utf8')==text,str(path)
    else:path.write_text(text,encoding='utf8',newline='\n')
if __name__=='__main__':
    snapshot=HERE/'before.zip'
    with zipfile.ZipFile(snapshot) as z:
        changes={p:dict(before=sha(z.read(p)),after=sha((ROOT/p).read_bytes()),
                        reason='기본 OFF 관측된 두 직선+접선 원호 경로 표현. 기존 경계·점유·독립 보호 유지') for p in z.namelist()}
    once(TARGET,dict(schema_version=1,scope='선택형 직각 경로. 지도·정답 제어 입력 없음, 보호 판정 유지',
        parent_contract=PARENT,parent_contract_sha256=sha((ROOT/PARENT).read_bytes()),
        before_snapshot=snapshot.relative_to(ROOT).as_posix(),before_snapshot_sha256=sha(snapshot.read_bytes()),
        approved_changes=changes,approved_evaluation_scripts={},new_dependencies={'src/arena_autonomy/arena_autonomy/piecewise_wall.py':sha((ROOT/'src/arena_autonomy/arena_autonomy/piecewise_wall.py').read_bytes())}))
    if '--register' not in sys.argv:raise SystemExit(0)
    assert not (HERE/'registered.json').exists(),'기존 등록 덮어쓰기 금지'
    entry=['scripts/run_current_solo.py',f'{REL}/run1','--track','official_v2026_11_06',
           '--local-sensor-config','src/arena_description/config/b0390_c1_jr60p20_sim.yaml','--piecewise-wall']
    args=json.loads(subprocess.check_output([sys.executable,'-B',*entry,'--print-args'],cwd=ROOT,text=True))
    names=subprocess.check_output(['git','ls-files','-co','--exclude-standard','src','scripts','config'],cwd=ROOT,text=True).splitlines()
    files=sorted({p for p in names if (ROOT/p).is_file()}|{f'{REL}/prepare.py',f'{REL}/run.sh'})
    with zipfile.ZipFile(HERE/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in files:z.write(ROOT/p,p)
    once(HERE/'registered.json',dict(registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        scope='B0390 새 코스 두 직선+원호 후보 대조 1회',entry=entry,args=args,
        input_sha256={p:sha((ROOT/p).read_bytes()) for p in files},snapshot_sha256=sha((HERE/'source_snapshot.zip').read_bytes())))
    print('registered',len(files))
