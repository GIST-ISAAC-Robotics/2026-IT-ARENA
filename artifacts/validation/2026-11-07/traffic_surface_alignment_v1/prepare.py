"""r7 시험용 단차 옵션 계약·같은 교통 장면의 단일 대조 등록."""
import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
OLD = HERE.parent/'traffic_course_migration_v1'
PARENT = 'config/tests/handoff_track_guard_r6_sha256.json'
TARGET = 'config/tests/grass_alignment_r7_sha256.json'
LAUNCH = 'src/arena_bringup/launch/simulation.launch.py'
HARNESS = 'scripts/validate_g1_traffic.py'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def contract():
    with zipfile.ZipFile(HERE/'before.zip') as z:
        changes = {p: dict(before=hashlib.sha256(z.read(p)).hexdigest(), after=sha(ROOT/p),
                           reason='default original; explicit v11.06 road_level grass height sensitivity only')
                   for p in (LAUNCH, HARNESS)}
    obj = dict(schema_version=1, scope='명시 시험 사본 잔디 1 mm 상승, 기본 원본·제어기·마찰·구동·STOP 불변',
               parent_contract=PARENT, parent_contract_sha256=sha(ROOT/PARENT),
               before_snapshot=(HERE/'before.zip').relative_to(ROOT).as_posix(),
               before_snapshot_sha256=sha(HERE/'before.zip'), approved_changes={LAUNCH:changes[LAUNCH]},
               approved_evaluation_scripts={HARNESS:changes[HARNESS]}, new_dependencies={})
    (ROOT/TARGET).write_text(json.dumps(obj,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    p=ROOT/'tests/test_source_provenance_r0_plus_b3.py'
    s=p.read_text(encoding='utf-8')
    anchor=f"M3_SUCCESSORS += ('{PARENT}',)"
    assert s.count(anchor)==1 and TARGET not in s
    p.write_text(s.replace(anchor,anchor+f"\nM3_SUCCESSORS += ('{TARGET}',)"),encoding='utf-8',newline='\n')


def scripts():
    for name in ('run.py','run.sh','assess.py'):
        s=(OLD/'r6'/name).read_text(encoding='utf-8')
        s=s.replace('HERE.parents[4]','HERE.parents[3]')
        s=s.replace('/../../../../..','/../../../..')
        s=s.replace('traffic_course_migration_v1/r6','traffic_surface_alignment_v1')
        s=s.replace("HERE.parent/'new_track_course.json'","HERE/'new_track_course.json'")
        (HERE/name).write_text(s,encoding='utf-8',newline='\n')
    (HERE/'new_track_course.json').write_bytes((OLD/'new_track_course.json').read_bytes())


def register():
    case='straight_moving_lead'
    folder=HERE/case
    folder.mkdir(exist_ok=False)
    old=json.loads((OLD/'r6'/case/'registered.json').read_text(encoding='utf-8'))
    args=old['args']+['--grass-alignment','road_level']
    names=subprocess.check_output(['git','ls-files','-co','--exclude-standard','src','scripts','config'],cwd=ROOT,text=True).splitlines()
    files=sorted({p for p in names if (ROOT/p).is_file()} |
                 {p.relative_to(ROOT).as_posix() for p in HERE.iterdir() if p.is_file()} |
                 {str(OLD.relative_to(ROOT)/'new_track_course.json').replace('\\','/')})
    with zipfile.ZipFile(folder/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
        for path in files:z.write(ROOT/path,path)
    obj=dict(registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),contract=TARGET,
             scope='same r6 failed moving-pass scene once; only 1 mm grass height removed; not adopted baseline',
             parent_registration=(OLD/'r6'/case/'registered.json').relative_to(ROOT).as_posix(),
             args=args,sources={p:sha(ROOT/p) for p in files},snapshot_sha256=sha(folder/'source_snapshot.zip'))
    (folder/'registered.json').write_text(json.dumps(obj,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print('registered',len(files))


if __name__=='__main__':
    {'contract':contract,'scripts':scripts,'register':register}[sys.argv[1]]()
