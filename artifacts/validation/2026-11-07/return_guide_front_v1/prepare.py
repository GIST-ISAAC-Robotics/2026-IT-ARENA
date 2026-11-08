"""r9 현재 관측 복귀 경로 시작 조건과 동일 평탄 교통 대조 등록."""
import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import zipfile

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
OLD=HERE.parent/'traffic_surface_alignment_v1'
PARENT='config/tests/paired_prefix_r8_sha256.json'
TARGET='config/tests/return_guide_front_r9_sha256.json'

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def contract():
    with zipfile.ZipFile(HERE/'before.zip') as z:
        changes={p:dict(before=hashlib.sha256(z.read(p)).hexdigest(),after=sha(ROOT/p),
                       reason='복귀 중 현재 차체 앞부분에서 시작하는 관측 기준선 허용. 상대·이동 영역·점유 확인 불변') for p in z.namelist()}
    data=dict(schema_version=1,scope='moving_pass_trial 복귀 안내, 기본 OFF 유지',
        parent_contract=PARENT,parent_contract_sha256=sha(ROOT/PARENT),
        before_snapshot=(HERE/'before.zip').relative_to(ROOT).as_posix(),before_snapshot_sha256=sha(HERE/'before.zip'),
        approved_changes=changes,approved_evaluation_scripts={},new_dependencies={})
    target=ROOT/TARGET
    assert not target.exists()
    target.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    p=ROOT/'tests/test_source_provenance_r0_plus_b3.py';s=p.read_text(encoding='utf-8')
    anchor=f"M3_SUCCESSORS += ('{PARENT}',)"
    assert s.count(anchor)==1 and TARGET not in s
    p.write_text(s.replace(anchor,anchor+f"\nM3_SUCCESSORS += ('{TARGET}',)"),encoding='utf-8',newline='\n')

def scripts():
    for name in ('run.py','run.sh','assess.py'):
        s=(OLD/name).read_text(encoding='utf-8').replace('traffic_surface_alignment_v1','return_guide_front_v1')
        (HERE/name).write_text(s,encoding='utf-8',newline='\n')
    (HERE/'new_track_course.json').write_bytes((OLD/'new_track_course.json').read_bytes())

def register():
    case='straight_moving_lead';folder=HERE/case;folder.mkdir(exist_ok=False)
    old=json.loads((OLD/case/'registered.json').read_text(encoding='utf-8'))
    names=subprocess.check_output(['git','ls-files','-co','--exclude-standard','src','scripts','config'],cwd=ROOT,text=True).splitlines()
    files=sorted({p for p in names if (ROOT/p).is_file()}|{p.relative_to(ROOT).as_posix() for p in HERE.iterdir() if p.is_file()})
    with zipfile.ZipFile(folder/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in files:z.write(ROOT/p,p)
    data=dict(registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),contract=TARGET,
        scope='same r7 flat moving-lead scene once, rejoin reference start within current front body; r8 inactive in traffic',
        parent_registration=(OLD/case/'registered.json').relative_to(ROOT).as_posix(),args=old['args'],
        sources={p:sha(ROOT/p) for p in files},snapshot_sha256=sha(folder/'source_snapshot.zip'))
    (folder/'registered.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print('registered',len(files))

if __name__=='__main__':{'contract':contract,'scripts':scripts,'register':register}[sys.argv[1]]()
