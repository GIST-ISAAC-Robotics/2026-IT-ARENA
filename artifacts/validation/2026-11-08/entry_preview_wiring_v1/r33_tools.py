"""r33 평가 도구를 복사하지 않고 해시를 고정해 이 폴더 출력으로 실행한다. 평가 전용."""
import hashlib,json,sys
from pathlib import Path

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
R33=ROOT/'artifacts/validation/2026-11-08/ops_traffic_centered_entry_v1'
TOOLS=('run.py','assess.py','settle.py')
REGISTERED=json.loads((R33/'registered.json').read_text(encoding='utf-8'))['sources']
FLAG="'evaluation_stopped_entry_centering:=true',"
EXTRA="'evaluation_adaptive_entry_preview:=true',"
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()

def rel(name):return (R33/name).relative_to(ROOT).as_posix()

def verify():
    """r33 등록 당시와 같은 도구 바이트만 사용한다."""
    changed=[n for n in TOOLS if sha(R33/n)!=REGISTERED[rel(n)]]
    if changed:raise SystemExit('r33 tool changed since r33 registration: '+','.join(changed))

def execute(name,patch=None):
    """HERE를 이 폴더로 바꿔 r33 도구를 실행한다. patch는 정확히 한 곳만 바꾼다."""
    verify()
    source=(R33/name).read_text(encoding='utf-8')
    if patch:
        old,new=patch
        if source.count(old)!=1:raise SystemExit('r33 patch anchor not unique: '+old)
        source=source.replace(old,new,1)
    if str(R33) not in sys.path:sys.path.insert(1,str(R33))  # settle.py
    code=compile(source,str(R33/name),'exec')
    exec(code,{'__name__':'__main__','__file__':str(HERE/name)})

def launch_patch():return (FLAG,FLAG+EXTRA)

if __name__=='__main__':
    # --dry: 등록·실행 없이 r33 도구 해시, 단일 치환, 컴파일만 확인한다.
    import difflib
    verify()
    for name,patch in (('run.py',launch_patch()),('assess.py',None),('settle.py',None)):
        source=(R33/name).read_text(encoding='utf-8')
        patched=source.replace(*patch,1) if patch else source
        assert (source.count(patch[0])==1 if patch else True),name
        compile(patched,name,'exec')
        changed=[l for l in difflib.unified_diff(source.splitlines(),patched.splitlines(),lineterm='',n=0) if l[:1] in '+-' and l[:3] not in ('+++','---')]
        print(name,'sha256 matches r33 registration; changed lines:',changed)
