"""고정한 인자 1회 실행, 원시 결과·종료코드·입력 해시를 보존한다."""
from pathlib import Path
import hashlib,json,subprocess,sys,time
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
r=json.loads((A/'registered.json').read_bytes())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert not (A/'run1').exists() and not (A/'run1.exit').exists()
changed=[p for p,h in r['sources'].items() if sha(ROOT/p)!=h]
assert not changed,changed
start=time.monotonic()
with (A/'run1_console.log').open('w') as f:
    code=subprocess.call([sys.executable,'-B',*r['args']],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
(A/'run1.exit').write_text(f'ACTUAL_EXIT_CODE={code} WALL_S={time.monotonic()-start:.3f}\n')
changed=[p for p,h in r['sources'].items() if sha(ROOT/p)!=h]
(A/'postflight_integrity.json').write_text(json.dumps(dict(inputs=len(r['sources']),changed=changed),indent=2)+'\n')
print('validator_exit',code,'changed',changed)
sys.exit(code)
