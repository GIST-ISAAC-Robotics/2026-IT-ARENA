"""r15와 같은 CLI(설정 경로만 오목 설정으로 교체) 14초 실행 등록. 현재 작업트리(r42) 소스를 해시·snapshot한다."""
import datetime
import hashlib
import json
import subprocess
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
R15 = HERE.parent/'medium_pass_connected_v1'
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()

r15 = json.loads((R15/'straight_moving_lead/registered.json').read_text(encoding='utf-8'))
args = list(r15['args'])
i = args.index('--config')+1
assert args[i] == 'artifacts/validation/2026-11-08/medium_pass_connected_v1/new_track_course.json'
args[i] = (HERE/'new_track_course.json').relative_to(ROOT).as_posix()
assert [a for k, a in enumerate(args) if k != i] == [a for k, a in enumerate(r15['args']) if k != i]

folder = HERE/'straight_moving_lead'
folder.mkdir(exist_ok=False)
files = set(subprocess.check_output(['git', 'ls-files', '-co', '--exclude-standard', 'src', 'scripts', 'config'],
                                    cwd=ROOT, text=True).splitlines())
helpers = ('register.py', 'run.py', 'run.sh', 'assess.py', 'new_track_course.json', 'outline_moving.py',
           'static_sdf_check.py', 'check_runtime.py', 'make_config.py')
files = sorted({p for p in files if (ROOT/p).is_file()} | {(HERE/p).relative_to(ROOT).as_posix() for p in helpers})
with zipfile.ZipFile(folder/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as z:
    for p in files:
        z.write(ROOT/p, p)
sources = {p: sha(ROOT/p) for p in files}
data = dict(registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
            scope='r15 medium_v1 same CLI/sensor/speed/14s; only actor.visual_outline=recessed_sides (visual only); '
                  'current r42 working tree, not r15 source',
            args=args, r15_registered_sha256=sha(R15/'straight_moving_lead/registered.json'),
            sources=sources, snapshot_sha256=sha(folder/'source_snapshot.zip'))
(folder/'registered.json').write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n', encoding='utf-8', newline='\n')

# r15 등록 대비 소스 차이(메타데이터 동일성은 주행 비교와 별개)
old = r15['sources']
delta = dict(r15_inputs=len(old), current_inputs=len(sources),
             changed=sorted(p for p in old if p in sources and old[p] != sources[p]),
             removed=sorted(p for p in old if p not in sources),
             added=sorted(p for p in sources if p not in old))
(HERE/'code_delta.json').write_text(json.dumps(delta, ensure_ascii=False, indent=1)+'\n', encoding='utf-8', newline='\n')
print('registered', len(files), 'changed', len(delta['changed']), 'added', len(delta['added']), 'removed', len(delta['removed']))
