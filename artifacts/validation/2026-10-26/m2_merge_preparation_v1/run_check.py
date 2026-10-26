"""제한 합류 실행 전후 소스·설치 의존성 확인. 실행/대기는 별도 담당."""
import hashlib
import importlib
import json
import os
from pathlib import Path
import sys
import zipfile

root = Path.cwd()
sys.path.insert(0, str(root/'scripts'))
import validate_g1_traffic as runner

out = Path(sys.argv[2]); phase = sys.argv[1]
digest = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
paths = runner.source_paths('config/tests/m2_merge_stop_restart_v1.json')
paths += ['tests/test_traffic_merge.py', 'scripts/replay_follow_trace.py', 'scripts/surface_contact_eval.py',
          'src/arena_autonomy/arena_autonomy/m3_guarded_return_trial.py',
          'src/arena_autonomy/arena_autonomy/m3_return_guard.py']
paths = sorted(set(paths))
current = {p: digest(root/p) for p in paths}
if phase == 'before':
    out.mkdir(parents=True, exist_ok=True)
    installed = {}
    # 제어기 소스는 바꾸지 않았으나 현재 install overlay가 같은 판본인지 읽기로 확인한다.
    for name in ('local_pursuit', 'follow_trace', 'observation_support', 'scan_topology',
                 'm3_execution_trial', 'm3_guarded_return_trial', 'race_session', 'lidar_safety'):
        module = importlib.import_module('arena_autonomy.'+name)
        path = Path(module.__file__)
        source = root/'src/arena_autonomy/arena_autonomy'/f'{name}.py'
        installed[name] = {'path': str(path), 'sha256': digest(path), 'matches_source': digest(path) == digest(source)}
    (out/'installed_modules.json').write_text(json.dumps(installed, indent=2)+'\n')
    if not all(v['matches_source'] for v in installed.values()):
        raise SystemExit('installed module/source mismatch')
    with zipfile.ZipFile(out/'execution_sources.zip', 'x', zipfile.ZIP_DEFLATED) as z:
        for p in paths: z.write(root/p, p)
    (out/'pre_run_hashes.json').write_text(json.dumps(current, indent=2)+'\n')
elif phase == 'after':
    old = json.loads((out/'pre_run_hashes.json').read_text())
    differences = {p: [old.get(p), current.get(p)] for p in old.keys() | current.keys() if old.get(p) != current.get(p)}
    (out/'source_hash_diff.json').write_text(json.dumps(differences, indent=2)+'\n')
    if differences: raise SystemExit('source changed during run')
else:
    raise SystemExit('unknown phase')

found = []
tokens = ('gz sim', 'ros2 launch', 'ros_gz_bridge', 'validate_g1_traffic.py', 'local_pursuit')
for p in Path('/proc').iterdir():
    if not p.name.isdigit() or int(p.name) == os.getpid(): continue
    try:
        cmd = (p/'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
        comm = (p/'comm').read_text().strip()
    except OSError:
        continue
    # 셸의 아직 실행하지 않은 명령 문자열을 시험 프로세스로 오인하지 않는다.
    if comm in ('bash', 'sh', 'timeout'): continue
    if any(t in cmd for t in tokens): found.append({'pid': int(p.name), 'comm': comm, 'cmd': cmd})
(out/(phase+'_processes.json')).write_text(json.dumps(found, indent=2)+'\n')
print(json.dumps({'phase': phase, 'sources': len(current), 'processes': found}))
if found: raise SystemExit('existing/residual trial processes; inspect ownership without killing unrelated work')
