"""실행 직전: 실제로 import/launch되는 경로가 등록한 src 바이트를 가리키는지 확인한다(r41 check_runtime.py 방식).

r41은 고정 모듈 목록이었다. 여기서는 두 Python 패키지의 모든 모듈을 import 없이 find_spec으로 해석하고,
주요 노드 모듈은 실제 import한다. 결과는 straight_moving_lead/runtime_check.json. 사용: check_runtime.py [--dry]
"""
import hashlib
import importlib
import importlib.util
import json
import os
import shutil
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
PACKAGES = ('arena_autonomy', 'arena_vehicle_interface')
IMPORTED = ('arena_vehicle_interface.rotating_lidar', 'arena_vehicle_interface.actuation_ros',
            'arena_autonomy.lidar_safety', 'arena_autonomy.local_pursuit', 'arena_autonomy.g1_g2_node',
            'arena_autonomy.local_follow', 'arena_autonomy.moving_pass', 'arena_autonomy.piecewise_wall')
SHARE = {'arena_bringup': ('launch/g1_follow.launch.py', 'launch/simulation.launch.py', 'launch/demo.launch.py'),
         'arena_description': ('config/b0390_c1_jr60p20_sim.yaml', 'config/vehicle.yaml')}
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main(dry):
    if dry:
        sources = {p.relative_to(ROOT).as_posix(): sha(p) for p in (ROOT/'src').rglob('*.py') if p.is_file()}
        sources.update({f'src/{k}/{f}': sha(ROOT/'src'/k/f) for k, fs in SHARE.items() for f in fs})
    else:
        sources = json.loads((HERE/'straight_moving_lead/registered.json').read_text(encoding='utf-8'))['sources']
    rows, ok = [], True
    for package in PACKAGES:
        for path in sorted((ROOT/'src'/package/package).glob('*.py')):
            name = f'{package}.{path.stem}' if path.stem != '__init__' else package
            spec = importlib.util.find_spec(name)
            resolved = Path(spec.origin).resolve()
            rel = path.relative_to(ROOT).as_posix()
            good = rel in sources and sha(resolved) == sources[rel]
            rows.append(dict(kind='module_spec', name=name, resolved=str(resolved), registered=rel, ok=good))
            ok &= good
    for name in IMPORTED:
        resolved = Path(importlib.import_module(name).__file__).resolve()
        rel = 'src/'+name.split('.')[0]+'/'+name.replace('.', '/')+'.py'
        good = rel in sources and sha(resolved) == sources[rel]
        rows.append(dict(kind='module_import', name=name, resolved=str(resolved), registered=rel, ok=good))
        ok &= good
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    for package, files in SHARE.items():
        share = Path(get_package_share_directory(package))
        for f in files:
            rel = f'src/{package}/{f}'
            path = (share/f).resolve()
            good = rel in sources and sha(path) == sources[rel]
            rows.append(dict(kind='share', name=f'{package}/{f}', resolved=str(path), registered=rel, ok=good))
            ok &= good
    for package, names in (('arena_vehicle_interface', ('rotating_lidar', 'virtual_mcu')),
                           ('arena_autonomy', ('lidar_safety', 'local_pursuit', 'g1_g2_handoff_trial'))):
        for name in names:
            path = Path(get_package_prefix(package))/'lib'/package/name
            good = path.is_file() and os.access(path, os.X_OK)
            rows.append(dict(kind='executable', name=name, resolved=str(path), ok=good))
            ok &= good
    out = dict(dry=dry, ok=ok, python=sys.executable, ros2=shutil.which('ros2'), gz=shutil.which('gz'), rows=rows)
    target = HERE/('runtime_check_dry.json' if dry else 'straight_moving_lead/runtime_check.json')
    target.write_text(json.dumps(out, indent=1)+'\n', encoding='utf-8', newline='\n')
    print(json.dumps(dict(dry=dry, ok=ok, checked=len(rows), failed=[r for r in rows if not r['ok']])))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main('--dry' in sys.argv[1:]))
