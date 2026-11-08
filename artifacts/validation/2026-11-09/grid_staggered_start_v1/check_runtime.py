"""실행 직전: 실제 import/launch 경로가 등록한 src 바이트를 가리키는지 확인한다. 사용: check_runtime.py [--dry]

r48 check_runtime.py의 모듈·share·실행 파일 목록을 그대로 확인하고, 추가로 실제 import된 두 패키지
디렉터리의 모든 .py와 console_scripts 진입점 모듈을 대조한다. 설치 방식(develop/symlink/copy)을 전제하지 않는다.
"""
import hashlib, importlib, json, os, shutil, sys
from importlib.metadata import distribution
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
MODULES = ('arena_vehicle_interface.rotating_lidar', 'arena_vehicle_interface.lidar_impairment',
           'arena_vehicle_interface.actuation_ros', 'arena_autonomy.lidar_safety', 'arena_autonomy.local_pursuit',
           'arena_autonomy.g1_g2_node', 'arena_autonomy.local_follow', 'arena_autonomy.piecewise_wall', 'arena_autonomy.same_path_gap_trial', 'arena_autonomy.pass_release_trial', 'arena_autonomy.masked_motion_speed', 'arena_autonomy.bypass_corridor_trial', 'arena_autonomy.short_entry_transition', 'arena_autonomy.adaptive_corridor_offset', 'arena_autonomy.bypass_entry_trial', 'arena_autonomy.bypass_handoff', 'arena_autonomy.bypass_handoff_corridor_trial', 'arena_autonomy.adaptive_entry_preview', 'arena_autonomy.corner_boundary_support')
SHARE = {'arena_bringup': ('launch/g1_follow.launch.py', 'launch/simulation.launch.py', 'launch/demo.launch.py'),
         'arena_description': ('config/b0390_c1_jr60p20_sim.yaml', 'config/vehicle.yaml')}
EXECUTABLES = (('arena_vehicle_interface', ('rotating_lidar', 'virtual_mcu')),
               ('arena_autonomy', ('lidar_safety', 'local_pursuit', 'g1_g2_handoff_trial')))
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main(dry):
    if dry:
        sources = {p.relative_to(ROOT).as_posix(): sha(p) for p in (ROOT/'src').rglob('*.py')}
        sources.update({p.relative_to(ROOT).as_posix(): sha(p) for p in (ROOT/'src').rglob('*.yaml')})
    else:
        sources = json.loads((HERE/'registered.json').read_bytes())['sources']
    rows, ok = [], True

    def add(kind, name, path, registered):
        nonlocal ok
        good = registered in sources and Path(path).is_file() and sha(path) == sources[registered]
        rows.append(dict(kind=kind, name=name, resolved=str(path), registered=registered, ok=good))
        ok &= good

    for name in MODULES:
        path = Path(importlib.import_module(name).__file__).resolve()
        add('module', name, path, 'src/'+name.split('.')[0]+'/'+name.replace('.', '/')+'.py')
    for package in ('arena_autonomy', 'arena_vehicle_interface'):
        folder = Path(importlib.import_module(package).__file__).resolve().parent
        for path in sorted(folder.glob('*.py')):
            add('package_file', f'{package}/{path.name}', path, f'src/{package}/{package}/{path.name}')
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    for package, files in SHARE.items():
        share = Path(get_package_share_directory(package))
        for f in files:
            add('share', f'{package}/{f}', (share/f).resolve(), f'src/{package}/{f}')
    for package, names in EXECUTABLES:
        dist = distribution(package.replace('_', '-'))
        for name in names:
            path = Path(get_package_prefix(package))/'lib'/package/name
            (entry,) = [e for e in dist.entry_points if e.group == 'console_scripts' and e.name == name]
            module = Path(importlib.import_module(entry.module).__file__).resolve()
            target = 'src/'+package+'/'+entry.module.replace('.', '/')+'.py'
            good = path.is_file() and os.access(path, os.X_OK) and sources.get(target) == sha(module)
            rows.append(dict(kind='executable', name=name, resolved=str(path), entry=entry.value,
                             entry_module=str(module), registered=target, ok=good))
            ok &= good
    out = dict(run='dry' if dry else 'CASE', ok=ok, python=sys.executable, ros2=shutil.which('ros2'),
               gz=shutil.which('gz'), checked=len(rows), rows=rows)
    if not dry:
        (HERE/'runtime_check.json').write_text(json.dumps(out, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(dict(run=out['run'], ok=ok, checked=len(rows), failed=[r for r in rows if not r['ok']])))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main('--dry' in sys.argv))
