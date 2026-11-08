"""실행 직전: 실제로 import/launch되는 설치 경로가 등록한 src 바이트를 가리키는지 확인한다. 사용: check_runtime.py A|B

colcon 설치는 src 심볼릭 링크(--symlink-install)라는 현재 상태를 전제로 하지 않고, 해석된 실제 파일의
해시를 registered.json의 같은 src 경로 해시와 대조한다. 결과는 <run 폴더>/runtime_check.json에 남긴다.
"""
import hashlib, importlib, json, os, shutil, sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
MODULES = ('arena_vehicle_interface.rotating_lidar', 'arena_vehicle_interface.lidar_impairment',
           'arena_vehicle_interface.actuation_ros', 'arena_autonomy.lidar_safety', 'arena_autonomy.local_pursuit',
           'arena_autonomy.g1_g2_node', 'arena_autonomy.local_follow', 'arena_autonomy.piecewise_wall')
SHARE = {'arena_bringup': ('launch/g1_follow.launch.py', 'launch/simulation.launch.py', 'launch/demo.launch.py'),
         'arena_description': ('config/b0390_c1_jr60p20_sim.yaml', 'config/vehicle.yaml')}
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main(key):
    dry = key == '--dry'  # 등록 전 점검: 현재 src 해시와 대조하고 아무것도 쓰지 않는다
    if dry:
        sources = {p.relative_to(ROOT).as_posix(): sha(p) for p in (ROOT/'src').rglob('*') if p.is_file()}
    else:
        registered = json.loads((HERE/'registered.json').read_text(encoding='utf-8'))
        sources = registered['sources']
    rows, ok = [], True
    for name in MODULES:
        path = Path(importlib.import_module(name).__file__).resolve()
        rel = 'src/'+name.split('.')[0]+'/'+name.replace('.', '/')+'.py'
        good = rel in sources and sha(path) == sources[rel]
        rows.append(dict(kind='module', name=name, resolved=str(path), registered=rel, ok=good)); ok &= good
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    for package, files in SHARE.items():
        share = Path(get_package_share_directory(package))
        for f in files:
            rel = f'src/{package}/{f}'
            path = (share/f).resolve()
            good = rel in sources and sha(path) == sources[rel]
            rows.append(dict(kind='share', name=f'{package}/{f}', resolved=str(path), registered=rel, ok=good)); ok &= good
    for package, names in (('arena_vehicle_interface', ('rotating_lidar', 'virtual_mcu')),
                           ('arena_autonomy', ('lidar_safety', 'local_pursuit', 'g1_g2_handoff_trial'))):
        for name in names:
            path = Path(get_package_prefix(package))/'lib'/package/name
            good = path.is_file() and os.access(path, os.X_OK)
            rows.append(dict(kind='executable', name=name, resolved=str(path), ok=good)); ok &= good
    out = dict(run=key, ok=ok, python=sys.executable, ros2=shutil.which('ros2'), gz=shutil.which('gz'), rows=rows)
    if not dry:
        folder = ROOT/registered['runs'][key]['folder']
        (folder/'runtime_check.json').write_text(json.dumps(out, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(dict(run=key, ok=ok, checked=len(rows), failed=[r for r in rows if not r['ok']])))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv[1]))
