"""r459 이관 뒤 실제 import 및 고정 CLI만 확인. ROS init/주행을 실행하지 않는다."""
import importlib
import json
from pathlib import Path
import sys

A=Path(__file__).resolve().parent.parent;ROOT=A.parents[3];BASE=A.parent/'guard_crawl_drive_r452_v1'
sys.path[:0]=[str(A),str(ROOT/'scripts'),str(ROOT/'tests')]
from prepare import POLICY,sha,read,write_new,verify_common,arguments
common=verify_common();old=read(BASE/'build_v1/import_paths.log')
modules={r['module'] for r in old['modules']}
modules.update(p['path'].removeprefix('src/arena_autonomy/').removesuffix('.py').replace('/','.') for p in read(POLICY/'changes.json') if p['path'].startswith('src/'))
rows=[]
for name in sorted(modules):
    path=Path(importlib.import_module(name).__file__).resolve()
    expected=ROOT/'src'/name.split('.')[0]/Path(*name.split('.')).with_suffix('.py')
    assert path==expected,(name,str(path));assert sha(path)==common[path.relative_to(ROOT).as_posix()]
    rows.append(dict(module=name,path=str(path),sha256=sha(path)))
from arena_autonomy import numeric_kernels,_numeric_kernels
numeric_kernels.configure('native_v1');native=Path(_numeric_kernels.__file__).resolve()
assert sha(native)==old['native_module']['sha256'] and _numeric_kernels.ABI_VERSION==1
from ament_index_python.packages import get_package_prefix
prefix=Path(get_package_prefix('arena_autonomy')).resolve();assert prefix==ROOT/'install/arena_autonomy'
assert (A/'expected_header.json').read_bytes()==(BASE/'expected_header.json').read_bytes()
assert (A/'launch_adapter.py').read_text().replace('solo_full_lap_r457_v1','guard_crawl_drive_r452_v1')==(BASE/'launch_adapter.py').read_text()
import rclpy
import solo_runner as target
from run import configure
configure(target)
class ReachedInit(Exception):pass
def stopped(*args,**kwargs):raise ReachedInit()
original_init=rclpy.init;before=list(sys.argv);rclpy.init=stopped
argv=arguments();argv[argv.index('--output')+1]=(A/'build_v1/entry_only').relative_to(ROOT).as_posix()
try:
    sys.argv=argv
    try:target.main()
    except ReachedInit:reached=True
    else:raise AssertionError('fixed CLI did not reach intercepted init')
finally:rclpy.init=original_init;sys.argv=before
assert (A/'build_v1/entry_only/facilities_only.sdf').read_bytes()==(A/'facilities_only.sdf').read_bytes()
value=dict(build_executed=True,packages=['arena_autonomy'],build_exitcode=int((A/'build_v1/build.exit').read_text()),
    import_cli_exitcode=0,modules=rows,native_import_verified=True,native=dict(path=str(native),sha256=sha(native)),
    common_sha256=common,common_delta=read(POLICY/'changes.json'),fixed_cli_reached_ros_init=True,
    header_selection_unchanged=True,ros_initialized=False,gazebo_started=False,
    r459_integration_sha256=sha(POLICY/'integration.json'),r452_archive_sha256=sha(BASE/'sources.zip'))
write_new(A/'build_v1/receipt.json',value)
print(json.dumps(dict(import_cli_passed=True,modules=len(rows),common_files=len(common),common_delta=6,
                     native_unchanged=True,header_selection_unchanged=True,ros_initialized=False)))
