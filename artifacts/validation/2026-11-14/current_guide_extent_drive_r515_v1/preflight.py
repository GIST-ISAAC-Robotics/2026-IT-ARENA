"""실제 소스 import와 기존 시설 구성 확인만 한다. ROS init/launch/Gazebo 없음."""
from pathlib import Path
import importlib,os,sys
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(A),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts'),str(ROOT/'tests')]
from prepare import verify_recipe,read,sha,write_new
def main():
    info,_=verify_recipe();imports=[]
    assert read(A/'geometry_right_side.json')['passed']
    for item in read(A/'source_selection.json')['control_changes']:
        p=item['path']
        if not p.startswith('src/arena_autonomy/arena_autonomy/'):continue
        name='arena_autonomy.'+p.split('/arena_autonomy/arena_autonomy/')[1][:-3].replace('/','.')
        path=Path(importlib.import_module(name).__file__).resolve();assert path==(ROOT/p).resolve()
        assert sha(path)==item['after_sha256']
        imports.append(dict(module=name,path=str(path),sha256=sha(path)))
    from arena_autonomy import _numeric_kernels
    native=Path(_numeric_kernels.__file__);assert sha(native)==info['native_sha256']
    import arena_autonomy.g1_g2_node
    import validate_g1_traffic as target
    from run import configure
    configure(target)
    # 소유 인계 뒤 실제 남은 Gazebo/validator 작업이 없는지 한 번만 확인한다.
    competing=[]
    for p in Path('/proc').glob('[0-9]*'):
        if int(p.name) in (os.getpid(),os.getppid()):continue
        try:
            args=(p/'cmdline').read_bytes().split(b'\0');env=(p/'environ').read_bytes().split(b'\0');exe=(p/'exe').resolve().name
            if exe in ('bash','sh','dash'):continue
            if any(x.startswith(b'GZ_PARTITION=arena_autonomy_test_') for x in env) or exe in ('gzserver','gzclient','gz','ruby'):
                competing.append(dict(pid=int(p.name),exe=exe))
        except (FileNotFoundError,PermissionError,ProcessLookupError):pass
    assert not competing,competing
    write_new(A/'preflight/result.json',dict(passed=True,imports=imports,native_path=str(native),native_sha256=sha(native),
        competing_work=competing,source_delta=info['source_delta'],additional_tests=info['additional_tests'],
        configure_import_and_geometry=True,build_executed=False,ros_initialized=False,simulator_started=False))
    print('r515 actual import and existing facility connection passed; no nodes started')
if __name__=='__main__':main()
