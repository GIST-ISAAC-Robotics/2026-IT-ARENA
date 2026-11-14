"""실제 소스 import와 기존 시설 구성 확인만 한다. ROS init/launch/Gazebo 없음."""
from pathlib import Path
import importlib,os,sys
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(A),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts'),str(ROOT/'tests')]
from prepare import verify_recipe,read,sha,write_new
def main():
    info,selected=verify_recipe();imports=[]
    assert read(A/'geometry_right_side.json')['passed']
    selection=read(A/'source_selection.json')
    import_items=[*selection['control_changes'],*[dict(path=p,after_sha256=h) for p,h in selection.get('additional_source_hashes',{}).items()]]
    for item in import_items:
        p=item['path']
        if not p.startswith('src/arena_autonomy/arena_autonomy/'):continue
        name='arena_autonomy.'+p.split('/arena_autonomy/arena_autonomy/')[1][:-3].replace('/','.')
        path=Path(importlib.import_module(name).__file__).resolve();assert path==(ROOT/p).resolve()
        assert sha(path)==item['after_sha256']
        imports.append(dict(module=name,path=str(path),sha256=sha(path)))
    for name in selection.get('preflight_imports',[]):
        if name in [r['module'] for r in imports]:continue
        p='src/arena_autonomy/'+name.replace('.','/')+'.py'
        path=Path(importlib.import_module(name).__file__).resolve();assert path==(ROOT/p).resolve()
        assert sha(path)==selected[p],p
        imports.append(dict(module=name,path=str(path),sha256=sha(path)))
    from arena_autonomy import _numeric_kernels
    native=Path(_numeric_kernels.__file__);assert sha(native)==info['native_sha256']
    import arena_autonomy.g1_g2_node
    import traffic_runner as target
    from run import configure
    configure(target)
    from field_wall_geometry import SOURCE,DERIVED,verify_derived
    field=verify_derived()
    assert sha(DERIVED)==read(A/'resource_manifest.json')['output_world_sha256']
    # 실제 evaluator의 연결된 polygon을 호출한다. ROS init이나 노드는 시작하지 않는다.
    obstacles=target.obstacle_polygons(SOURCE)
    assert len(obstacles)>=field['derived_wall_collision_count']+3
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
        configure_import_and_geometry=True,derived_world_sha256=sha(DERIVED),evaluation_obstacle_count=len(obstacles),
        additional_sources=info['additional_sources'],build_executed=False,ros_initialized=False,simulator_started=False))
    print('r539 actual import and existing facility connection passed; no nodes started')
if __name__=='__main__':main()
