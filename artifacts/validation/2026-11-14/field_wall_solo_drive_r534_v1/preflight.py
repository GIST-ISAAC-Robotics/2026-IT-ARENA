"""최종 인계 뒤 WSL 실제 import와 연결 확인. 현재 준비에서는 실행하지 않는다."""
import importlib,os,sys
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(A),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts'),str(ROOT/'tests')]
from prepare import TRAFFIC,read,sha,verify_recipe,write_new


def main():
    info,selected=verify_recipe();imports=[]
    paths={'src/arena_autonomy/arena_autonomy/'+p+'.py' for p in ('local_follow','moving_pass','lidar_safety','g1_g2_node','bypass_wall_fit','current_wall_reacquire')}
    for source in (read(TRAFFIC/'source_selection.json'),read(A/'source_selection.json')):
        paths.update(x['path'] for x in source['control_changes']);paths.update(source.get('additional_sources',[]))
    for p in sorted(paths):
        if not p.startswith('src/arena_autonomy/arena_autonomy/') or not p.endswith('.py'):continue
        module='arena_autonomy.'+p.split('/arena_autonomy/arena_autonomy/')[1][:-3].replace('/','.')
        actual=Path(importlib.import_module(module).__file__).resolve();assert actual==(ROOT/p).resolve() and sha(actual)==selected[p]
        imports.append(dict(module=module,path=str(actual),sha256=sha(actual)))
    from arena_autonomy import _numeric_kernels
    native=Path(_numeric_kernels.__file__);assert sha(native)==info['native_sha256']
    import solo_runner as target
    from run import configure
    from field_wall_geometry import SOURCE,DERIVED,collision_digest
    configure(target,selected);obstacles=target.obstacle_polygons(SOURCE);assert len(obstacles)>=478
    competing=[]
    for p in Path('/proc').glob('[0-9]*'):
        if int(p.name) in (os.getpid(),os.getppid()):continue
        try:
            exe=(p/'exe').resolve().name;env=(p/'environ').read_bytes().split(b'\0')
            if exe in ('bash','sh','dash'):continue
            if any(v.startswith(b'GZ_PARTITION=arena_autonomy_test_') for v in env) or exe in ('gzserver','gzclient','gz','ruby'):
                competing.append(dict(pid=int(p.name),exe=exe))
        except (FileNotFoundError,PermissionError,ProcessLookupError):pass
    assert not competing,competing
    write_new(A/'preflight/result.json',dict(passed=True,imports=imports,native_sha256=sha(native),competing_work=competing,
        derived_world_sha256=sha(DERIVED),walls_collision_sha256=collision_digest(DERIVED),evaluator_obstacle_count=len(obstacles),
        source_delta=info['source_delta'],source_snapshot_includes_world=True,build_executed=False,ros_initialized=False,simulator_started=False))
    print('r534 actual import and geometry connection passed; no nodes started')


if __name__=='__main__':main()
