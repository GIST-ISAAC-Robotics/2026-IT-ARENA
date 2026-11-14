"""최종 인계 후 WSL에서 실제 import 경로와 경쟁 프로세스만 확인한다."""
import importlib,os,sys
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(A),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts'),str(ROOT/'tests')]
from prepare import read,sha,verify_recipe,write_new
def main():
    info,selected=verify_recipe();imports=[]
    for name in ('local_follow','local_path','moving_pass','lidar_safety','g1_g2_node','bypass_wall_fit','current_wall_reacquire','guard_status_payload','guard_memory_command','current_path_recheck._geometry','race_runtime_policy.retained_recheck._geometry'):
        module=importlib.import_module('arena_autonomy.'+name);path=Path(module.__file__).resolve();relative='src/arena_autonomy/arena_autonomy/'+name.replace('.','/')+'.py'
        assert path==(ROOT/relative).resolve() and sha(path)==selected[relative]
        imports.append(dict(module=module.__name__,path=str(path),sha256=sha(path)))
    from arena_autonomy import _numeric_kernels
    native=Path(_numeric_kernels.__file__);assert sha(native)==info['native_sha256']
    competing=[]
    for p in Path('/proc').glob('[0-9]*'):
        if int(p.name) in (os.getpid(),os.getppid()):continue
        try:
            exe=(p/'exe').resolve().name;env=(p/'environ').read_bytes().split(b'\0')
            if exe in ('bash','sh','dash'):continue
            if any(v.startswith(b'GZ_PARTITION=arena_autonomy_test_') for v in env) or exe in ('gzserver','gzclient','gz','ruby'):competing.append(dict(pid=int(p.name),exe=exe))
        except (FileNotFoundError,PermissionError,ProcessLookupError):pass
    assert not competing,competing
    write_new(A/'preflight/result.json',dict(passed=True,imports=imports,native_sha256=sha(native),competing_work=competing,
        geometry_receipts_reused_from='r544 attempt2',derived_world_sha256=sha(A/'field_world.sdf'),source_delta=info['source_delta'],
        status_detail='full',build_executed=read(A/'build_result.json')['passed'],ros_initialized=False,simulator_started=False))
    print('r550 final import/competition passed; no nodes started')
if __name__=='__main__':main()
