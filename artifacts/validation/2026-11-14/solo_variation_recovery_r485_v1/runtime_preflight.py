"""배정된 WSL에서 실제 import 경로와 원 launch 연결만 확인. build/노드 실행 없음."""
import importlib,importlib.util,os,sys
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(A),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts'),str(ROOT/'tests')]
from prepare_execution import verify_connection,read
from variation_launch import write_new,sha

def main():
    value,_=verify_connection();selected=read(A/'source_selection.json');imports=[]
    for module,path in [('arena_autonomy.local_follow','src/arena_autonomy/arena_autonomy/local_follow.py'),
        ('arena_autonomy.race_runtime_policy.current_route_priority','src/arena_autonomy/arena_autonomy/race_runtime_policy/current_route_priority.py'),
        ('arena_autonomy.current_lane_preference','src/arena_autonomy/arena_autonomy/current_lane_preference.py'),
        ('arena_autonomy._numeric_kernels','src/arena_autonomy/arena_autonomy/_numeric_kernels.cpython-312-x86_64-linux-gnu.so')]:
        actual=Path(importlib.import_module(module).__file__).resolve();expected=(ROOT/path).resolve()
        assert actual==expected,(module,str(actual),str(expected))
        imports.append(dict(module=module,path=str(actual),sha256=sha(actual)))
    assert imports[-1]['sha256']==selected['expected_native_sha256']
    import arena_autonomy.g1_g2_node
    import solo_runner
    from run import configure
    name='start_left20_yaw_plus3'
    os.environ['IT_ARENA_R485_VARIATION']=name
    spec=importlib.util.spec_from_file_location('r485_preflight',A/'launch/simulation.launch.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    assert module.generate_launch_description().entities
    original=solo_runner.obstacle_polygons
    configure(solo_runner,name,A/'runtime_preflight'/name)
    count=len(solo_runner.obstacle_polygons(None))
    assert count==len(original(A/'connection_v1'/f'{name}.sdf'))+3
    write_new(A/'runtime_preflight/result.json',dict(passed=True,imports=imports,evaluator_obstacles=count,
        source_delta=value['source_delta'],build_executed=False,native_rebuilt=False,
        ros_initialized=False,nodes_started=False,gazebo_started=False))

if __name__=='__main__':main()
