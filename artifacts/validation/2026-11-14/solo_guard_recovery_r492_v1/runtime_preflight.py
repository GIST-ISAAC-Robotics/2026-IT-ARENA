"""배정된 WSL에서 실제 import 경로와 원 launch 연결만 확인. build/노드 실행 없음."""
import importlib,importlib.util,os,sys
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(A),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts'),str(ROOT/'tests')]
from prepare_execution import verify_connection,read
from variation_launch import write_new,sha

def guard_launch_selection():
    """원 실행 명령의 Declare/SetParameter만 평가한다. Node/Include/Opaque는 실행하지 않는다."""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument,GroupAction
    from launch_ros.actions import SetParameter
    command=read(A/'expected_launch_command.json')
    context=LaunchContext()
    context.launch_configurations.update(dict(x.split(':=',1) for x in command[4:] if ':=' in x))
    def parameter_actions(action):
        if isinstance(action,(DeclareLaunchArgument,SetParameter)):
            action.execute(context)
        elif isinstance(action,GroupAction):
            for child in action.get_sub_entities():
                parameter_actions(child)
    for label,path in [('follow',A/'course.launch.py'),('demo',A/'launch/demo.launch.py')]:
        spec=importlib.util.spec_from_file_location('r492_params_'+label,path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        for action in module.generate_launch_description().entities:parameter_actions(action)
    values=dict(context.launch_configurations.get('global_params',[]))
    selected={k:values[k] for k in ('g1_guard_observation_policy','g1_traffic_recovery_trial',
        'partial_observation_policy','g1_crop_edge_component_trial')}
    assert selected==dict(g1_guard_observation_policy='selected_r181',g1_traffic_recovery_trial=True,
        partial_observation_policy='bounded_local_v4',g1_crop_edge_component_trial=True),selected
    return dict(parameters=selected,command_observation_priority_selected=True,
        source='unchanged r485 launch command; actual launch DeclareLaunchArgument/SetParameter evaluation',
        ros_constructor_repeated=False,node_or_include_or_opaque_executed=False)

def main():
    value,_=verify_connection();selected=read(A/'source_selection.json');imports=[]
    modules=[('arena_autonomy.local_follow','src/arena_autonomy/arena_autonomy/local_follow.py'),
        ('arena_autonomy.race_runtime_policy.current_route_priority','src/arena_autonomy/arena_autonomy/race_runtime_policy/current_route_priority.py'),
        ('arena_autonomy.current_lane_preference','src/arena_autonomy/arena_autonomy/current_lane_preference.py'),
        ('arena_autonomy._numeric_kernels','src/arena_autonomy/arena_autonomy/_numeric_kernels.cpython-312-x86_64-linux-gnu.so')]
    modules += [('arena_autonomy.'+Path(r['path']).stem,r['path']) for r in selected['policy_changes']
        if r['role']=='production' and r['path'].startswith('src/arena_autonomy/')]
    for module,path in modules:
        actual=Path(importlib.import_module(module).__file__).resolve();expected=(ROOT/path).resolve()
        assert actual==expected,(module,str(actual),str(expected))
        imports.append(dict(module=module,path=str(actual),sha256=sha(actual)))
    assert next(x['sha256'] for x in imports if x['module']=='arena_autonomy._numeric_kernels')==selected['expected_native_sha256']
    import arena_autonomy.g1_g2_node
    import solo_runner
    from run import configure
    name='start_left20_yaw_plus3'
    os.environ['IT_ARENA_R492_VARIATION']=name
    spec=importlib.util.spec_from_file_location('r492_preflight',A/'launch/simulation.launch.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    assert module.generate_launch_description().entities
    original=solo_runner.obstacle_polygons
    configure(solo_runner,name,A/'runtime_preflight'/name)
    count=len(solo_runner.obstacle_polygons(None))
    assert count==len(original(A/'connection_v1'/f'{name}.sdf'))+3
    guard_selection=guard_launch_selection()
    write_new(A/'runtime_preflight/result.json',dict(passed=True,imports=imports,evaluator_obstacles=count,
        guard_launch_selection=guard_selection,
        source_delta=value['source_delta'],build_executed=False,native_rebuilt=False,
        ros_initialized=False,nodes_started=False,gazebo_started=False))

if __name__=='__main__':main()
