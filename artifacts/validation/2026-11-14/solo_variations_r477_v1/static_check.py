"""Windows XML/AST 연결 검사. ROS import/시뮬레이터/물리 실행을 하지 않는다."""
import ast
import copy
import inspect
import json
import math
from pathlib import Path
import struct
import tempfile
import xml.etree.ElementTree as ET

from prepare import A, ROOT, WORLD, SCENE, CASES, world_variant, spawn_variant
from prepare_execution import BASE, baseline_inputs, read
from variation_launch import SPAWN_LINE, runtime_wrapper, spawn_clone, sha, write_new


def xml_functions(path):
    tree = ast.parse(path.read_bytes())
    kept = [ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations')], level=0)]
    kept += [n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.Assign, ast.AnnAssign))]
    namespace = dict(ET=ET, Path=Path, math=math, json=json, copy=copy, struct=struct, tempfile=tempfile,
                     __file__=str(path))
    exec(compile(ast.fix_missing_locations(ast.Module(body=kept, type_ignores=[])), str(path), 'exec'), namespace)
    return namespace


def normalized_xml(path):
    tree = ET.parse(path).getroot()
    for value in tree.findall('.//uri'):
        if value.text and Path(value.text).name.startswith('batched_'):
            value.text = '<runtime>/'+Path(value.text).name
    return tree


def polygon_functions():
    # 원 evaluator의 rectangle/obstacle_polygons를 그대로 실행하되 Polygon 생성만 꼭짓점 tuple로 받는다.
    # Shapely 공간 연산 실행은 이 Windows 검사 범위가 아니다.
    path = ROOT/'scripts/build_experimental_track.py'
    tree = ast.parse(path.read_bytes())
    kept = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in ('rectangle','obstacle_polygons')]
    namespace = dict(ET=ET, Path=Path, math=math, Polygon=lambda points:tuple(points))
    exec(compile(ast.Module(body=kept, type_ignores=[]), str(path), 'exec'), namespace)
    return namespace['obstacle_polygons']


def main():
    source_hashes = baseline_inputs()
    original_prepared = {p:sha(p) for p in (A/'prepared').iterdir() if p.is_file()}
    output = A/'static_v1'
    output.mkdir(exist_ok=False)
    scripts = [*A.glob('*.py'), *(A/'launch').glob('*.py'), *(A/'execution').glob('*.py')]
    for path in scripts:
        ast.parse(path.read_bytes())
    checks = dict(artifact_python_ast=True)
    baseline_config, new_config = read(BASE/'config.json'), read(A/'config.json')
    expected = copy.deepcopy(baseline_config)
    expected['scope'] = new_config['scope']
    expected['launch']['entry_g1'] = new_config['launch']['entry_g1']
    assert new_config == expected
    assert baseline_config['cases'][0]['duration_s'] == 80.
    assert new_config['launch']['grid_slot'] == 3
    assert (A/'facilities_only.sdf').read_bytes() == (BASE/'facilities_only.sdf').read_bytes()
    checks.update(config_only_scope_and_artifact_path=True, facility_overlay_byte_exact=True)
    common = xml_functions(ROOT/'src/arena_bringup/launch/simulation.launch.py')
    original = common['_runtime_world_overrides']
    baseline_folder = output/'baseline'
    baseline_folder.mkdir()
    # r475 artifact의 실제 isolated 함수 자체를 호출한다. imports/generate/ROS setup은 호출하지 않는다.
    path = BASE/'launch/simulation.launch.py'
    node = next(n for n in ast.parse(path.read_bytes()).body if isinstance(n,ast.FunctionDef) and n.name=='isolated')
    namespace = dict(original=original, A=baseline_folder, ET=ET, Path=Path, json=json,
                     hashlib=__import__('hashlib'))
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), namespace)
    options = dict(detector='configured', scene_broadcaster=False, speed_bump=False, batch_visuals=True,
        evaluation_models=A/'facilities_only.sdf', evaluation_wheel_contacts=True,
        evaluation_physics_step='', evaluation_course_markers='blank_20_30',
        evaluation_grass_alignment='road_level', signal_posts='none', track='official_v2026_11_06')
    baseline_path = baseline_folder/'world.sdf'
    baseline_report = namespace['isolated'](WORLD, baseline_path, **options)
    baseline_tree = normalized_xml(baseline_path)
    obstacles = polygon_functions()
    baseline_walls = obstacles(baseline_path)
    assert obstacles(WORLD) == baseline_walls
    checks['raw_static_evaluator_matches_nominal_runtime'] = True
    slot = next(s for s in read(SCENE)['starting_grid']['slots'] if s['index']==3)
    base_pose = (slot['x'],slot['y'],slot['yaw_rad'])
    records = []
    for name, values in CASES.items():
        case = output/name
        case.mkdir()
        destination = case/'world.sdf'
        report = runtime_wrapper(original,name,case)(WORLD,destination,**options)
        assert report == baseline_report
        runtime_tree = normalized_xml(destination)
        expected_tree = copy.deepcopy(baseline_tree)
        if values['wall_extra_width_m']:
            size = expected_tree.find(".//collision[@name='walls_main_left_1_col']/geometry/box/size")
            size.text = '1.880000000 0.070000000 0.400000000'
        assert ET.tostring(runtime_tree) == ET.tostring(expected_tree)
        assert len(runtime_tree.findall('world/model')) == 4
        assert runtime_tree.find(".//visual[@name='walls_main_left_1_vis']") is None
        mesh_changes = {}
        for mesh in baseline_folder.glob('batched_*.obj'):
            before, after = mesh.read_text().splitlines(), (case/mesh.name).read_text().splitlines()
            assert len(before) == len(after)
            changed = [i for i,(x,y) in enumerate(zip(before,after)) if x!=y]
            if changed:
                assert values['wall_extra_width_m'] and mesh.name.startswith('batched_walls_')
                assert len(changed)==8 and all(before[i].startswith('v ') and after[i].startswith('v ') for i in changed)
                mesh_changes[mesh.name] = dict(changed_vertex_lines=changed)
        assert bool(mesh_changes) == bool(values['wall_extra_width_m'])
        raw, edits = world_variant(ET.parse(WORLD).getroot(),name)
        raw_path = case/'raw_for_evaluator.sdf'
        ET.ElementTree(raw).write(raw_path,encoding='utf8',xml_declaration=True)
        expected_polygons = obstacles(raw_path)
        assert expected_polygons == obstacles(destination)
        polygon_changes = sum(a!=b for a,b in zip(baseline_walls,expected_polygons))
        assert polygon_changes == (1 if values['wall_extra_width_m'] else 0)
        old_source = inspect.getsource(common['_launch_setup'])
        clone = spawn_clone(common['_launch_setup'],name,case)
        assert clone.__globals__ is common
        pose = clone.__globals__['_r477_spawn'](*base_pose)
        assert pose == spawn_variant(*base_pose,name)
        assert math.isclose(math.hypot(pose[0]-base_pose[0],pose[1]-base_pose[1]),abs(values['lateral_m']),abs_tol=1e-12)
        assert math.isclose(pose[2]-base_pose[2],math.radians(values['yaw_delta_deg']),abs_tol=1e-12)
        assert inspect.getsource(common['_launch_setup']) == old_source
        launch_source = old_source.replace(SPAWN_LINE,SPAWN_LINE+'\n    spawn_x, spawn_y, spawn_yaw = _r477_spawn(spawn_x, spawn_y, spawn_yaw)')
        old_body = ast.parse(old_source).body[0].body
        new_body = ast.parse(launch_source).body[0].body
        inserted = [i for i,n in enumerate(new_body) if isinstance(n,ast.Assign) and isinstance(n.value,ast.Call)
                    and isinstance(n.value.func,ast.Name) and n.value.func.id=='_r477_spawn']
        assert len(inserted)==1
        new_body.pop(inserted[0])
        assert ast.dump(ast.Module(body=new_body,type_ignores=[])) == ast.dump(ast.Module(body=old_body,type_ignores=[]))
        records.append(dict(case=name, spawn=list(pose), raw_edits=edits, static_wall_count=len(expected_polygons),
            changed_wall_polygons=polygon_changes, runtime_xml_delta='one collision size' if edits else 'none',
            mesh_changes=mesh_changes, actor_count=0, facilities=3, runtime_wall_matches_evaluator=True,
            runtime_world_sha256=sha(destination), spawn_after_slot_only=True))
    checks.update(runtime_two_variants_exact=True, visual_mesh_matches_collision=True,
                  original_source_relative_uris_preserved=True, spawn_local_assignment_only=True)
    # 실행 연결/STOP argv는 같은 case 명칭을 사용하며 controller 선택은 r475과 동일하다.
    from launch_adapter import SELECTION,rewrite_command,instrument_start
    base_adapter = {}
    for node in ast.parse((BASE/'launch_adapter.py').read_bytes()).body:
        if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='SELECTION' for t in node.targets):
            base_adapter['selection'] = ast.literal_eval(node.value)
    assert SELECTION == base_adapter['selection']
    command = ['ros2','launch','--noninteractive','arena_bringup','g1_follow.launch.py','grid_slot:=3']
    assert rewrite_command(command) == command[:3]+[str(A/'course.launch.py')]+command[5:]+SELECTION
    class Process:
        def Popen(self, command, *args, **kwargs):
            assert kwargs['env']['IT_ARENA_R477_VARIATION'] in CASES
            assert kwargs['env']['KEEP'] == 'yes'
            return command,kwargs['env']
    def starter(command, **kwargs):
        return subprocess.Popen(command,**kwargs)
    starter.__globals__['subprocess'] = Process()
    for name in CASES:
        result = instrument_start(starter,output/name/'command.json',name)(command,env={'KEEP':'yes'})
        assert result[0] == rewrite_command(command) and result[1]['IT_ARENA_R477_VARIATION'] == name
    checks.update(controller_selection_exact=True, variation_environment_preserves_existing=True)
    assert all(sha(ROOT/p)==h for p,h in source_hashes.items())
    assert all(sha(p)==h for p,h in original_prepared.items())
    checks.update(common_and_baseline_inputs_unchanged=True, initial_static_manifest_preserved=True)
    value = dict(passed=True, checks=checks, cases=records, python_files_checked=len(scripts),
        baseline_input_count=len(source_hashes), common_delta=[], control_selection_delta=[],
        scope='Windows original Python XML/AST and polygon-coordinate functions; no ROS/physics/Shapely operations',
        ros_initialized=False, simulator_started=False, build_executed=False, real_launch_setup_executed=False)
    write_new(output/'result.json',value)
    print(json.dumps(dict(passed=True,cases=len(records),checks=len(checks),baseline_inputs=len(source_hashes))))


if __name__=='__main__':
    main()
