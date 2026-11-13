"""Windows 준비 대조. 공통 평가기/ROS import 또는 실행은 하지 않는다."""
import ast
import json
import math
import xml.etree.ElementTree as ET
from prepare import A, BASE, ROOT, read, sha, write_new
from run import compose_facilities


def main():
    manifest = read(A/'recipe_manifest.json')
    assert all(sha(A/p)==h for p,h in manifest['recipe_files'].items())
    source = ROOT/'scripts/validate_g1_traffic.py'
    tree = ast.parse(source.read_bytes())
    actor = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name=='actor_sdf')
    namespace = dict(ET=ET, math=math, CONTACT_TOPIC='/eval/g1b_actor/contact')
    exec(compile(ast.Module(body=[actor], type_ignores=[]), str(source), 'exec'), namespace)
    config = read(A/'config.json')
    original = ET.fromstring(namespace['actor_sdf'](config, (0., 0., 0.)))
    combined = ET.fromstring(compose_facilities(ET.tostring(original, encoding='unicode')))
    expected_facilities = ET.parse(A/'facilities_only.sdf').getroot().findall('model')
    models = combined.findall('model')
    assert len(models)==4 and ET.tostring(models[0])==ET.tostring(original.find('model'))
    assert [ET.tostring(m) for m in models[1:]]==[ET.tostring(m) for m in expected_facilities]
    assert len(combined.findall("plugin[@name='gz::sim::systems::Contact']"))==1
    assert models[0].findtext("link/collision/geometry/box/size")=='0.2 0.15 0.22'
    flags = {arg.value for n in ast.walk(tree) if isinstance(n, ast.Call)
             and isinstance(n.func, ast.Attribute) and n.func.attr=='add_argument'
             for arg in n.args if isinstance(arg, ast.Constant) and isinstance(arg.value,str)
             and arg.value.startswith('--')}
    selected = [x for x in read(A/'arguments.json')[1:] if x.startswith('--')]
    assert set(selected)<=flags, set(selected)-flags
    before = read(BASE/'prepared.json')['frozen_inputs']
    common = {p:h for p,h in before.items() if p.startswith(('src/','scripts/','tests/','config/'))}
    changed = [p for p,h in common.items() if not (ROOT/p).is_file() or sha(ROOT/p)!=h]
    assert not changed, changed
    # 공통 선언 기본값 OFF. 실행 명령에서 별도 활성화하지 않는다.
    control = ast.parse((ROOT/'src/arena_autonomy/arena_autonomy/local_pursuit.py').read_bytes())
    marker = [n for n in ast.walk(control) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
              and n.func.attr=='declare_parameter' and len(n.args)>1
              and isinstance(n.args[0],ast.Constant) and n.args[0].value=='vehicle_marker_observer_enabled']
    assert len(marker)==1 and isinstance(marker[0].args[1],ast.Constant) and marker[0].args[1].value is False
    result = dict(passed=True, recipe_hashes_match=True, source_case_exact=manifest['case_exact'],
        common_reference_files=len(common), changed_common_inputs=changed,
        primary_actor_xml_unchanged_by_composition=True, facility_models_exact=True, actor_models=1, facility_models=3,
        actor_collision_size_m=[.2,.15,.22], contact_plugin_count=1,
        selected_cli_flags_known=len(selected), marker_observer_enabled=False,
        source_evaluator_sha256=sha(source), recipe_manifest_sha256=sha(A/'recipe_manifest.json'),
        scope='AST and pure XML composition only; no common evaluator import, route geometry execution, build, WSL, Jetson or simulation',
        simulation_started=False, source_frozen=False,
        pending=['root review', 'r477 WSL ownership return', 'final freeze', 'one execution and original route validation'])
    write_new(A/'static_check.json', result)
    print(json.dumps(result))


if __name__=='__main__':
    main()
