"""실제 노드 생성과 별개인 launch 설정 정적 연결 검사. r418이 실제 치환을 검증한다."""
import ast
from pathlib import Path

A = Path(__file__).resolve().parent
LAUNCH = A/'staged/src/arena_bringup/launch'
FLAG = 'g1_crop_edge_component_trial'


def test_one_false_default_selection_reaches_both_nodes():
    tree = ast.parse((LAUNCH/'simulation.launch.py').read_text(encoding='utf8'))
    declarations = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
        and isinstance(n.func, ast.Name) and n.func.id == 'DeclareLaunchArgument'
        and n.args and isinstance(n.args[0], ast.Constant) and n.args[0].value == FLAG]
    assert len(declarations) == 1
    assert next(k.value.value for k in declarations[0].keywords if k.arg == 'default_value') == 'false'
    assignments = [value for node in ast.walk(tree) if isinstance(node, ast.Dict)
        for key, value in zip(node.keys, node.values) if isinstance(key, ast.Constant) and key.value == FLAG]
    assert len(assignments) == 2
    assert all(isinstance(value, ast.Name) and value.id == 'crop_edge' for value in assignments)
    text = (LAUNCH/'g1_follow.launch.py').read_text(encoding='utf8')
    assert "DeclareLaunchArgument('g1_crop_edge_component_trial', default_value='false'" in text
    assert "value=ParameterValue(LaunchConfiguration('g1_crop_edge_component_trial'), value_type=bool)" in text
