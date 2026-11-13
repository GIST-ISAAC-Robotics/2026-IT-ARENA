"""r444의 계약 변환 원문만 재사용한다. 센서/운동 수치를 수정하지 않는다."""
import ast
import copy
from pathlib import Path

ESTIMATED_SCOPE = 'host_estimated_crop_v1'


def load_converters(path):
    source = ast.parse(Path(path).read_bytes())
    kept = [n for n in source.body if isinstance(n, ast.FunctionDef)
            and n.name in ('estimated_header', 'estimated_record', 'normalized_plan')]
    assert len(kept) == 3
    namespace = dict(copy=copy, ESTIMATED_SCOPE=ESTIMATED_SCOPE,
        component_scope=lambda scope: 'bounded_estimated_edge_component_not_free_space'
            if scope == ESTIMATED_SCOPE else 'bounded_simulation_edge_component_not_free_space')
    exec(compile(ast.Module(body=kept, type_ignores=[]), str(path), 'exec'), namespace)
    # plan 한 개의 metadata 변환에도 원 for-body를 사용한다. 가짜 current를 만들지 않는다.
    record = next(n for n in kept if n.name == 'estimated_record')
    loop = next(n for n in record.body if isinstance(n, ast.For))
    function = ast.parse('def estimated_snapshot(record):\n snapshot=copy.deepcopy(record)\n return snapshot\n').body[0]
    function.body[1:1] = copy.deepcopy(loop.body)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[])), str(path), 'exec'), namespace)
    return namespace


def assert_snapshot_numbers_unchanged(original, converted):
    restored = copy.deepcopy(converted)
    assert restored['timing_verified'] is False
    assert restored['timing_policy'] == 'host_estimated_v1'
    assert restored['scan']['measurements']['time_basis'] == 'unverified_estimate'
    for key in ('timing_verified', 'timing_policy'):
        if key in original:
            restored[key] = original[key]
        else:
            restored.pop(key, None)
    restored['scan']['measurements']['time_basis'] = original['scan']['measurements']['time_basis']
    assert restored == original, 'numeric scan/history/speed/epoch changed'
