"""평가기의 launch 진입점만 선택한다. 실제 노드 prefix/정책 monkeypatch는 없다."""
import json
from pathlib import Path
import types

A = Path(__file__).resolve().parent
SELECTION = [
    'g1_current_path_recheck:=selected_v1', 'g1_submission_policy:=fresh_native_v1',
    'g1_guard_observation_policy:=selected_r181', 'g1_released_body_gap_trial:=true',
    'g1_expiry_route_reference_trial:=true', 'g1_oblique_gap_trial:=true',
    'g1_current_recheck_execution:=worker_event_v1', 'g1_numeric_execution:=native_v1',
    'g1_runtime_policy:=cad_runtime_r377_v1', 'g1_current_corner_budget:=r396_v1',
    'g1_crop_edge_component_trial:=true',
    'g1_complete_plan_trace:=true',
]


def rewrite_command(command):
    assert command[:5] == ['ros2', 'launch', '--noninteractive', 'arena_bringup', 'g1_follow.launch.py']
    assert not any('--launch-prefix' in word or 'node_entry.py' in word for word in command)
    keys = {item.split(':=', 1)[0] for item in SELECTION}
    assert not any(word.split(':=', 1)[0] in keys for word in command[5:])
    assert not any(word.startswith('pilot_marker_z:=') for word in command), 'use final YAML marker90mm'
    return [*command[:3], str(A/'marker_course.launch.py'), *command[5:], *SELECTION]


def instrument_start(original, evidence):
    process_module = original.__globals__['subprocess']

    class Proxy:
        def __getattr__(self, name):
            return getattr(process_module, name)

        def Popen(self, command, *args, **kwargs):
            selected = rewrite_command(command)
            Path(evidence).write_text(json.dumps(selected, indent=2)+'\n', encoding='utf8')
            return process_module.Popen(selected, *args, **kwargs)

    clone = types.FunctionType(original.__code__, {**original.__globals__, 'subprocess': Proxy()},
        original.__name__, original.__defaults__, original.__closure__)
    clone.__kwdefaults__ = original.__kwdefaults__
    return clone
