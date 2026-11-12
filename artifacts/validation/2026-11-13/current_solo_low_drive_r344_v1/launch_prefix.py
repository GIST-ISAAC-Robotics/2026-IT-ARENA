"""확정 구형 지주와 선택형 r336 노드만 연결하는 시험 launch."""
import json
from pathlib import Path
import shlex
import sys
import types

A = Path(__file__).resolve().parent


def instrument_start(original, evidence):
    process_module = original.__globals__['subprocess']
    entry = A/'node_entry.py'
    profile = entry.with_name('profile.json')

    class Proxy:
        def __getattr__(self, name):
            return getattr(process_module, name)

        def Popen(self, command, *args, **kwargs):
            assert command[:5] == ['ros2', 'launch', '--noninteractive', 'arena_bringup', 'g1_follow.launch.py']
            command = [*command[:3], str(A/'course.launch.py'), *command[5:]]
            command = [*command[:2], '--launch-prefix', shlex.join([sys.executable, str(entry), str(profile)]),
                '--launch-prefix-filter', '.*g1_g2_handoff_trial.*', *command[2:],
                'g1_current_path_recheck:=selected_v1', 'g1_submission_policy:=fresh_native_v1',
                'g1_guard_observation_policy:=selected_r181', 'g1_released_body_gap_trial:=true',
                'g1_expiry_route_reference_trial:=true', 'g1_oblique_gap_trial:=true',
                'g1_current_recheck_execution:=worker_event_v1', 'g1_numeric_execution:=native_v1']
            Path(evidence).write_text(json.dumps(command, indent=2)+'\n')
            return process_module.Popen(command, *args, **kwargs)

    clone = types.FunctionType(original.__code__, {**original.__globals__, 'subprocess': Proxy()},
        original.__name__, original.__defaults__, original.__closure__)
    clone.__kwdefaults__ = original.__kwdefaults__
    return clone
