"""기존 launch에 공통 선택 옵션만 전달한다. 평가 지연/노드 monkeypatch 없음."""
import json
from pathlib import Path
import types


def instrument_start(original, evidence):
    module = original.__globals__['subprocess']
    class ProcessProxy:
        def __getattr__(self, name):return getattr(module,name)
        def Popen(self, command, *args, **kwargs):
            if command[:3] != ['ros2','launch','--noninteractive']:
                raise ValueError('expected existing ros2 launch invocation')
            command = [*command, 'g1_current_path_recheck:=selected_v1',
                'g1_submission_policy:=fresh_native_v1', 'g1_guard_observation_policy:=selected_r181',
                'g1_released_body_gap_trial:=true', 'g1_expiry_route_reference_trial:=true',
                'g1_oblique_gap_trial:=true', 'g1_current_recheck_execution:=worker_event_v1']
            with Path(evidence).open('x',encoding='utf8') as output:
                json.dump(dict(command=command,scope='common worker_event_v1; no evaluation delay'),output,indent=2)
            return module.Popen(command,*args,**kwargs)
    clone=types.FunctionType(original.__code__,{**original.__globals__,'subprocess':ProcessProxy()},
        original.__name__,original.__defaults__,original.__closure__)
    clone.__kwdefaults__=original.__kwdefaults__
    return clone
