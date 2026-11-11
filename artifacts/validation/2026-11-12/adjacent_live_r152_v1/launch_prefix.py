"""평가기의 기존 launch 명령에 등록한 단일 노드 prefix만 추가한다."""
import json
from pathlib import Path
import shlex
import sys
import types


def instrument_start(original, profile, evidence):
    module=original.__globals__['subprocess']
    entry=Path(__file__).with_name('node_entry.py')
    class ProcessProxy:
        def __getattr__(self,name):return getattr(module,name)
        def Popen(self,command,*args,**kwargs):
            if command[:3]!=['ros2','launch','--noninteractive']:
                raise ValueError('expected existing ros2 launch invocation')
            command=[*command[:2],'--launch-prefix',
                shlex.join([sys.executable,str(entry),str(profile)]),
                '--launch-prefix-filter','.*g1_g2_handoff_trial.*',*command[2:]]
            with Path(evidence).open('x',encoding='utf8') as output:
                json.dump(dict(command=command,scope='single controller delivery delay'),output,indent=2)
            return module.Popen(command,*args,**kwargs)
    clone=types.FunctionType(original.__code__,
        {**original.__globals__,'subprocess':ProcessProxy()},
        original.__name__,original.__defaults__,original.__closure__)
    clone.__kwdefaults__=original.__kwdefaults__
    return clone
