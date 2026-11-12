"""단일 local_pursuit 노드에만 실험 prefix를 넣는다."""
import json,shlex,sys,types
from pathlib import Path

def instrument_start(original,evidence):
    module=original.__globals__['subprocess'];entry=Path(__file__).with_name('node_entry.py')
    class Proxy:
        def __getattr__(self,name):return getattr(module,name)
        def Popen(self,command,*args,**kwargs):
            assert command[:3]==['ros2','launch','--noninteractive']
            command=[*command[:2],'--launch-prefix',shlex.join([sys.executable,str(entry)]),
                '--launch-prefix-filter','.*local_pursuit.*',*command[2:]]
            with Path(evidence).open('x') as f:json.dump(command,f,indent=2)
            return module.Popen(command,*args,**kwargs)
    clone=types.FunctionType(original.__code__,{**original.__globals__,'subprocess':Proxy()},original.__name__,original.__defaults__,original.__closure__)
    clone.__kwdefaults__=original.__kwdefaults__
    return clone
