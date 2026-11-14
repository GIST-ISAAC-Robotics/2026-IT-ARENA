"""실제 launch 함수 AST를 실행하는 Windows 대역. native parser 검사는 별도다."""
import ast
from pathlib import Path
from types import SimpleNamespace as N

import pytest

A=Path(__file__).resolve().parent


class Action:
    def __init__(self,kind,*args,**kwargs):
        self.kind,self.args,self.kwargs=kind,args,kwargs


class LaunchConfiguration:
    def __init__(self,name):self.name=name
    def perform(self,context):return context.launch_configurations[self.name]


class ParameterValue:
    def __init__(self,value,*,value_type):self.value,self.value_type=value,value_type


def generate(name,before=False):
    path=A/(name+('.before' if before else ''))
    tree=ast.parse(path.read_bytes())
    tree.body=[n for n in tree.body if not isinstance(n,(ast.Import,ast.ImportFrom))]
    scope=dict(Path=Path,get_package_share_directory=lambda package:'/fixture/'+package,
        LaunchDescription=lambda actions:actions,LaunchConfiguration=LaunchConfiguration,ParameterValue=ParameterValue)
    for kind in ['DeclareLaunchArgument','GroupAction','IncludeLaunchDescription','PythonLaunchDescriptionSource','SetParameter','OpaqueFunction']:
        scope[kind]=lambda *args,_kind=kind,**kwargs:Action(_kind,*args,**kwargs)
    exec(compile(tree,str(path),'exec'),scope)
    return scope['generate_launch_description']()


def evaluate(actions,trace=None):
    context=N(launch_configurations={} if trace is None else {'g1_trace_path':trace})
    parameters={}
    def value(item):
        if isinstance(item,LaunchConfiguration):return item.perform(context)
        if isinstance(item,ParameterValue):return item.value_type(value(item.value))
        return item
    def visit(items):
        for action in items:
            args,kw=action.args,action.kwargs
            if action.kind=='GroupAction':
                assert kw['scoped'] is False
                visit(kw['actions'])
            elif action.kind=='DeclareLaunchArgument':context.launch_configurations.setdefault(args[0],kw['default_value'])
            elif action.kind=='OpaqueFunction':visit(kw['function'](context))
            elif action.kind=='SetParameter':parameters[kw['name']]=value(kw['value'])
            elif action.kind=='IncludeLaunchDescription':pass
            else:raise AssertionError(action.kind)
    visit(actions)
    return parameters,context.launch_configurations


@pytest.mark.parametrize('name',['g1_follow.launch.py','r0_plus.launch.py'])
@pytest.mark.parametrize('trace',[None,''])
def test_default_and_explicit_empty_trace_omit_only_trace_parameter(name,trace):
    old,old_args=evaluate(generate(name,before=True),trace)
    new,new_args=evaluate(generate(name),trace)
    assert old.pop('g1_trace_path')==''  # 원본은 실패 CLI g1_trace_path:=를 만들 수 있다.
    assert new==old and new_args==old_args
    assert 'g1_trace_path' not in new


@pytest.mark.parametrize('name',['g1_follow.launch.py','r0_plus.launch.py'])
@pytest.mark.parametrize('trace',['/tmp/trace.jsonl','/tmp/trace with spaces.jsonl',"/tmp/trace's name.jsonl"])
def test_nonempty_trace_preserves_all_parameters_arguments_and_value_types(name,trace):
    old,old_args=evaluate(generate(name,before=True),trace)
    new,new_args=evaluate(generate(name),trace)
    assert new==old and new_args==old_args
    assert new['g1_trace_path']==trace
    assert {k:type(v) for k,v in old.items()}=={k:type(v) for k,v in new.items()}


def test_actual_consumer_empty_default_and_no_trace_creation():
    root=A.parents[4]
    path=root/'src/arena_autonomy/arena_autonomy/local_pursuit.py'
    tree=ast.parse(path.read_bytes())
    node=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='LocalPursuit')
    declarations=[n for n in ast.walk(node) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)
        and n.func.attr=='declare_parameter' and n.args and isinstance(n.args[0],ast.Constant)
        and n.args[0].value=='g1_trace_path']
    assert len(declarations)==1 and declarations[0].args[1].value==''
    method=next(n for n in node.body if isinstance(n,ast.FunctionDef) and n.name=='create_follow_trace')
    scope={}
    exec(compile(ast.Module(body=[method],type_ignores=[]),str(path),'exec'),scope)
    assert scope['create_follow_trace'](N(),'') is None
