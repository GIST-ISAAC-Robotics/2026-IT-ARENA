"""trace OFF 빈 global parameter 두 곳만 stage에 수정한다. 공통은 읽기 전용."""
import ast
import copy
import difflib
import hashlib
import json
from pathlib import Path

A=Path(__file__).resolve().parent
ROOT=A.parents[4]
manifest={}
patch=[]
sha=lambda raw:hashlib.sha256(raw).hexdigest()
for name in ('g1_follow.launch.py','r0_plus.launch.py'):
    source=ROOT/'src/arena_bringup/launch'/name
    before=source.read_bytes()
    text=before.decode('utf-8')
    old_import='from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription'
    old_action="SetParameter(name='g1_trace_path', value=LaunchConfiguration('g1_trace_path'))"
    assert text.count(old_import)==1 and text.count(old_action)==1
    text=text.replace(old_import,old_import+', OpaqueFunction')
    helper=("def _trace_path_parameters(context):\n"
        "    # 빈 global parameter를 name:=로 보내지 않고 소비자의 기록 OFF 기본값을 쓴다.\n"
        "    if not LaunchConfiguration('g1_trace_path').perform(context):return []\n"
        "    return [SetParameter(name='g1_trace_path', value=LaunchConfiguration('g1_trace_path'))]\n\n\n")
    text=text.replace(old_action,"OpaqueFunction(function=_trace_path_parameters)")
    text=text.replace('def generate_launch_description():',helper+'def generate_launch_description():')
    ast.parse(text)
    after=text.encode('utf-8')
    for suffix,raw in [('.before',before),('',after)]:
        with (A/(name+suffix)).open('xb') as stream:stream.write(raw)
    normalized=copy.deepcopy(ast.parse(text))
    normalized.body=[n for n in normalized.body if not isinstance(n,ast.FunctionDef) or n.name!='_trace_path_parameters']
    for node in normalized.body:
        if isinstance(node,ast.ImportFrom) and node.module=='launch.actions':
            node.names=[n for n in node.names if n.name!='OpaqueFunction']
    class RestoreTrace(ast.NodeTransformer):
        def visit_Call(self,node):
            if isinstance(node.func,ast.Name) and node.func.id=='OpaqueFunction':
                return ast.parse(old_action,mode='eval').body
            return self.generic_visit(node)
    normalized=RestoreTrace().visit(normalized)
    assert ast.dump(normalized,include_attributes=False)==ast.dump(ast.parse(before),include_attributes=False)
    manifest[name]=dict(source=source.relative_to(ROOT).as_posix(),before_sha256=sha(before),
        stage_sha256=sha(after),other_launch_ast_identical=True)
    relative=source.relative_to(ROOT).as_posix()
    patch.extend(difflib.unified_diff(before.decode('utf-8').splitlines(True),text.splitlines(True),
        fromfile='a/'+relative,tofile='b/'+relative))
(A/'trace_optional_parameter.patch').write_text(''.join(patch),encoding='utf-8',newline='\n')
(A/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
print(json.dumps(manifest,indent=2))
