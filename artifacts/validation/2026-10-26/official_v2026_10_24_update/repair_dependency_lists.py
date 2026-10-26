"""현재 정적 import 폐쇄에 맞춰 명시적인 이관/소스 보존 목록을 보충한다."""
import ast
import importlib.util
from pathlib import Path
import pprint

ROOT = Path(__file__).resolve().parents[4]

def closure(modules):
    pending = list(modules)
    result = set()
    while pending:
        name = pending.pop()
        if name in result:
            continue
        result.add(name)
        package, module = name.split('.')
        tree = ast.parse((ROOT/f'src/{package}/{package}/{module}.py').read_text(encoding='utf-8'))
        for node in ast.walk(tree):
            names = [node.module] if isinstance(node, ast.ImportFrom) and node.module else []
            if isinstance(node, ast.Import):
                names += [alias.name for alias in node.names]
            pending += [item for item in names if item.startswith(('arena_autonomy.', 'arena_vehicle_interface.'))]
    return result

for filename, variable in [('scripts/profile_follow_trace.py','SNAPSHOT_MODULES'), ('scripts/validate_pipeline_faults_ros.py','MODULES')]:
    p = ROOT/filename
    text = p.read_text(encoding='utf-8')
    node = next(n for n in ast.parse(text).body if isinstance(n, ast.Assign) and any(isinstance(t,ast.Name) and t.id == variable for t in n.targets))
    original = ast.literal_eval(node.value)
    modules = closure({f'{package}.{name}' for package,names in original.items() for name in names})
    updated = {package: tuple(sorted(n.split('.')[1] for n in modules if n.startswith(package+'.'))) for package in original}
    lines = text.splitlines(keepends=True)
    lines[node.lineno-1:node.end_lineno] = [variable+' = '+pprint.pformat(updated,width=100,sort_dicts=False)+'\n']
    p.write_text(''.join(lines),encoding='utf-8',newline='\n')
    print(filename, sorted(modules-{f'{p}.{n}' for p,ns in original.items() for n in ns}))

p = ROOT/'scripts/prepare_jetson_bundle.py'
spec = importlib.util.spec_from_file_location('bundle_repair',p)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
paths = set(module.FOLLOW_PROFILE_TOOLS)
names = {path.split('/')[1]+'.'+Path(path).stem for path in paths if path.startswith('src/')}
missing = sorted(f"src/{n.split('.')[0]}/{n.split('.')[0]}/{n.split('.')[1]}.py" for n in closure(names))
missing = [path for path in missing if path not in paths]
text = p.read_text(encoding='utf-8')
node = next(n for n in ast.parse(text).body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id == 'FOLLOW_PROFILE_TOOLS' for t in n.targets))
lines = text.splitlines(keepends=True)
lines[node.end_lineno-1:node.end_lineno-1] = [f'    {path!r},\n' for path in missing]
p.write_text(''.join(lines),encoding='utf-8',newline='\n')
print('bundle added',missing)
