"""동결 준비만 수행하는 stdlib 도구. 시험 실행/공통 쓰기 없음."""
import ast
import difflib
import hashlib
import json
from pathlib import Path

A = Path(__file__).resolve().parent
ROOT = A.parents[3]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, value):
    with path.open('x', encoding='utf8', newline='\n') as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False)
        stream.write('\n')


def main():
    staged = sorted((A/'staged').rglob('*.py'))
    assert len(staged) == 8
    diff = []
    links = []
    for path in staged:
        relative = path.relative_to(A/'staged').as_posix()
        original = ROOT/relative
        diff.extend(difflib.unified_diff(
            original.read_text(encoding='utf8').splitlines(keepends=True) if original.exists() else [],
            path.read_text(encoding='utf8').splitlines(keepends=True),
            fromfile='a/'+relative if original.exists() else '/dev/null', tofile='b/'+relative))
        links.append(dict(path=relative, base_sha256=digest(original) if original.exists() else None,
                          staged_sha256=digest(path)))
    with (A/'proposed.patch').open('x', encoding='utf8', newline='\n') as stream:
        stream.writelines(diff)
    python = sorted(A.rglob('*.py'))
    for path in python:
        ast.parse(path.read_text(encoding='utf8'), filename=str(path))
    write(A/'static_preparation.json', dict(status='prepared_not_executed',
        python_ast_files=len(python), staged_files=links,
        r414_partial_sha256='576966260591adf169c426f33c921df0586ca75f31e9e973c69be94b2c268425',
        constraints=['no common writes', 'no WSL/ROS/Gazebo/Jetson execution',
                     'no speed/physics/TTL/current-recheck algorithm changes']))
    inputs = set()
    for folder in ('src/arena_autonomy/arena_autonomy', 'src/arena_vehicle_interface/arena_vehicle_interface'):
        inputs.update((ROOT/folder).rglob('*.py'))
        inputs.update((ROOT/folder).rglob('*.so'))
    inputs.update((ROOT/'src/arena_bringup/launch').glob('*.launch.py'))
    for relative in ('runtime_launch_profile_r401_v1/prepared_v2/expected_header.json',
                     'runtime_launch_profile_r401_v1/prepared_v2/runtime_profile.yaml',
                     'crop_edge_component_r414_v1/staged/arena_autonomy/partial_observation.py',
                     'crop_edge_component_r414_v1/verify_native_v2/report.json',
                     'crop_edge_component_r414_v1/verify_native_v2/off_report.json'):
        inputs.add(A.parent/relative)
    # 선택 회귀의 test_* / scripts 모듈 의존만 재귀적으로 기록한다.
    import run_verify
    pending = [ROOT/'tests'/name for name in run_verify.REGRESSION]
    visited = set()
    while pending:
        path = pending.pop()
        if path in visited:
            continue
        visited.add(path); inputs.add(path)
        tree = ast.parse(path.read_text(encoding='utf8'))
        for item in ast.walk(tree):
            names = ([item.module] if isinstance(item, ast.ImportFrom) and item.module else
                     [alias.name for alias in item.names] if isinstance(item, ast.Import) else [])
            for name in names:
                if '.' in name:
                    continue
                for folder in ('tests', 'scripts'):
                    dependency = ROOT/folder/(name+'.py')
                    if dependency.exists() and dependency not in visited:
                        pending.append(dependency)
    inputs.update(path for path in A.rglob('*') if path.is_file() and '__pycache__' not in path.parts)
    write(A/'manifest.json', dict(schema='r417_prepared_v1',
        purpose='staged common connection and bounded configuration regression; no live drive',
        files=[dict(path=path.relative_to(ROOT).as_posix(), sha256=digest(path), bytes=path.stat().st_size)
               for path in sorted(inputs)]))
    print(json.dumps(dict(manifest_sha256=digest(A/'manifest.json'), files=len(inputs),
                         prepared_python_ast=len(python), staged=len(staged))))


if __name__ == '__main__':
    main()
