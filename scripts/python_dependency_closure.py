#!/usr/bin/env python3
"""명시 진입점에서 프로젝트 내부 Python import를 정적으로 닫는다. 대상 코드는 실행하지 않는다.

모듈 최상단·함수 내부 지연 import·상대 import·패키지 __init__을 모두 따라간다. 내부 패키지
이름이거나 탐색 루트에 있는 모듈 이름인데 파일이 없으면 MissingInternalDependency로 실패하고,
그 밖의 이름은 표준/외부 의존성으로만 분류한다(설치 여부는 확인하지 않는다).
"""
import ast
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
INTERNAL_PACKAGES = ('arena_autonomy', 'arena_vehicle_interface')
PACKAGE_ROOTS = tuple(f'src/{name}' for name in INTERNAL_PACKAGES)
SCRIPT_ROOTS = ('scripts',)
TEST_ROOTS = ('tests',)
CLOSURE_VERSION = 1


class MissingInternalDependency(ImportError):
    pass


class _Source:
    """저장소 폴더 또는 {상대 경로: bytes}(압축 묶음 내부)에서 같은 규칙으로 찾는다."""
    def __init__(self, base, members):
        self.base = None if members is not None else Path(base).resolve()
        self.members = members

    def is_file(self, rel):
        return rel in self.members if self.members is not None else (self.base/rel).is_file()

    def read(self, rel):
        return self.members[rel] if self.members is not None else (self.base/rel).read_bytes()


def _imports(tree, module, is_package):
    """(절대 이름, from 목록의 이름인지, 줄). ast.walk라서 함수 내부 지연 import도 포함한다."""
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                yield alias.name, False, node.lineno
        elif isinstance(node, ast.ImportFrom):
            if node.level:
                package = module.split('.') if is_package else module.split('.')[:-1]
                if not module or not package or node.level > len(package):
                    raise MissingInternalDependency(f'relative import outside package at line {node.lineno}')
                package = package[:len(package)-node.level+1]
                name = '.'.join(package+([node.module] if node.module else []))
            else:
                name = node.module
            yield name, False, node.lineno
            for alias in node.names:
                if alias.name != '*':
                    yield f'{name}.{alias.name}', True, node.lineno


def _bound_names(tree):
    """모듈 최상위(if/try 포함)에서 정의되는 이름. star import가 있으면 None(판단 불가)."""
    names, body = set(), list(tree.body)
    while body:
        node = body.pop()
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            names.add(node.name)
        elif isinstance(node, (ast.Assign, ast.AnnAssign, ast.AugAssign)):
            for target in node.targets if isinstance(node, ast.Assign) else [node.target]:
                names.update(n.id for n in ast.walk(target) if isinstance(n, ast.Name))
        elif isinstance(node, (ast.Import, ast.ImportFrom)):
            for alias in node.names:
                if alias.name == '*':
                    return None
                names.add(alias.asname or alias.name.split('.')[0])
        elif isinstance(node, (ast.If, ast.Try, ast.With)):
            body += node.body+getattr(node, 'orelse', [])+getattr(node, 'finalbody', [])
            for handler in getattr(node, 'handlers', []):
                body += handler.body
    return names


def collect(entries, base=ROOT, roots=PACKAGE_ROOTS+SCRIPT_ROOTS, internal_packages=INTERNAL_PACKAGES,
            members=None):
    """entries(저장소 상대 POSIX 경로)의 내부 import 폐포.

    반환 files는 진입점·내부 모듈·필요한 패키지 __init__의 정렬 목록, external은 최상위 이름별
    'stdlib'/'third_party' 분류다. members를 주면 디스크 대신 그 내용만 사용한다.
    """
    source = _Source(base, members)
    roots = tuple(root.rstrip('/') for root in roots)
    stdlib = getattr(sys, 'stdlib_module_names', frozenset())

    def module_file(root, name):
        stem = f'{root}/' + name.replace('.', '/')
        for candidate in (stem+'.py', stem+'/__init__.py'):
            if source.is_file(candidate):
                return candidate
        return None

    def find(name):
        for root in roots:
            found = module_file(root, name)
            if found is not None:
                return root, found
        return None

    def internal(top):
        return top in internal_packages or find(top) is not None

    def module_name(rel):
        for root in roots:
            if rel.startswith(root+'/'):
                parts = rel[len(root)+1:-len('.py')].split('/')
                is_package = parts[-1] == '__init__'
                return '.'.join(parts[:-1] if is_package else parts), is_package
        raise MissingInternalDependency(f'entry outside module roots: {rel}')

    files, modules, external, pending = set(), {}, {}, []
    for entry in entries:
        entry = Path(entry).as_posix()
        if not source.is_file(entry):
            raise MissingInternalDependency(f'entry not found: {entry}')
        pending.append(entry)
    while pending:
        rel = pending.pop()
        if rel in files:
            continue
        files.add(rel)
        if not rel.endswith('.py'):
            continue
        module, is_package = module_name(rel)
        if module:
            modules[module] = rel
        tree = ast.parse(source.read(rel), filename=rel)
        for name, from_list, line in _imports(tree, module, is_package):
            top = name.split('.')[0]
            if not internal(top):
                if not from_list:
                    external[top] = 'stdlib' if top in stdlib else 'third_party'
                continue
            found = find(name)
            if found is None:
                # from 모듈 import 함수/클래스는 속성이다. 패키지 __init__이면 실제 정의된 이름인지 본다.
                parent = find(name.rsplit('.', 1)[0]) if from_list else None
                if parent is not None:
                    if not parent[1].endswith('/__init__.py'):
                        continue
                    bound = _bound_names(ast.parse(source.read(parent[1]), filename=parent[1]))
                    if bound is None or name.rsplit('.', 1)[1] in bound:
                        continue
                raise MissingInternalDependency(f'{rel}:{line} imports missing internal module {name}')
            root, target = found
            parts = name.split('.')
            for depth in range(1, len(parts)):  # 하위 모듈 import는 상위 패키지 __init__도 실행한다.
                init = module_file(root, '.'.join(parts[:depth]))
                if init is not None:
                    pending.append(init)
            pending.append(target)
    clash = sorted(name for name in modules if name.split('.')[0] in stdlib)
    if clash:
        raise ValueError(f'project module shadows standard library: {clash}')
    return dict(files=sorted(files), modules=dict(sorted(modules.items())),
                external=dict(sorted(external.items())))


def third_party(closure):
    return sorted(name for name, kind in closure['external'].items() if kind == 'third_party')


def package_modules(closure, packages=INTERNAL_PACKAGES):
    """{패키지: (하위 모듈 이름, ...)}. 패키지 __init__ 자체는 이름 목록에 넣지 않는다."""
    return {package: tuple(sorted(name.split('.', 1)[1] for name in closure['modules']
                                  if name.startswith(package+'.') and name.count('.') == 1))
            for package in packages}


if __name__ == '__main__':
    import json
    print(json.dumps(collect(sys.argv[1:]), ensure_ascii=False, indent=2))
