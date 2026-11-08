"""정적 내부 import 폐포: 지연·상대 import, 패키지 __init__, 내부 누락과 외부 의존성 구분."""
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from python_dependency_closure import (MissingInternalDependency, collect, package_modules,  # noqa: E402
                                       third_party)

PKG = 'src/arena_autonomy/arena_autonomy'


def tree(tmp_path, files):
    for name, text in files.items():
        path = tmp_path/name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding='utf-8')
    return tmp_path


def sample(tmp_path, **extra):
    return tree(tmp_path, {
        f'{PKG}/__init__.py': '',
        f'{PKG}/top.py': 'import json\nimport numpy as np\nfrom arena_autonomy.mid import Thing\n'
                         'def later():\n    from arena_autonomy import lazy\n    return lazy\n',
        f'{PKG}/mid.py': 'from .leaf import value\nfrom . import sibling\nThing = value\n',
        f'{PKG}/leaf.py': 'value = 1\n',
        f'{PKG}/sibling.py': 'import collections\n',
        f'{PKG}/lazy.py': 'try:\n    import shapely\nexcept ImportError:\n    shapely = None\n',
        f'{PKG}/unused.py': 'import rclpy\n',
        'scripts/entry.py': 'import helper\nfrom arena_autonomy.top import later\n',
        'scripts/helper.py': 'import hashlib\n',
        **extra})


def test_lazy_relative_and_package_imports_are_closed_without_unused_modules(tmp_path):
    result = collect(['scripts/entry.py'], base=sample(tmp_path))
    assert result['files'] == sorted(['scripts/entry.py', 'scripts/helper.py', f'{PKG}/__init__.py',
        f'{PKG}/top.py', f'{PKG}/mid.py', f'{PKG}/leaf.py', f'{PKG}/sibling.py', f'{PKG}/lazy.py'])
    assert package_modules(result)['arena_autonomy'] == ('lazy', 'leaf', 'mid', 'sibling', 'top')
    # 외부 이름은 분류만 하고 설치 여부·실행은 확인하지 않는다. 사용하지 않는 rclpy는 들어오지 않는다.
    assert third_party(result) == ['numpy', 'shapely']
    assert result['external']['json'] == 'stdlib' and 'rclpy' not in result['external']


@pytest.mark.parametrize('line', ['from arena_autonomy.gone import x', 'import arena_autonomy.gone',
                                  'from arena_autonomy import gone_module', 'from .gone import x'])
def test_missing_internal_module_fails_explicitly(tmp_path, line):
    base = sample(tmp_path, **{f'{PKG}/leaf.py': f'value = 1\ndef f():\n    {line}\n'})
    with pytest.raises(MissingInternalDependency, match='leaf.py:3'):
        collect(['scripts/entry.py'], base=base)


def test_package_attribute_import_is_not_a_missing_module(tmp_path):
    base = sample(tmp_path, **{f'{PKG}/__init__.py': 'VERSION = 1\n',
                               f'{PKG}/leaf.py': 'from arena_autonomy import VERSION as value\n'})
    assert f'{PKG}/leaf.py' in collect(['scripts/entry.py'], base=base)['files']


def test_relative_import_outside_package_and_missing_entry_fail(tmp_path):
    base = sample(tmp_path, **{f'{PKG}/leaf.py': 'from ... import x\n'})
    with pytest.raises(MissingInternalDependency, match='relative'):
        collect(['scripts/entry.py'], base=base)
    with pytest.raises(MissingInternalDependency, match='entry not found'):
        collect(['scripts/absent.py'], base=base)


def test_project_module_shadowing_standard_library_is_rejected(tmp_path):
    base = sample(tmp_path, **{'scripts/json.py': '', 'scripts/helper.py': 'import json\n'})
    with pytest.raises(ValueError, match='shadows'):
        collect(['scripts/entry.py'], base=base)


def test_archive_members_give_same_closure_as_disk(tmp_path):
    base = sample(tmp_path)
    disk = collect(['scripts/entry.py'], base=base)
    members = {p.relative_to(base).as_posix(): p.read_bytes() for p in base.rglob('*.py')}
    assert collect(['scripts/entry.py'], members=members) == disk
    del members[f'{PKG}/leaf.py']
    with pytest.raises(MissingInternalDependency, match='leaf'):
        collect(['scripts/entry.py'], members=members)
