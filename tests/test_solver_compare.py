"""선택형 solver 시험만 수정하며 기존 기본 실행은 그대로 둔다."""
import sys
from types import SimpleNamespace
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from probe_solver_compare import configure_solver, probe_arguments
import probe_solver_compare as wrapper


def world(tmp_path, content='<max_step_size>0.001</max_step_size><dart><collision_detector>ode</collision_detector></dart>'):
    path = tmp_path/'world.sdf'
    path.write_text(f'<sdf><world name="w"><physics name="p">{content}</physics><model name="keep"/></world></sdf>')
    return path


@pytest.mark.parametrize('solver', ['dantzig', 'pgs'])
def test_only_solver_added(tmp_path, solver):
    path = world(tmp_path)
    before = ET.parse(path)
    configure_solver(path, solver)
    after = ET.parse(path)
    dart = after.find('world/physics/dart')
    assert dart.findtext('solver/solver_type') == solver
    dart.remove(dart.find('solver'))
    assert ET.tostring(after.getroot()) == ET.tostring(before.getroot())
    with pytest.raises(ValueError, match='preexisting'):
        configure_solver(path, solver)


@pytest.mark.parametrize('content', ['<max_step_size>0.0005</max_step_size>',
    '<max_step_size>0.001</max_step_size><dart/><dart/>'])
def test_reject_other_step_or_ambiguous_dart(tmp_path, content):
    path = world(tmp_path, content)
    before = path.read_bytes()
    with pytest.raises(ValueError):
        configure_solver(path, 'pgs')
    assert path.read_bytes() == before


def test_invalid_name_and_fixed_input(tmp_path):
    path = world(tmp_path)
    with pytest.raises(ValueError):
        configure_solver(path, 'unregistered')
    args = probe_arguments(tmp_path/'out')
    assert '--evaluation-scheduled-apply' in args
    assert '--physics-step' not in args
    assert args[args.index('--surface')+1] == 'flat'
    assert args[args.index('--zero-speed-mode')+1] == 'passive_damping'


def test_child_failure_propagated_and_source_registered(tmp_path, monkeypatch):
    fake = SimpleNamespace(SOURCE_FILES=['old.py'], flat_world=lambda a, b: 0, main=lambda: 7)
    monkeypatch.setitem(sys.modules, 'probe_braking_contact', fake)
    monkeypatch.setattr(wrapper, 'ROOT', ROOT)
    monkeypatch.setattr(sys, 'argv', ['wrapper', '--solver', 'pgs', '--output',
        str(ROOT/'artifacts/__unit_test_nonexistent_solver_output__')])
    assert wrapper.main() == 7
    assert fake.SOURCE_FILES == ['old.py', 'scripts/probe_solver_compare.py']
