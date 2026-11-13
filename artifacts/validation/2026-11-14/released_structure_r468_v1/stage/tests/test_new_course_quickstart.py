"""새 코스 단독 이름 있는 프리셋과 기존 run_current_solo.py 행위를 구분한다."""
import json
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT/'scripts/run_new_course_solo.py'
sys.path.insert(0, str(ROOT/'scripts'))
import run_new_course_solo as quick  # noqa: E402


def _run(*args, cwd=ROOT):
    return subprocess.run([sys.executable, '-B', str(SCRIPT), *args], cwd=cwd, capture_output=True, text=True)


def test_preset_matches_every_r25_registered_option_and_value(tmp_path):
    args = quick.preset_args('new-course-solo', tmp_path/'out')
    assert quick.compare_with_r25(args) == []
    head, table = quick.normalized(args)
    assert head == 'scripts/validate_basic_autonomy.py'
    assert table['--track'] == 'official_v2026_11_06'
    assert table['--local-sensor-config'] == 'src/arena_description/config/b0390_c1_jr60p20_sim.yaml'
    assert args[args.index('--output')+1] == str((tmp_path/'out').resolve())


def test_comparison_reports_changed_missing_and_duplicate_options(tmp_path):
    args = quick.preset_args('new-course-solo', tmp_path/'out')
    changed = list(args)
    changed[changed.index('--max-sim-seconds')+1] = '120'
    assert quick.compare_with_r25(changed) == ['--max-sim-seconds: r25=110 현재=120']
    missing = [a for a in args if a != '--g1-solo-corner-approach']
    assert quick.compare_with_r25(missing) == ['--g1-solo-corner-approach: r25=None 현재=<없음>']
    with pytest.raises(ValueError, match='중복'):
        quick.normalized([*args, '--video'])


@pytest.mark.parametrize('key', ['--output', '--local-sensor-config'])
def test_path_option_without_value_is_an_error_not_hidden(tmp_path, key):
    args = quick.preset_args('new-course-solo', tmp_path/'out')
    i = args.index(key)
    for broken in (args[:i+1] + args[i+2:], args[:i+1]):  # 다음 옵션 앞·맨 끝에서 값 누락
        with pytest.raises(ValueError, match='값이 없는 경로 옵션'):
            quick.normalized(broken)
        with pytest.raises(ValueError, match='값이 없는 경로 옵션'):
            quick.compare_with_r25(broken)


def test_relative_output_is_resolved_from_caller_cwd(tmp_path):
    printed = _run('rel_out', '--preset', 'new-course-solo', '--print-args', cwd=tmp_path)
    assert printed.returncode == 0, printed.stderr
    args = json.loads(printed.stdout)
    assert args[args.index('--output')+1] == str((tmp_path/'rel_out').resolve())
    assert not (tmp_path/'rel_out').exists()
    (tmp_path/'taken').mkdir()
    assert _run('taken', '--preset', 'new-course-solo', '--print-args', cwd=tmp_path).returncode == 2


def test_run_executes_the_same_printed_args(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    calls = []
    monkeypatch.setattr(quick.subprocess, 'call', lambda command, cwd: calls.append((command, cwd)) or 0)
    monkeypatch.setattr(sys, 'argv', ['run_new_course_solo.py', 'rel_run', '--preset', 'new-course-solo'])
    assert quick.main() == 0
    expected = quick.preset_args('new-course-solo', (tmp_path/'rel_run').resolve())
    assert calls == [([sys.executable, '-B', *expected], quick.ROOT)]


def test_cli_check_and_rejections(tmp_path):
    ok = _run(str(tmp_path/'new'), '--preset', 'new-course-solo', '--check-r25')
    assert ok.returncode == 0 and json.loads(ok.stdout)['differences'] == []
    assert _run(str(tmp_path/'new'), '--print-args').returncode == 2  # 프리셋 이름 필수
    assert _run(str(tmp_path/'new'), '--preset', 'new-course-solo', '--previous-jr50').returncode == 2
    assert _run(str(tmp_path/'new'), '--preset', 'new-course-solo', '--print-args', '--check-r25').returncode == 2
    assert not (tmp_path/'new').exists()


def test_previous_entry_point_keeps_old_defaults():
    proc = subprocess.run([sys.executable, '-B', str(ROOT/'scripts/run_current_solo.py'), '/tmp/old_default',
                           '--print-args'], cwd=ROOT, check=True, capture_output=True, text=True)
    args = json.loads(proc.stdout)
    assert '--track' not in args and '--g1-solo-straight-budget' not in args
    assert args[args.index('--local-sensor-config')+1].replace('\\', '/').endswith('b0183_c1_jr60p20_sim.yaml')
