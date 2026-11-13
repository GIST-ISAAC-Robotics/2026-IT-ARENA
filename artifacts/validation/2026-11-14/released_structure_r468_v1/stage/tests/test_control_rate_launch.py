"""ROS launch 경계에서 제어 주기를 덮어쓰던 r91 오류의 실행 없는 재현."""
import importlib.util
from pathlib import Path
import pytest

pytest.importorskip('launch')
pytest.importorskip('launch_ros')
ROOT = Path(__file__).resolve().parents[1]
PATH = ROOT / 'artifacts/validation/2026-11-11/control_10hz_r91_v1/check_launch_resolution.py'
spec = importlib.util.spec_from_file_location('rate_launch_diagnostic', PATH)
diagnostic = importlib.util.module_from_spec(spec)
spec.loader.exec_module(diagnostic)


def test_default_and_explicit_50hz_launch_remain_equivalent():
    default, explicit = diagnostic.resolve(), diagnostic.resolve(50.)
    assert float(default['autonomy_control_rate_hz']) == 50.
    assert default == explicit


def test_explicit_10hz_reaches_simulation_without_other_changes():
    base, selected = diagnostic.resolve(), diagnostic.resolve(10.)
    assert float(selected['autonomy_control_rate_hz']) == 10.
    differences = {k for k in base.keys() | selected.keys() if base.get(k) != selected.get(k)}
    assert differences == {'autonomy_control_rate_hz'}
