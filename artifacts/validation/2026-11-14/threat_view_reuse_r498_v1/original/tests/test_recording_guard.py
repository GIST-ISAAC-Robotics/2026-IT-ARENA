import math
from pathlib import Path
import sys
from unittest.mock import Mock
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from recording_guard import (DEFAULT_MAX_RAW_GIB, DEFAULT_MAX_WALL_SECONDS,
                             recorder_arguments, require_recorder_running, validate_limits)


def test_defaults_preserved():
    assert (DEFAULT_MAX_RAW_GIB, DEFAULT_MAX_WALL_SECONDS) == (6.,1200.)


@pytest.mark.parametrize('raw,wall',[(.01,1.),(6.,1200.),(16.,2520.),(20.,7200.)])
def test_explicit_limits_forwarded(raw,wall):
    assert recorder_arguments(raw,wall) == ['--max-raw-gib',str(raw),'--max-wall-seconds',str(wall)]


@pytest.mark.parametrize('raw,wall',[(0,1200),(20.01,1200),(6,0),(6,7201),
    (math.nan,1200),(6,math.nan),(math.inf,1200),(6,math.inf)])
def test_invalid_limits_fail_before_start(raw,wall):
    with pytest.raises(ValueError): validate_limits(raw,wall)


def test_disabled_or_running_recorder_is_ok():
    require_recorder_running(None)
    process = Mock()
    process.poll.return_value = None
    require_recorder_running(process)
    process.poll.assert_called_once_with()


@pytest.mark.parametrize('code',[0,1,-2,-9])
def test_early_recorder_exit_is_not_silent(code):
    process = Mock()
    process.poll.return_value = code
    with pytest.raises(RuntimeError,match='recorder exited before requested stop'):
        require_recorder_running(process)
