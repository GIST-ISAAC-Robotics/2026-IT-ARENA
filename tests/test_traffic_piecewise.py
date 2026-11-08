"""교통 CLI의 기존 직각 옵션 연결. 주행 결과를 대신하지 않는다."""
import pytest
from validate_g1_traffic import piecewise_arguments


def test_existing_traffic_launch_unchanged_when_off():
    assert piecewise_arguments(False, 'g1g2', True) == []
    assert piecewise_arguments(False, 'r0plus', False) == []


@pytest.mark.parametrize('mode', ['g1', 'g1g2'])
def test_existing_candidate_forwarded_without_other_changes(mode):
    assert piecewise_arguments(True, mode, True) == ['g1_piecewise_wall_trial:=true']


@pytest.mark.parametrize('mode,record', [('m3', True), ('r0plus', True), ('g1g2', False)])
def test_unregistered_pipeline_or_unrecorded_choice_rejected(mode, record):
    with pytest.raises(ValueError, match='recorded G1'):
        piecewise_arguments(True, mode, record)
