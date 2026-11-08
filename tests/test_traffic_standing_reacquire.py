"""기존 재획득을 교통 실행에서 명시 선택하며 실제 헤더와 대조한다."""
from pathlib import Path
import json
import pytest
import g1_handoff_eval as ev

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT/'artifacts/validation/2026-11-01/race_follow_pair_v1/stop_restart.json'


def test_only_requested_launch_adds_existing_parameter():
    launch = json.loads(CONFIG.read_bytes())['launch']
    off = ev.launch_arguments('g1g2', 'paced', launch, True)
    on = ev.launch_arguments('g1g2', 'paced', launch, True, standing_reacquire=True)
    assert on == ['g1_standing_wall_reacquire:=true'] + off
    with pytest.raises(ValueError):
        ev.launch_arguments('g1', None, launch, True, standing_reacquire=True)
    with pytest.raises(ValueError):
        ev.launch_arguments('g1g2', 'paced', launch, False, standing_reacquire=True)


@pytest.mark.parametrize('actual,requested', [(False, False), (False, True), (True, False), (True, True)])
def test_header_verifies_actual_setting_in_both_directions(tmp_path, actual, requested):
    p = tmp_path/'decisions.jsonl'
    header = dict(kind='header', data=dict(controller_mode='g1_g2_handoff_trial',
                  follow_config=dict(standing_wall_reacquire=actual), execution_config={}))
    p.write_text(json.dumps(header)+'\n', encoding='utf-8')
    assert ev.inspect_header(p, 'paced', standing_reacquire=requested)['checks']['standing_reacquire'] == (actual is requested)
