"""실제 비교의 구성 혼입과 제어기 오분류를 막는 평가 경계."""
from pathlib import Path
import json
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import g1_handoff_eval as handoff
import g1_traffic_eval as ev

CONFIG = ROOT / 'artifacts/validation/2026-11-01/race_follow_pair_v1/stop_restart.json'


def test_pair_has_one_difference_and_keeps_registered_contact_and_restart_gates():
    config = json.loads(CONFIG.read_text())
    original = json.loads((ROOT / 'config/tests/g1_traffic_v1.json').read_text())
    assert config['actor'] == original['actor'] and config['acceptance'] == original['acceptance']
    assert config['cases'] == [next(c for c in original['cases'] if c['name']=='stop_restart')]
    fixed, moving = (handoff.launch_arguments('g1g2', arm, config['launch'], True)
                     for arm in ('fixed', 'moving'))
    assert [(a,b) for a,b in zip(fixed,moving,strict=True) if a!=b] == [
        ('evaluation_handoff_reserve:=fixed', 'evaluation_handoff_reserve:=moving')]
    with pytest.raises(ValueError):
        handoff.launch_arguments('g1g2','moving',dict(config['launch'],speed_profile='cautious'),True)
    with pytest.raises(ValueError):
        handoff.launch_arguments('g1g2','moving',config['launch'],False)
    assert handoff.launch_arguments('g1',None,config['launch'],True) == []


def test_ordinary_controller_does_not_count_as_g2_or_the_reverse():
    def status(mode, **extra):
        return dict(mode=mode, behavior='FOLLOW', passing_enabled=False, **extra)
    a=ev.controller_mode_summary([status('g1_follow')])
    b=ev.controller_mode_summary([status('g1_g2_handoff_trial',simulation_trial_only=True)])
    def matches(summary, mode):
        return ev.finalize_verdict({'evaluation':{'controller_mode':summary}},mode)['integrity']['controller_mode_matches']
    assert matches(a,'g1') and not matches(a,'g1g2')
    assert matches(b,'g1g2') and not matches(b,'g1') and not matches(b,'r0')
    assert not matches(ev.controller_mode_summary([status('g1_g2_handoff_trial')]),'g1g2')


def test_claimed_mode_without_execution_header_is_not_evidence(tmp_path):
    first = {'kind':'header','data':{'controller_mode':'g1_g2_handoff_trial','follow_config':{},
             'execution_config':{'moving_follow_reserve':False}}}
    path = tmp_path / 'decisions.jsonl'
    path.write_text(json.dumps(first)+'\n')
    wrong = handoff.inspect_header(path,'moving')
    assert not wrong['passed'] and not wrong['checks']['reserve'] and not wrong['checks']['normal_path']


@pytest.mark.parametrize('expected,flag,agrees',[
    ('closing',None,False), ('closing',True,True), ('moving',True,False), ('moving',None,True)])
def test_closing_trial_cannot_be_mistaken_for_the_previous_moving_trial(tmp_path,expected,flag,agrees):
    ex={'moving_follow_reserve':True}
    if flag is not None:
        ex['closing_follow_reserve']=flag
    path=tmp_path/'header.jsonl'
    path.write_text(json.dumps({'kind':'header','data':{'follow_config':{},'execution_config':ex}}))
    assert handoff.inspect_header(path,expected)['checks']['closing'] is agrees


@pytest.mark.parametrize('requested,recorded,valid',[
    ('recent',True,True),('recent',False,False),('closing',True,False),('closing',False,True)])
def test_recent_motion_option_must_be_present_in_actual_trace(tmp_path,requested,recorded,valid):
    path=tmp_path/'header.jsonl'
    ex=dict(moving_follow_reserve=True,closing_follow_reserve=True,recent_forward_reserve=recorded)
    path.write_text(json.dumps(dict(kind='header',data=dict(follow_config={},execution_config=ex))))
    assert handoff.inspect_header(path,requested)['checks']['recent'] is valid
