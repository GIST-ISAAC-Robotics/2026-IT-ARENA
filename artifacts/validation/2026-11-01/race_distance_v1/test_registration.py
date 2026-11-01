import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import pytest

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('explicit_five_lap_run',HERE/'run.py')
trial=importlib.util.module_from_spec(spec);spec.loader.exec_module(trial)


def request():
    return SimpleNamespace(config=trial.CONFIG,actor_overlay=trial.OVERLAY,output=trial.OUTPUT,
        mode='g1g2',seed=0,handoff_reserve='recent',record_follow_decisions=True)


def config():
    cfg,over=trial.expected()
    assert cfg==json.loads(trial.CONFIG.read_bytes()) and over==json.loads(trial.OVERLAY.read_bytes())
    return cfg,over


def test_five_laps_use_original_acceptance_except_declared_distance():
    cfg,_=config();baseline=json.loads((HERE.parent/'mixed_course_v1/course.json').read_bytes())
    a=cfg['acceptance']['stop_restart_full_course'];b=baseline['acceptance']['stop_restart_full_course']
    assert {k:v for k,v in a.items() if k not in ('basis','minimum_laps','maximum_actor_laps')} == {
        k:v for k,v in b.items() if k not in ('basis','minimum_laps','maximum_actor_laps')}
    from g1_traffic_eval import actor_travel_m
    assert actor_travel_m(cfg['cases'][0]['schedule'],300)==pytest.approx(238.45)
    assert cfg['cases'][0]['schedule'][:2]==baseline['cases'][0]['schedule'][:2]


def test_adapter_registers_its_code_and_original_five_lap_evaluator():
    cfg,over=config();case=trial.overlay_case(cfg['cases'][0],trial.OVERLAY)
    with patch.object(trial,'parameters',return_value=(cfg,over)):
        files=trial.facility(request(),cfg,case)
    for p in (trial.CHECK_FIVE,HERE/'run.py',HERE/'course.launch.py',trial.CONFIG,trial.OVERLAY):
        assert files[p.relative_to(trial.ROOT).as_posix()]==trial.sha256(p.read_bytes()).hexdigest()


@pytest.mark.parametrize('key,value',[('handoff_reserve','closing'),('record_follow_decisions',False),('seed',1)])
def test_unregistered_controller_or_output_is_rejected(key,value):
    cfg,over=config();r=request();setattr(r,key,value)
    with patch.object(trial,'parameters',return_value=(cfg,over)),pytest.raises(ValueError):
        trial.facility(r,cfg,trial.overlay_case(cfg['cases'][0],trial.OVERLAY))
