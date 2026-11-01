from dataclasses import replace
import json

import pytest

from race_traffic_course import ROLLING, entry_for_output, inputs
from prelap_signal_eval import ENTRY as SIGNAL_ENTRY
from test_race_traffic_course import Request


def registered():
    config=json.loads((ROLLING/'course.json').read_bytes())
    request=replace(Request(),config=ROLLING/'course.json', output=ROLLING/'run1',actor_overlay=None,case='first_curve')
    return request,config


def test_rolling_lead_preserves_first_curve_and_uses_same_physical_signal():
    req,cfg=registered()
    assert entry_for_output(req.output)==SIGNAL_ENTRY
    files=inputs(req,cfg,cfg['cases'][0])
    original=json.loads((ROLLING.parents[3]/'config/tests/g1_traffic_extensions_v1.json').read_bytes())
    assert cfg['acceptance']==original['acceptance']
    assert cfg['cases'][0]['schedule']!=next(c['schedule'] for c in original['cases'] if c['name']=='first_curve')
    assert 'scripts/race_traffic_course.py' in files
    with pytest.raises(ValueError):entry_for_output(ROLLING.parents[0]/'unregistered'/'run1')


def test_substituting_a_stopped_lead_or_easier_gap_is_rejected():
    req,cfg=registered()
    cfg['cases'][0]['schedule'][1]['speed_mps']=0.
    with pytest.raises(ValueError):inputs(req,cfg,cfg['cases'][0])
    req,cfg=registered()
    cfg['acceptance']['first_curve']['maximum_final_gap_m']=3.
    with pytest.raises(ValueError):inputs(req,cfg,cfg['cases'][0])
