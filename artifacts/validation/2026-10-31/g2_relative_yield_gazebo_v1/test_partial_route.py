from copy import deepcopy
import json
from pathlib import Path
import pytest
import partial_route


def registered():
    config=json.loads(Path(__file__).with_name('traffic_config.json').read_text(encoding='utf-8'))
    return config,config['cases'][0]


def test_exact_prefix_restores_whole_reference_for_geometry_only():
    config,case=registered();before=deepcopy(case)
    extended=partial_route.prefix_case(config,case)
    assert extended['duration_s']==155.
    assert extended['schedule'][-1]['to_s']==155.
    assert case==before and case['duration_s']==45.


@pytest.mark.parametrize('change',['duration','speed','stop_time','shape','lap','side'])
def test_changed_schedule_or_geometry_is_rejected(change):
    config,case=registered()
    if change=='duration':case['duration_s']=50.
    elif change=='speed':case['schedule'][2]['speed_mps']=.4
    elif change=='stop_time':case['schedule'][1]['to_s']=10.
    elif change=='shape':config['actor']['width_m']=.10
    elif change=='lap':config['acceptance'][case['name']]['minimum_laps']=0
    elif change=='side':case['actor_lateral_offset_m']=.1
    with pytest.raises(ValueError):partial_route.prefix_case(config,case)


def test_original_geometry_failure_is_not_swallowed(monkeypatch):
    config,case=registered();calls=[]
    def fail(c,l,k,s,r,o):
        calls.append(k['duration_s'])
        raise ValueError('wall overlap')
    monkeypatch.setattr(partial_route,'check_full_course',fail)
    with pytest.raises(ValueError,match='wall overlap'):
        partial_route.check_registered_prefix(config,None,case,0.,None,[])
    assert calls==[155.]
