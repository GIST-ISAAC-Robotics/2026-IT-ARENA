import json
from dataclasses import dataclass,replace
from pathlib import Path
import pytest
from race_traffic_course import BASE, CONFIG, OVERLAY, inputs, signal
from traffic_actors import overlay_case
from test_prelap_signal_eval import sequence


@dataclass
class Request:
    config:Path=CONFIG
    actor_overlay:Path=OVERLAY
    mode:str='g1g2'
    seed:int=0
    record_follow_decisions:bool=True
    output:Path=BASE/'example'
    case:str='stop_restart_full_course'


def test_course_cannot_substitute_one_car_or_relaxed_contact_thresholds():
    cfg=json.loads(CONFIG.read_text(encoding='utf-8'))
    case=overlay_case(cfg['cases'][0],OVERLAY)
    assert inputs(Request(),cfg,case)
    with pytest.raises(ValueError):inputs(replace(Request(),actor_overlay=None),cfg,case)
    altered={**cfg,'acceptance':{**cfg['acceptance'],'minimum_body_gap_m':0}}
    with pytest.raises(ValueError):inputs(Request(),altered,case)


def test_traffic_timestamp_is_compared_to_the_actual_green_receipt(tmp_path):
    rows,p=sequence(tmp_path)
    p.write_text('\n'.join(map(json.dumps,rows)))
    trace=[dict(t=1.2,autonomy={'started':False}),dict(t=7.1,autonomy={'started':True})]
    labels={'green_probe','red','green'}
    assert signal(tmp_path,labels,trace)['passed']
    trace[0]['autonomy']['started']=True
    assert not signal(tmp_path,labels,trace)['passed']
