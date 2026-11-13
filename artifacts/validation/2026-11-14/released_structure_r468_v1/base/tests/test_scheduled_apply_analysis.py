import copy
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from analyze_scheduled_apply import compare_envelopes,scheduled_readiness,model_without_schedule,terminal_order_valid


def test_all_envelopes_not_only_transitions():
    event={'type':'applied','seq':2,'stamp_us':1000,'apply_us':21000,'sim_us':21000,'speed':1.,'steering':.1,'epoch':'a'}
    second={**event,'stamp_us':6000,'apply_us':26000,'sim_us':26000}
    a=[event,second];b=copy.deepcopy(a)
    for e in b:e['epoch']='different_uuid'
    assert compare_envelopes(a,b,0,30000)['same_all_normal_applied_envelopes']
    b[1]['stamp_us']+=1000
    assert not compare_envelopes(a,b,0,30000)['same_all_normal_applied_envelopes']
    assert not compare_envelopes(a,b[:1],0,30000)['same_all_normal_applied_envelopes']
    assert not compare_envelopes([],[],0,30000)['same_all_normal_applied_envelopes']


def test_readiness_missing_topics_fail():
    assert not scheduled_readiness({})
    assert not scheduled_readiness({'readiness':{}})


def test_stop_is_terminal_until_new_arm():
    events=[{'type':t} for t in ('armed','applied','stopped')]
    assert terminal_order_valid(events)
    assert not terminal_order_valid(events+[{'type':'applied'}])
    assert not terminal_order_valid([{'type':'applied'},{'type':'stopped'}])
    assert not terminal_order_valid(events[:-1])


def test_model_removes_only_evaluation_tags(tmp_path):
    p=tmp_path/'model.sdf'
    p.write_text('<sdf><model><plugin><wheel>65</wheel><evaluation_scheduled_apply>true</evaluation_scheduled_apply>'
                 '<evaluation_wall_timeout>30</evaluation_wall_timeout></plugin></model></sdf>')
    cleaned=model_without_schedule(p)
    assert b'wheel' in cleaned and b'evaluation_scheduled_apply' not in cleaned
    p.write_text('<sdf><model><plugin><wheel>64</wheel></plugin></model></sdf>')
    assert model_without_schedule(p)!=cleaned
