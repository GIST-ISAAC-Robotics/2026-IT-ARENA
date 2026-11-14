"""복수 상대 시험만 명시 개방하며 단일 상대 및 기본 코스의 계약은 보존한다."""
from copy import deepcopy
from types import SimpleNamespace

import pytest

import official_traffic_course as course

NEW = 'official_v2026_11_06'
ARGS = dict(mode='g1g2', handoff_reserve='paced', handoff_yield_progress=True,
            handoff_rear_sweep=True, record_follow_decisions=True, seed=0,
            actor_overlay='registered/actors.json')
CONFIG = dict(track=NEW, launch=dict(speed_profile='g1_open_1p5', grid_slot=3),
              official_multi_actor_trial=True)
CASE = dict(additional_actors=[dict(model_name='eval_side')], additional_actor_scope='registered case')


@pytest.mark.parametrize('count', [1, 2, 3])
def test_explicit_multi_actor_selection(count):
    case = {**CASE, 'additional_actors':[dict(model_name=f'eval_{i}') for i in range(count)]}
    result = course.inputs(SimpleNamespace(**ARGS), CONFIG, case, track=NEW)
    assert 'scripts/official_traffic_course.py' in result


@pytest.mark.parametrize('which', ['no_optin', 'not_bool', 'no_overlay', 'empty', 'too_many', 'no_scope', 'old_track'])
def test_incomplete_multi_actor_contract_is_rejected(which):
    args, cfg, case, track = deepcopy(ARGS), deepcopy(CONFIG), deepcopy(CASE), NEW
    if which=='no_optin': cfg.pop('official_multi_actor_trial')
    if which=='not_bool': cfg['official_multi_actor_trial']=1
    if which=='no_overlay': args['actor_overlay']=None
    if which=='empty': case['additional_actors']=[]
    if which=='too_many': case['additional_actors']*=4
    if which=='no_scope': case.pop('additional_actor_scope')
    if which=='old_track': track=cfg['track']=course.DEFAULT_TRACK
    with pytest.raises(ValueError, match='복수 상대'):
        course.inputs(SimpleNamespace(**args), cfg, case, track=track)


def test_new_track_single_actor_unchanged_without_multi_selection():
    cfg=deepcopy(CONFIG);cfg.pop('official_multi_actor_trial')
    args=deepcopy(ARGS);args.pop('actor_overlay')
    result=course.inputs(SimpleNamespace(**args),cfg,{},track=NEW)
    assert result
