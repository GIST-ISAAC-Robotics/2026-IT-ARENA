import pytest
from candidate import DirectionalPairReference
from arena_autonomy.bypass_initial_reference import CurrentPairReference
from arena_autonomy.bypass_handoff import handoff_baseline
from arena_autonomy.bypass_entry_trial import entry_path, EntryConfig


def walls(left=(-.05, 1.5), right=(-.4, 1.8), shift=0.):
    return {side:dict(slope=0., intercept_m=shift if side=='left' else 0.,
        observed_x_m=list(span)) for side, span in [('left', left), ('right', right)]}


def invoke(monkeypatch, boundary, **extra):
    detail=dict(reason='reference_does_not_cover_entry', walls=boundary,
        all_returns_remain_occupied=True, remembered_line_is_current_observation=False)
    detail.update(extra)
    monkeypatch.setattr(CurrentPairReference, 'resolve', lambda *_:(None, detail))
    return DirectionalPairReference().resolve(None, handoff_baseline(), None)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_only_observed_outer_side_can_generate_entry(monkeypatch, side):
    boundary=walls()
    if side=='left':boundary['left'],boundary['right']=boundary['right'],boundary['left']
    reference, detail=invoke(monkeypatch, boundary)
    assert reference[2]==-.05  # 공통 범위를 뒤로 연장하지 않는다.
    assert detail['entry_covering_sides']==[side]
    assert not detail['unknown_filled'] and detail['all_returns_remain_occupied']
    cfg=handoff_baseline()
    for name, wall in boundary.items():
        fit=(wall['slope'],wall['intercept_m'],*wall['observed_x_m'])
        if name==side:
            path=entry_path(fit,.16 if side=='left' else -.16,cfg,EntryConfig())
            assert path[0,0]==-cfg.wheelbase_m/2
            assert path[-1,0]<=wall['observed_x_m'][1]
        else:
            with pytest.raises(ValueError,match='prefix too short'):
                entry_path(fit,.16,cfg,EntryConfig())


@pytest.mark.parametrize('boundary', [walls(left=(0.,1.5),right=(0.,1.5)),
    walls(left=(1.3,1.5)), walls(left=(2.,3.)), walls(left=(-.05,.24),right=(-.4,.24)),
    walls(shift=.5)])
def test_missing_coverage_overlap_or_agreement_stays_unavailable(monkeypatch,boundary):
    assert invoke(monkeypatch,boundary)[0] is None


@pytest.mark.parametrize('detail', [dict(reason='current_pair_unavailable'),
    dict(reason='seed_transport_unavailable'),dict(all_returns_remain_occupied=False)])
def test_parent_rejections_not_overridden(monkeypatch,detail):
    assert invoke(monkeypatch,walls(),**detail)[0] is None


def test_valid_original_is_identical(monkeypatch):
    result=((0.,0.,-.15,1.5),dict(reason='current_two_wall_straight_reference'))
    monkeypatch.setattr(CurrentPairReference,'resolve',lambda *_:result)
    assert DirectionalPairReference().resolve(None,None,None) == result
