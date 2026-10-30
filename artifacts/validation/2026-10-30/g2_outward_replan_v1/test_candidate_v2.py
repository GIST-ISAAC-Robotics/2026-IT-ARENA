"""기존 검사 거절을 승격하지 않고 새 관측의 같은 방향 변경만 남긴다."""
from copy import deepcopy
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import pytest

HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
from candidate_v2 import OutwardReplanGuide
from arena_autonomy.bypass_entry_trial import EntryGuide,EntryConfig


def subject(sign=1):
    h=OutwardReplanGuide(entry=EntryConfig(offsets_m=(0.,-.16,.16,-.20,.20,-.24,.24,-.26,.26)),
        stationary_outer_boundary=True,rejoin_enabled=True,wall_reanchor=True,tracked_wall_fit=True)
    h.path=np.array([[-.1,0.],[1.3,.1]])
    h.offset=sign*.24;h.transition=1.2;h.side='left' if sign>0 else 'right'
    h.last_source=.4;h.reference_s=.5;h.wall_confirmations=2;h.intent.sign=sign
    h.rejoin_confirmations=h.center_confirmations=1
    scene=SimpleNamespace(source_first_s=.5,t_ref_s=.6)
    original=dict(reason='no_entry_command_candidate',guide_mode='measured_motion_transport_current_outer_wall',
        selected=None,candidates=[dict(command_clearance_m=.04)],reference=dict(walls={
            h.side:dict(slope=0.,intercept_m=-sign*.2,observed_x_m=[-.2,1.5])}))
    return h,scene,original


def accepted(*args,**kwargs):
    g=kwargs['guide'];offset=g['offset_m'];transition=g['transition_m']
    return dict(reason='entry_command_candidates_available',selected=dict(offset_m=offset,transition_m=transition,
        speed_mps=.35,steering_rad=.06),candidates=[dict(offset_m=offset,transition_m=transition,
        eligible=True,transition_reaches_offset=True,preview_point_gap_m=.01,command_clearance_m=.20,
        preview_return_progress_m=1.2,path_points=np.asarray(g['path']).tolist())])


@pytest.mark.parametrize('sign',[-1,1])
def test_one_small_same_side_commit_without_extra_observations(sign):
    h,sc,old=subject(sign)
    with patch.object(EntryGuide,'evaluate',return_value=old),patch('candidate_v2._compare_entries',side_effect=accepted) as compare:
        result=h.evaluate(sc,None,0.,0.,.02)
        assert len(compare.call_args_list)==3
        assert all(c.kwargs['stationary_boundaries'] is True for c in compare.call_args_list)
        assert all(c.kwargs['guide']['offset_m']==sign*.26 for c in compare.call_args_list)
    assert result['outward_replan']['applied'] and h.offset==sign*.26
    assert h.outward_replanned and h.wall_confirmations==2 and h.intent.sign==sign
    assert h.rejoin_confirmations==h.center_confirmations==0
    sc.source_first_s=.6;sc.t_ref_s=.7
    with patch.object(EntryGuide,'evaluate',return_value=old),patch('candidate_v2._compare_entries',side_effect=AssertionError('second replan')):
        assert h.evaluate(sc,None,0.,0.,.02) is old


@pytest.mark.parametrize('gate',['duplicate','rejoin','missing_path','unconfirmed_wall','wrong_mode','at_cap','too_large_step','no_stationary_assumption'])
def test_ineligible_context_does_not_request_new_permission(gate):
    h,sc,old=subject()
    if gate=='no_stationary_assumption':h.stationary_outer_boundary=False
    elif gate=='duplicate':sc.source_first_s=.4
    elif gate=='rejoin':h.phase='rejoin'
    elif gate=='missing_path':h.path=None
    elif gate=='unconfirmed_wall':h.wall_confirmations=1
    elif gate=='wrong_mode':old['guide_mode']='entry_guide_input_reset'
    elif gate=='at_cap':old['selected']=dict(speed_mps=.35)
    elif gate=='too_large_step':h.offset=.20
    before=h.offset
    with patch.object(EntryGuide,'evaluate',return_value=old),patch('candidate_v2._compare_entries',side_effect=AssertionError('unexpected comparison')):
        result=h.evaluate(sc,None,0.,0.,.02)
    assert not h.outward_replanned and h.offset==before


@pytest.mark.parametrize('failure',['no_selection','blocked_region','point_overlap','short_transition','no_speed_gain','no_clearance_gain'])
def test_current_rejection_cannot_be_overridden(failure):
    h,sc,old=subject();before=h.path.copy()
    def rejected(*args,**kwargs):
        r=accepted(*args,**kwargs);row=r['candidates'][0]
        if failure=='no_selection':r['selected']=None
        elif failure=='blocked_region':row['eligible']=False
        elif failure=='point_overlap':row['preview_point_gap_m']=-.001
        elif failure=='short_transition':row['transition_reaches_offset']=False
        elif failure=='no_speed_gain':r['selected']['speed_mps']=0.
        elif failure=='no_clearance_gain':row['command_clearance_m']=.06
        return r
    with patch.object(EntryGuide,'evaluate',return_value=old),patch('candidate_v2._compare_entries',side_effect=rejected):
        result=h.evaluate(sc,None,0.,0.,.02)
    assert result['selected'] is None and not result['outward_replan']['applied']
    assert not h.outward_replanned and np.array_equal(h.path,before)


def test_reset_forgets_trial_path_and_single_use_state():
    h,sc,old=subject();h.outward_replanned=True;h.reset()
    assert h.path is None and not h.outward_replanned and h.wall_confirmations==0
