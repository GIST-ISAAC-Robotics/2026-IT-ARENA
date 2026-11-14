"""실제 잘린 코너 경로와 초기/잘못된 접속 반례. 자유 공간 증명은 별도다."""
from dataclasses import replace
import json
from pathlib import Path
import numpy as np
import pytest
from arena_autonomy.local_follow import FollowConfig,retained_forward_prefix,anchor_error

ROOT=Path(__file__).resolve().parents[1]
FIX=json.loads((ROOT/'artifacts/validation/2026-11-11/forward_prefix_r74_v1/fixture.json').read_text(encoding='utf-8'))


def case():
    return np.asarray(FIX['candidate']),np.asarray(FIX['reference']),dict(FIX['info']),FollowConfig(piecewise_wall_trial=True)


@pytest.mark.parametrize('sign',[1.,-1.])
def test_forward_cut_is_not_lateral_displacement(sign):
    path,reference,info,cfg=case();path[:,1]*=sign;reference[:,1]*=sign
    assert anchor_error(info,cfg)>.14
    before=path.copy()
    result=retained_forward_prefix(path,reference,info,cfg)
    assert result is not None and result['lateral_error_m']<.04
    assert result['unknown_filled'] is False and result['observed_prefix_extended'] is False
    np.testing.assert_array_equal(path,before)


@pytest.mark.parametrize('fault',['no_history','short_wall','other_model','off','far_start','lateral','backward','different_path','nan'])
def test_unconfirmed_or_displaced_candidate_is_rejected(fault):
    path,reference,info,cfg=case()
    if fault=='no_history':reference=None
    elif fault=='short_wall':info['boundary_observed_ahead_m']=.3
    elif fault=='other_model':info['wall_model']='observed_fragment_corner'
    elif fault=='off':cfg=replace(cfg,piecewise_wall_trial=False)
    elif fault=='far_start':path[:,0]+=.3
    elif fault=='lateral':path[:,1]+=.2
    elif fault=='backward':path[:,0]*=-1
    elif fault=='different_path':reference[:,1]+=.2
    elif fault=='nan':path[0,0]=float('nan')
    assert retained_forward_prefix(path,reference,info,cfg) is None
