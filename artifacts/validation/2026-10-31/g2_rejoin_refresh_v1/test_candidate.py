from dataclasses import replace
import numpy as np
import pytest
from candidate import refresh_rejoin_path,original,modified,insertion,needle,tail
from arena_autonomy.bypass_entry_trial import EntryConfig,entry_path
from arena_autonomy.bypass_handoff import handoff_baseline


def data():
    config=handoff_baseline();entry=EntryConfig();fit=(0.,.26,-.2,1.4)
    # 직선 경로가 현재 차체보다 옆으로 크게 벗어났고, 추종 조향이 포화되는 상태.
    path=np.c_[np.linspace(-.15,1.4,80),np.full(80,.26)]
    return path,fit,config,entry


def test_only_proposal_hook_changed_existing_permission_checks_remain():
    restored=modified.replace(insertion,'').replace("'center_confirmations':self.center_confirmations,'rejoin_refresh':rejoin_refresh}",tail)
    assert restored==original


def test_observed_return_reduces_saturation_without_extending_wall():
    path,fit,c,e=data();out,detail=refresh_rejoin_path(path,fit,c,e,'rejoin',True,2,.35,.1)
    assert detail['applied'] and not detail['proposal_is_permission']
    assert abs(detail['new_pursuit_steering_rad'])<abs(detail['old_pursuit_steering_rad'])
    assert out[:,0].min()>=fit[2] and out[:,0].max()<=fit[3]
    assert out[0,1]==0 and out[-1,1]==pytest.approx(fit[1])
    np.testing.assert_array_equal(path[:,1],np.full(len(path),.26))


@pytest.mark.parametrize('phase,fresh,confirmations',[('entry',True,2),('centered',True,2),('rejoin',False,2),('rejoin',True,1)])
def test_other_phase_reused_scan_and_unconfirmed_wall_unchanged(phase,fresh,confirmations):
    path,fit,c,e=data();out,d=refresh_rejoin_path(path,fit,c,e,phase,fresh,confirmations,.35,.1)
    assert out is path and not d['applied']


def test_non_saturated_valid_return_unchanged():
    _,fit,c,e=data();path=entry_path(fit,0.,c,e)
    out,d=refresh_rejoin_path(path,fit,c,e,'rejoin',True,2,.35,.1)
    assert out is path and not d['applied']


@pytest.mark.parametrize('fit',[(0.,.26,-.2,.7),(0.,.26,.01,1.4),(float('nan'),.26,-.2,1.4)])
def test_short_missing_or_invalid_current_wall_not_extrapolated(fit):
    path,_,c,e=data();out,d=refresh_rejoin_path(path,fit,c,e,'rejoin',True,2,.35,.1)
    assert out is path and not d['applied']


@pytest.mark.parametrize('speed,age',[(-.1,.1),(float('nan'),.1),(1.1,.1),(.35,-.1)])
def test_invalid_or_reverse_motion_not_replanned(speed,age):
    path,fit,c,e=data();out,d=refresh_rejoin_path(path,fit,c,e,'rejoin',True,2,speed,age)
    assert out is path and not d['applied']
