from dataclasses import replace
from pathlib import Path
import sys
from unittest.mock import patch
import numpy as np
import pytest
BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
sys.path[:0]=[str(BASE),str(ROOT/'tests')]
from candidate import CorridorHandoff,ForecastGuide
from test_bypass_handoff import context
from arena_autonomy.bypass_handoff import HandoffExecutionConfig
from arena_autonomy.bypass_entry_follow import BypassExecutionConfig
from arena_autonomy.bypass_corridor_trial import CorridorGuide
from arena_autonomy.bypass_entry_trial import straight_reference

def subject():
 return CorridorHandoff(execution=HandoffExecutionConfig(bypass=BypassExecutionConfig(corridor_policy='corridor_v2_signed_hold'),
  preparation_policy='observed_entry_space_v1',persistence_window_policy='trim_before_compare_v1',
  follow_entry_space_enabled=True,reacquire_policy='known_hold_v1'),allow_simulation_trial=True)

def test_same_surface_zero_horizon_and_positive_growth():
 scan,hist,scene=context(.7)
 a=subject().bypass.guide;b=CorridorGuide(a.config,a.registered_entry,stationary_outer_boundary=True,
  wall_reanchor=True,tracked_wall_fit=True,rejoin_enabled=True)
 a.motion_history=b.motion_history=hist
 ref,evidence=straight_reference(scene,a.config)
 assert ref is not None
 for side in ('left','right'):
  p0,_=a._padding(scene,a.config,side,evidence['walls'][side],0.)
  old,_=b._padding(scene,b.config,side,evidence['walls'][side],0.)
  np.testing.assert_array_equal(p0,old)
  ahead,_=a._padding(scene,a.config,side,evidence['walls'][side],.1)
  assert np.all(ahead>=p0-1e-12)
  assert len(a.monitors[side].rows)==len(b.monitors[side].rows)

@pytest.mark.parametrize('growth',[-.1,.251,float('nan'),float('inf')])
def test_bad_horizon_rejected(growth):
 h=subject();scan,hist,scene=context(.7);_,ev=straight_reference(scene,h.config)
 with pytest.raises(ValueError):h.bypass.guide._padding(scene,h.config,'left',ev['walls']['left'],growth)

def test_horizon_decreases_with_reuse_and_uses_dynamic_path():
 h=subject();scan,hist,scene=context(.7)
 h.monitor.rows=[dict(source=.5,time=.6),dict(source=.6,time=.7)]
 plan=dict(chosen='right',rows={'right':dict(offset_m=-.263,transition_m=.9)})
 def cmp(*args,**kw):
  assert kw['entry'].offsets_m==(0.,-.263) and kw['entry'].transitions_m==(.9,)
  return dict(reason='diagnostic',candidates=[])
 with patch.object(h.bypass.guide,'_plan',return_value=plan),patch('candidate._compare_entries',side_effect=cmp):
  _,fresh=h._execution_entries(scene,0.,0.,.02)
  _,reuse=h._execution_entries(replace(scene,t_ref_s=.78),0.,0.,.02)
  assert fresh['padding_growth_s']==pytest.approx(.1)
  assert reuse['padding_growth_s']==pytest.approx(.02)
  entries,stale=h._execution_entries(replace(scene,t_ref_s=.82),0.,0.,.02)
  assert entries==[] and stale['reason']=='observed_scan_horizon_expired'

def test_geometry_discard_keeps_only_surface_observations():
 h=subject();g=h.bypass.guide
 g.monitors['right'].rows=[dict(source=.5)];old=g.monitors
 g.path=np.zeros((2,2));g.wall_confirmations=2;g.reverse_wait=True
 h._discard_path_only()
 assert g.path is None and g.wall_confirmations==0 and g.reverse_wait is False
 assert g.monitors is old and g.monitors['right'].rows==[dict(source=.5)]
 h.reset();assert g.monitors is not old and not g.monitors['right'].rows

@pytest.mark.parametrize('mode',['BYPASS','REACQUIRE','FOLLOW'])
def test_active_reverse_delegates_signed_zero_without_deleting_g1_history(mode):
 h=subject();scan,hist,scene=context(.7);h.mode=mode
 h.normal.command=lambda *a,**kw:(0.,0.,dict(selector_choice='baseline',selector_sides={}))
 h.monitor.update=lambda *a:dict(ready=False,observed=False,fresh=True)
 h.bypass.command=lambda *a,**kw:(0.,.02,dict(reason='entry_reverse_motion_hold',entry_phase='entry'))
 old=h.normal.pass_selector.sides
 with patch.object(h.bypass,'reset',wraps=h.bypass.reset) as reset:
  result=h.command(scene,hist,-.2,'left',.02,.02,raw_scan=scan)
  assert result[0]==0 and h.normal.pass_selector.sides is old
  assert reset.call_count==(1 if mode=='FOLLOW' else 0)
  assert h.mode==('FOLLOW' if mode=='FOLLOW' else 'BYPASS')

@pytest.mark.parametrize('fault',['nan','epoch','no_scan'])
def test_real_fault_still_clears_active_geometry(fault):
 h=subject();scan,hist,scene=context(.7);h.mode='BYPASS'
 h.bypass.guide.path=np.zeros((2,2))
 speed=-.2
 if fault=='nan':speed=float('nan')
 if fault=='epoch':h.last_key=('laser_frame',999,.6);h.last_ref=.6
 if fault=='no_scan':scan=None
 r=h.command(scene,hist,speed,'left',0.,.02,raw_scan=scan)
 assert r[0]==0 and h.mode=='FOLLOW' and h.bypass.guide.path is None


@pytest.mark.parametrize('eligible,speed,offset,expected',[
 (True,.35,-.26,True),(False,.35,-.26,False),(True,.20,-.26,False),
 (True,float('nan'),-.26,False),(True,.35,0.,False),(True,True,-.26,False)])
def test_local_admission_requires_actual_capped_command_support(eligible,speed,offset,expected):
 from candidate import local_admission
 row=dict(eligible=eligible,target_speed_mps=speed,offset_m=offset,preview_point_gap_m=-.06)
 assert local_admission(row,.35)==expected


@pytest.mark.parametrize('primary_reason,path_present,attempt',[
 ('entry_reverse_recovery_wait',True,False),('entry_guide_input_reset',True,False),
 ('no_entry_command_candidate',False,False),('no_entry_command_candidate',True,True)])
def test_slow_trial_only_existing_path_and_valid_primary(primary_reason,path_present,attempt):
 from arena_autonomy.bypass_signed_hold_trial import SignedHoldGuide
 h=subject();g=h.bypass.guide;scan,hist,scene=context(.7)
 if path_present:g.path=np.array([[0.,0.],[1.,0.]])
 calls=[]
 def evaluate(self,scene,history,speed,previous,dt):
  calls.append((self.entry.speed_cap_mps,speed,previous,scene.source_first_s))
  if len(calls)==1:return dict(reason=primary_reason,selected=None)
  return dict(reason='entry_command_candidates_available',selected=dict(speed_mps=self.entry.speed_cap_mps))
 with patch.object(SignedHoldGuide,'evaluate',evaluate):r=g.evaluate(scene,hist,.36,.03,.02)
 assert len(calls)==(2 if attempt else 1)
 assert all(c[1:]==calls[0][1:] for c in calls)
 assert g.entry.speed_cap_mps==g.registered_entry.speed_cap_mps==.35
 if attempt:assert r['selected']['speed_mps']==.25 and r['slow_command_trial']['applied']

def test_all_slow_candidates_blocked_keeps_zero_and_restores_cap():
 from arena_autonomy.bypass_signed_hold_trial import SignedHoldGuide
 h=subject();g=h.bypass.guide;g.path=np.array([[0.,0.],[1.,0.]])
 scan,hist,scene=context(.7);caps=[]
 def evaluate(self,*args):
  caps.append(self.entry.speed_cap_mps)
  return dict(reason='no_entry_command_candidate',selected=None)
 with patch.object(SignedHoldGuide,'evaluate',evaluate):r=g.evaluate(scene,hist,.1,0.,.02)
 assert caps==[.35,.25,.15,.08] and r['selected'] is None
 assert not r['slow_command_trial']['applied'] and g.entry.speed_cap_mps==.35
