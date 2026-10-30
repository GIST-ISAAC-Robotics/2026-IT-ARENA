from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from g2_handoff_eval_core_v3 import status_ok

def row(automatic=False):
 return dict(behavior='HOLD',mode='g1_g2_handoff_trial' if automatic else 'g2_entry_trial',
  automatic_race_mode_selection=automatic,simulation_trial_only=True,
  handoff_state='FOLLOW',output_requires_independent_guard=True)

def test_modes_are_explicit_and_not_interchangeable():
 assert status_ok([row()])
 assert status_ok([row(True)],'g1_g2_handoff_trial')
 assert not status_ok([row(True)])
 assert not status_ok([row()],'g1_g2_handoff_trial')
 assert not status_ok([row(),row(True)],'g1_g2_handoff_trial')

def test_missing_rows_and_unknown_mode_fail():
 assert not status_ok([],'g1_g2_handoff_trial')
 assert not status_ok([{}],'g1_g2_handoff_trial')
 assert not status_ok([row(True)],'any_controller')

def test_bad_mode_flags_or_guard_boundary_fail():
 for key,value in [('simulation_trial_only',False),('automatic_race_mode_selection',False),
                   ('handoff_state','UNKNOWN'),('output_requires_independent_guard',False)]:
  r=row(True);r[key]=value
  assert not status_ok([r],'g1_g2_handoff_trial')

def test_all_valid_handoff_states_including_zero_command():
 rows=[]
 for state in ('FOLLOW','BYPASS','REACQUIRE','FOLLOW'):
  r=row(True);r['handoff_state']=state;rows.append(r)
 assert status_ok(rows,'g1_g2_handoff_trial')
