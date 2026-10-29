"""g2_eval_core 음성 반례. 정상 합성 주행 1개를 만들고 한 조건씩 깨뜨려 해당 판정만 실패하는지 본다."""
import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parent))
import g2_eval_core as core

EXEC={'stationary_outer_boundary':True,'wall_reanchor':True,'tracked_wall_fit':True,'rejoin_enabled':True,
      'wide_entry':True,'prefer_earlier_transition':False}
RULES=dict(returned_to_center_lateral_m=.05,returned_to_center_heading_rad=.10,returned_to_center_hold_s=1.0,
           actor_contact_messages_with_ego=0,overlap_samples=0,max_off_road_wheels=2,command_cap_mps=.35,
           max_sample_interval_s=.05,
           trace_header_expected=dict(schema=4,controller_mode='g2_entry_trial',execution_config=EXEC,
                                      follow_config={'max_speed_mps':1.0,'boundary_evidence_mode':'connected'},
                                      motion_config={'max_input_gap_s':.05}))
GREEN,END,STOP=0.,12.,13.
TARGET_FRONT=1.3


def drive(target_lateral=-.149,dt=.01,return_at=6.0):
    """초록 뒤 0.35 m/s로 진행, 2–5 s 사이 표적 반대쪽으로 0.16 m 비켰다가 return_at 이후 중앙."""
    side=1. if target_lateral<0 else -1.
    rows=[];t=GREEN
    while t<=STOP+1e-9:
        s=.35*min(t,END)
        if 1.<t<return_at:lateral=side*.16
        else:lateral=0.
        rows.append(dict(t=round(t,6),s=s,lateral=lateral,heading=0.,rear_s=s-.1,front_s=s+.1))
        t+=dt
    return rows


def surface(max_off=2,events=0,wall=0,gaps=0):
    return dict(interpretations={k:dict(max_off_wheels=max_off,penalty_events_merged=events) for k in ('any','center','full')},
                wall_wheel_samples=wall,samples=100,sample_gaps_over_limit=gaps)


def base(**over):
    rows=over.pop('rows',None) or drive()
    kw=dict(rows=rows,green=GREEN,window_end=END,stop_end=STOP,target_front_s=TARGET_FRONT,target_lateral_m=-.149,rules=RULES,
            ego_times=[r['t'] for r in rows],contacts_in_run=0,overlap_samples=0,surface=surface(),
            trace_rows=[dict(drive_speed_mps=.35,safe_speed_mps=.35)],decision_speeds=[.35,0.],
            header=dict(schema=4,controller_mode='g2_entry_trial',execution_config=dict(EXEC),
                        follow_config={'max_speed_mps':1.0,'boundary_evidence_mode':'connected','extra':1},
                        motion_config={'max_input_gap_s':.05}),
            replay=dict(passed=True,calls=640,matched=640,mismatches=0),
            status_rows=[dict(behavior='bypass',mode='g2_entry_trial',simulation_trial_only=True,automatic_race_mode_selection=False)],
            harness_valid=True,target_static=dict(valid=True))
    kw.update(over)
    return core.gates(**kw)


class G2CoreTest(unittest.TestCase):
    def failed(self,gates):return sorted(k for k,v in gates.items() if not v)

    def test_nominal_passes_all(self):
        g,d=base()
        self.assertEqual(self.failed(g),[])
        self.assertTrue(d['center']['returned'])
        self.assertLessEqual(d['center']['hold_satisfied_s'],END)

    def test_return_only_after_window_fails(self):
        # 주행 창이 끝날 때까지 비켜 있다가 STOP 창에서만 중앙: 기존 g2_eval(stop_end까지)는 통과시켰을 경우.
        rows=drive(return_at=END+.05)
        g,d=base(rows=rows)
        self.assertIn('returned_to_center_in_window',self.failed(g))
        self.assertFalse(d['center']['returned'])

    def test_short_hold_inside_window_fails(self):
        rows=drive(return_at=END-.5)
        g,_=base(rows=rows)
        self.assertIn('returned_to_center_in_window',self.failed(g))

    def test_data_gap_breaks_hold_and_coverage(self):
        holes=((6.7,7.),(7.6,7.9),(8.5,8.8),(9.4,9.7),(10.3,10.6),(11.2,11.5))   # 중앙 구간을 1 s 미만 조각으로 자름
        rows=[r for r in drive() if not any(a<r['t']<b for a,b in holes)]
        g,d=base(rows=rows)
        self.assertIn('ego_window_coverage',self.failed(g))
        self.assertIn('returned_to_center_in_window',self.failed(g))   # 각 연속 구간이 1 s 미만
        self.assertLess(d['center']['longest_s'],1.0)

    def test_target_not_passed_fails(self):
        g,_=base(target_front_s=10.)   # 창 안 최대 진행 4.2 m 미만의 표적 앞끝이 아니면 통과 불가
        g2,_=base(target_front_s=5.)
        self.assertIn('target_passed',self.failed(g2))
        self.assertIn('returned_to_center_in_window',self.failed(g2))
        self.assertIn('target_passed',self.failed(g))

    def test_wrong_side_fails(self):
        g,_=base(target_lateral_m=.149)   # 왼쪽 표적인데 자차가 왼쪽(+)으로 지남
        self.assertEqual(self.failed(g),['bypass_side_matches'])

    def test_left_target_right_bypass_passes(self):
        rows=drive(target_lateral=.149)
        g,d=base(rows=rows,target_lateral_m=.149)
        self.assertEqual(self.failed(g),[])
        self.assertEqual(d['bypass_side']['observed'],'right')

    def test_contact_and_overlap_fail(self):
        self.assertEqual(self.failed(base(contacts_in_run=1)[0]),['no_actor_contact'])
        self.assertEqual(self.failed(base(overlap_samples=3)[0]),['no_body_overlap'])

    def test_three_wheel_or_wall_fail(self):
        self.assertEqual(self.failed(base(surface=surface(max_off=3,events=1))[0]),['wheels_two_or_fewer_off_road'])
        self.assertEqual(self.failed(base(surface=surface(events=1))[0]),['wheels_two_or_fewer_off_road'])
        self.assertEqual(self.failed(base(surface=surface(wall=1))[0]),['no_wall_wheel'])
        self.assertEqual(self.failed(base(surface=surface(gaps=2))[0]),['surface_samples_continuous'])
        missing=surface();missing['interpretations'].pop('full')
        self.assertEqual(self.failed(base(surface=missing)[0]),['wheels_two_or_fewer_off_road'])

    def test_safe_over_cap_fails_even_if_drive_ok(self):
        g,d=base(trace_rows=[dict(drive_speed_mps=.35,safe_speed_mps=.36)])
        self.assertEqual(self.failed(g),['command_cap_drive_safe_decision'])
        self.assertEqual(self.failed(base(decision_speeds=[.4])[0]),['command_cap_drive_safe_decision'])
        self.assertEqual(self.failed(base(trace_rows=[dict(drive_speed_mps=.35)])[0]),['command_cap_drive_safe_decision'])

    def test_wrong_g2_schema_or_config_fails(self):
        for change in (dict(schema=3),dict(controller_mode='local_pursuit'),
                       dict(execution_config={**EXEC,'rejoin_enabled':False}),
                       dict(follow_config={'max_speed_mps':.5,'boundary_evidence_mode':'connected'}),
                       dict(motion_config={'max_input_gap_s':.1})):
            header=dict(schema=4,controller_mode='g2_entry_trial',execution_config=dict(EXEC),
                        follow_config={'max_speed_mps':1.0,'boundary_evidence_mode':'connected'},motion_config={'max_input_gap_s':.05})
            header.update(change)
            self.assertEqual(self.failed(base(header=header)[0]),['trace_header_matches_config'],change)

    def test_replay_status_harness_target_fail(self):
        self.assertEqual(self.failed(base(replay=dict(passed=True,calls=640,matched=639,mismatches=1))[0]),['decision_replay_identical'])
        self.assertEqual(self.failed(base(replay=dict(passed=True,calls=0,matched=0,mismatches=0))[0]),['decision_replay_identical'])
        self.assertEqual(self.failed(base(status_rows=[])[0]),['g2_controller_rows'])
        self.assertEqual(self.failed(base(status_rows=[dict(behavior='x',mode='g1',simulation_trial_only=True,
                                                            automatic_race_mode_selection=False)])[0]),['g2_controller_rows'])
        self.assertEqual(self.failed(base(harness_valid=False)[0]),['harness_valid'])
        self.assertEqual(self.failed(base(target_static=dict(valid=False))[0]),['static_target_pose_valid'])

    def test_stop_window_separate(self):
        rows=drive()
        for r in rows:
            if r['t']>END+.2:r['lateral']=.08
        g,d=base(rows=rows)
        self.assertEqual(self.failed(g),['stop_window_within_center_tolerance'])
        self.assertTrue(d['center']['returned'])

    def test_static_target_helper(self):
        actor=[(t/10,1.,2.,0.) for t in range(50)]
        self.assertTrue(core.static_target(actor,(1.,2.),0.,5.,.005,[])['valid'])
        moved=actor+[(5.,1.01,2.,0.)]
        self.assertFalse(core.static_target(moved,(1.,2.),0.,5.,.005,[])['valid'])
        self.assertFalse(core.static_target(actor,(1.,2.),0.,5.,.005,[{'t':1}])['valid'])

    def test_coverage_edges(self):
        self.assertFalse(core.coverage([.1,.12],0.,.2,.05)['valid'])     # 시작 경계 공백 0.1 s
        self.assertTrue(core.coverage([0.,.04,.08,.12,.16,.2],0.,.2,.05)['valid'])
        self.assertFalse(core.coverage([],0.,1.,.05)['valid'])


if __name__=='__main__':unittest.main()
