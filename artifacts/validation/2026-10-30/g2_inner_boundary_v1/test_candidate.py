import math
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest import TestCase, main

import numpy as np

ROOT=Path(__file__).resolve().parents[4]
sys.path[:0]=[str(Path(__file__).resolve().parent),str(ROOT/'src/arena_autonomy')]
from candidate import (BoundaryMonitor,CorridorPolicy,PlannedIntent,abeam_age,boundary_face,choose_side,
    connected_wall_returns,straight_blocked,wall_frame)
from arena_autonomy.local_follow import FollowConfig

P=CorridorPolicy()


class S:
    def __init__(self,t,epoch=0):
        self.source_first_s=t;self.source_epoch=epoch;self.t_ref_s=t+.1


def face(n,along=(1.,1.2)):
    return dict(normal_m=float(n),method='test',along_range_m=along,endpoints_xy_m=None)


def same_interval(row):
    return (1.,1.2)


class Monitor(TestCase):
    def feed(self,values,dt=.1):
        m=BoundaryMonitor(P);out=None
        for i,v in enumerate(values):
            out=m.update(S(i*dt),face(v),same_interval)
        return m,out

    def test_needs_full_window(self):
        self.assertFalse(self.feed([.35]*4)[1]['stable'])
        out=self.feed([.35]*5)[1]
        self.assertTrue(out['stable']);self.assertAlmostEqual(out['observed_rate_term_mps'],.075)
        self.assertFalse(out['rate_term_is_physical_bound'])

    def test_surface_error_budget_widens_rate_term(self):
        policy=CorridorPolicy(surface_error_budget_m=.06);m=BoundaryMonitor(policy)
        for i in range(5):
            out=m.update(S(i*.1),face(.35),same_interval)
        self.assertAlmostEqual(out['observed_rate_term_mps'],.225)

    def test_approach_cancels_evidence(self):
        out=self.feed([.35,.35,.35,.35,.31])[1]
        self.assertFalse(out['stable']);self.assertAlmostEqual(out['approach_m'],.04)

    def test_retreat_is_not_approach(self):
        self.assertTrue(self.feed([.31,.32,.33,.34,.35])[1]['stable'])

    def test_same_scan_is_not_new_evidence(self):
        m=BoundaryMonitor(P)
        for _ in range(9):
            out=m.update(S(1.),face(.35),same_interval)
        self.assertFalse(out['stable']);self.assertEqual(len(m.rows),1)

    def test_gap_or_epoch_or_loss_resets(self):
        m=self.feed([.35]*5)[0]
        self.assertFalse(m.update(S(.7),face(.35),same_interval)['stable'])
        self.assertEqual(len(m.rows),1)
        m=self.feed([.35]*5)[0]
        self.assertFalse(m.update(S(.5,1),face(.35),same_interval)['stable'])
        m=self.feed([.35]*5)[0]
        self.assertFalse(m.update(S(.5),None)['stable'])
        self.assertEqual(m.rows,[])

    def test_different_face_does_not_inherit_evidence(self):
        m=self.feed([.35]*5)[0]
        out=m.update(S(.5),face(.35,(2.,2.2)),same_interval)
        self.assertFalse(out['stable']);self.assertEqual(len(m.rows),1)

    def test_untransportable_history_does_not_inherit_evidence(self):
        m=self.feed([.35]*5)[0]
        out=m.update(S(.5),face(.35),lambda row:None)
        self.assertFalse(out['stable']);self.assertEqual(len(m.rows),1)


class Face(TestCase):
    def rows(self,along,normal):
        along=np.asarray(along,float);normal=np.asarray(normal,float)
        return np.arange(len(along)),np.c_[along,.425-normal],along,normal

    def test_wall_frame_reflects_sides(self):
        config=FollowConfig();wall=dict(slope=0.,intercept_m=0.)
        points=np.array([[1.,.35],[1.,-.35]])
        self.assertAlmostEqual(wall_frame(points,config,'left',wall)[1][0],.075)
        self.assertAlmostEqual(wall_frame(points,config,'right',wall)[1][1],.075)

    def test_side_face_uses_spread_median(self):
        along=np.linspace(0.,.2,41);normal=.35+.03*np.sin(np.arange(41)*2.399)
        ids,points,along,normal=self.rows(along,normal)
        out=boundary_face(ids,points,along,normal,np.ones(41,bool),P)
        self.assertEqual(out['method'],'parallel_face_median')
        self.assertAlmostEqual(out['normal_m'],.35,delta=.008)
        self.assertLess(out['low_order_m'],.33)

    def test_rear_face_alone_keeps_lowest_return(self):
        along=np.full(30,1.1)+np.linspace(-.01,.01,30);normal=np.linspace(.35,.50,30)
        ids,points,along,normal=self.rows(along,normal)
        out=boundary_face(ids,points,along,normal,np.ones(30,bool),P)
        self.assertEqual(out['method'],'low_order');self.assertAlmostEqual(out['normal_m'],normal[0])

    def test_face_excludes_returns_past_the_band(self):
        # 경계 뒤쪽 깊은 반환은 면 구간이 아니다.
        along=np.r_[np.linspace(1.,1.2,20),np.linspace(1.2,1.3,20)];normal=np.r_[np.full(20,.35),np.linspace(.42,.60,20)]
        ids,points,along,normal=self.rows(along,normal)
        out=boundary_face(ids,points,along,normal,np.ones(40,bool),P)
        self.assertTrue(set(out['members'])<=set(range(20)))

    def test_second_object_behind_boundary_is_excluded(self):
        # 정지 경계 뒤쪽에 떨어진 다른 물체는 깊이와 무관하게 면에 포함되지 않는다.
        along=np.r_[np.linspace(1.,1.2,20),np.linspace(1.8,2.0,20)];normal=np.r_[np.full(20,.35),np.full(20,.45)]
        ids,points,along,normal=self.rows(along,normal)
        out=boundary_face(ids,points,along,normal,np.ones(40,bool),P)
        self.assertTrue(set(out['members'])<=set(range(20)))

    def test_second_object_at_same_depth_is_excluded_when_apart(self):
        along=np.r_[np.linspace(1.,1.2,20),np.linspace(1.8,2.0,20)];normal=np.full(40,.35)
        ids,points,along,normal=self.rows(along,normal)
        out=boundary_face(ids,points,along,normal,np.ones(40,bool),P)
        self.assertEqual(len(out['members']),20)

    def test_single_outlier_stays_inside_the_face(self):
        along=np.linspace(0.,.2,41);normal=np.full(41,.35);normal[20]=.60
        ids,points,along,normal=self.rows(along,normal)
        out=boundary_face(ids,points,along,normal,np.ones(41,bool),P)
        self.assertEqual(len(out['members']),40);self.assertNotIn(20,set(out['members']))

    def test_low_outlier_narrows_only(self):
        along=np.linspace(0.,.2,41);normal=np.full(41,.35);normal[0]=.07
        ids,points,along,normal=self.rows(along,normal)
        out=boundary_face(ids,points,along,normal,np.ones(41,bool),P)
        self.assertAlmostEqual(out['normal_m'],.07)

    def test_no_returns_in_window(self):
        ids,points,along,normal=self.rows(np.zeros(5),np.full(5,.85))
        self.assertIsNone(boundary_face(ids,points,along,normal,np.ones(5,bool),P))


class Wall(TestCase):
    def rows(self,count=80):
        along=np.linspace(-.3,1.3,count);normal=.03*np.sin(np.arange(count)*2.399)
        return np.arange(count),np.c_[along,.425-normal],along,normal

    def test_noisy_connected_wall_is_kept(self):
        ids,points,along,normal=self.rows()
        self.assertTrue(connected_wall_returns(ids,points,along,normal,P).all())

    def test_missing_ray_splits_short_pieces(self):
        ids,points,along,normal=self.rows()
        keep=np.ones(80,bool);keep[[20,40,60]]=False
        self.assertFalse(connected_wall_returns(ids[keep],points[keep],along[keep],normal[keep],P).any())

    def test_returns_away_from_line_stay_out(self):
        ids,points,along,normal=self.rows()
        normal=normal.copy();normal[30:45]=.15
        self.assertFalse(connected_wall_returns(ids,points,along,normal,P)[30:45].any())


class Plan(TestCase):
    def rows(self,left,right,inner_left=.35,inner_right=.35):
        return dict(left=dict(width_m=left,inner_normal_m=inner_left),right=dict(width_m=right,inner_normal_m=inner_right))

    def test_wider_corridor_decides_side_not_scan_age(self):
        self.assertEqual(choose_side(self.rows(.02,.045)),'right')
        self.assertEqual(choose_side(self.rows(.20,.02)),'left')
        self.assertEqual(choose_side(self.rows(.03,.033)),'right')   # 비슷해도 고정 편이 순위로 되돌아가지 않는다

    def test_blocked_only_when_inner_boundaries_cover_body_band(self):
        config=FollowConfig()
        self.assertTrue(straight_blocked(self.rows(0,0),config))
        self.assertTrue(straight_blocked(self.rows(0,0,.50,.20),config))   # 오른쪽 가장자리 표적
        self.assertFalse(straight_blocked(self.rows(0,0,.60,.10),config))  # 벽에 붙어 직진 띠 밖
        self.assertFalse(straight_blocked(self.rows(0,0,.78,.78),config))

    def test_forced_offset_needs_current_eligible_row(self):
        intent=PlannedIntent();intent.forced_offset=-.27
        scene=SimpleNamespace(t_ref_s=1.,source_first_s=.9,source_epoch=0)
        straight=dict(offset_m=0.,transition_m=.8,eligible=True,target_speed_mps=.3,steering_rad=0.,preview_return_progress_m=.8)
        blocked=dict(offset_m=-.27,transition_m=.8,eligible=False)
        selected=dict(offset_m=0.,transition_m=.8,speed_mps=.3,steering_rad=0.)
        out=intent.select(scene,dict(candidates=[straight,blocked],reason='x',selected=selected))
        self.assertEqual(out['selected']['offset_m'],0.);self.assertIsNone(intent.sign)
        allowed=dict(blocked,eligible=True,target_speed_mps=.2,steering_rad=-.1,preview_return_progress_m=.5)
        out=intent.select(scene,dict(candidates=[straight,allowed],reason='x',selected=selected))
        self.assertEqual(out['selected']['offset_m'],-.27);self.assertEqual(intent.sign,-1.)


class Age(TestCase):
    def test_abeam_age_follows_scan_order(self):
        rays=[]
        for i in range(360):
            angle=-math.pi+i*math.tau/360
            rays.append(SimpleNamespace(status='return',direction_xy=(np.cos(angle),np.sin(angle)),acquired_at_s=i/3600))
        scene=SimpleNamespace(rays=rays,source_last_s=359/3600)
        self.assertAlmostEqual(abeam_age(scene,'left',.08),.155,delta=.002)   # 왼쪽 통과: 표적이 먼저 쓸린 오른쪽 옆면
        self.assertAlmostEqual(abeam_age(scene,'right',.08),.105,delta=.002)


if __name__=='__main__':
    main()
