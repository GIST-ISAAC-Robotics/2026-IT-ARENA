"""예측 좌표·시간·후보 범위를 해석적인 대칭성과 대조 조건으로 확인한다."""
import unittest
import numpy as np
from analyze import predict, initial_scene
from prototype import rerank, choose_initial_intent
from arena_autonomy.bypass_entry_trial import EntryIntent
from arena_autonomy.local_follow import FollowConfig


class PredictionTests(unittest.TestCase):
    def test_straight_motion_and_speed_bound(self):
        x = np.linspace(-.0675, 1.6, 90)
        result = predict(np.c_[x, np.zeros_like(x)], np.array([[1., 2.]]), np.array([.055]), FollowConfig(), .1)
        self.assertFalse(result['padded_interference'])
        self.assertEqual(result['stop_reason'], 'observed_preview_horizon')
        self.assertTrue(all(abs(r['center_pose'][1]) < 1e-12 for r in result['rows']))
        self.assertTrue(all(0 <= r['speed_mps'] <= .35 for r in result['rows']))
        self.assertLess(abs(result['rows'][-1]['center_pose'][0]-1.25), .0071)

    def test_reflection_equivariance(self):
        x = np.linspace(-.0675, 1.6, 90)
        t = np.clip((x+.0675)/1.2, 0, 1)
        y = .24*(10*t**3-15*t**4+6*t**5)
        points = np.array([[1.1,-.075],[1.1,.075],[.5,.425],[1.,.425]])
        config = FollowConfig()
        a = predict(np.c_[x,y],points,np.full(len(points),.055),config,.1)
        b = predict(np.c_[x,-y],points*np.array([1.,-1.]),np.full(len(points),.055),config,.1)
        self.assertEqual(len(a['rows']),len(b['rows']))
        for first,second in zip(a['rows'],b['rows']):
            np.testing.assert_allclose(first['center_pose'],np.array(second['center_pose'])*[1,-1,-1],atol=1e-12)
            self.assertAlmostEqual(first['padded_point_gap_m'],second['padded_point_gap_m'])

    def test_reranking_never_adds_rejected_candidate(self):
        config, scene, comparison = initial_scene()
        allowed = [r for r in comparison['candidates'] if r.get('eligible') and r['offset_m']==0.]
        self.assertTrue(allowed)
        for row in comparison['candidates']:
            row['eligible'] = row in allowed
        result = rerank(scene,comparison,config)
        self.assertEqual(result['selected']['offset_m'],0.)
        self.assertFalse(result['control_authorized'])
        self.assertFalse(result['prototype_ranking']['future_occupancy_or_observation_permission'])

    def test_initial_intent_uses_same_prediction_as_ranking(self):
        config, scene, comparison = initial_scene()
        ranked = rerank(scene,comparison,config)
        intent = EntryIntent()
        chosen = choose_initial_intent(intent,scene,ranked,EntryIntent.select)
        self.assertGreater(chosen['prototype_tracking_gain_m'],.04)
        self.assertEqual(chosen['selected'],ranked['selected'])
        self.assertEqual(chosen['selected']['transition_m'],.9)
        self.assertEqual(intent.sign,1.)

    def test_no_tracking_gain_stays_straight(self):
        config, scene, comparison = initial_scene()
        ranked = rerank(scene,comparison,config)
        for row in ranked['candidates']:
            if row.get('eligible'):
                row['prototype_tracking_progress_m'] = .4
        intent = EntryIntent()
        chosen = choose_initial_intent(intent,scene,ranked,EntryIntent.select)
        self.assertEqual(chosen['selected']['offset_m'],0.)
        self.assertIsNone(intent.sign)


if __name__ == '__main__':
    unittest.main()
