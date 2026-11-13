"""하네스의 diff·부모 cap·실패 불변·단일 표본 집계만. 제어기/ROS import 없음."""
import copy
import json
import unittest
from diff_utils import comparison, recorded_result
from support import DIAGNOSTIC, current_invariants, statistics


class HelpersTest(unittest.TestCase):
    def test_exact_diff_keeps_tiny_numeric_and_region_count(self):
        got=comparison({'path_points':[[.1,.2]],'region':{'queries':10}},
                       {'path_points':[[.1,.20000000000000004]],'region':{'queries':9}})
        self.assertFalse(got['exact_tree_equal']);self.assertEqual(got['difference_count'],2)
        self.assertEqual(got['categories'],{'path_coordinate':1,'region_count':1})

    def test_command_diff_and_signed_zero_remain_visible(self):
        got=comparison([0.,.1,{}],[-0.,.10000000000000002,{}])
        self.assertFalse(got['exact_tree_equal'])
        self.assertEqual(got['categories'],{'command_or_measured_state':2})
        # r382의 kind 중복 오류처럼 분류 key를 덮지 않고 구조적으로 보존한다.
        record=dict(comparison_kind='recorded_plan',index=0,**got)
        self.assertEqual(json.loads(json.dumps(record))['differences'][0]['kind'],'value')

    def test_parent_cap_checked_target_cannot_hide_bad_evaluator_speed(self):
        before={'speed_mps':1.,'reason':'same'}
        expected={'speed_mps':.8,'reason':'same','new_plan_deceleration_cap':{'speed_mps':.8,'checked_target_mps':1.}}
        after,check=recorded_result(before,expected)
        self.assertTrue(check['exact_tree_equal']);self.assertEqual(after,expected)
        bad=dict(before,speed_mps=.9);restored,bad_check=recorded_result(bad,expected)
        self.assertEqual(restored,expected)
        self.assertFalse(bad_check['exact_tree_equal'])
        self.assertEqual(before,{'speed_mps':1.,'reason':'same'})

    def test_non_target_or_rejected_candidate_cannot_change_command(self):
        before={'speed_mps':0.,'reason':'unknown'}
        self.assertEqual(current_invariants(before,copy.deepcopy(before),False),'non_target')
        with self.assertRaisesRegex(AssertionError,'non-target'):
            current_invariants(before,dict(before,speed_mps=.1),False)
        diagnostic=dict(additional_target_attempts=0,final_full_checks=0,unknown_filled=False,
            source_timestamp_renewed=False,applied=False,proposal={'accepted':False})
        self.assertEqual(current_invariants(before,{**before,DIAGNOSTIC:diagnostic},True),'proposal_rejected')
        with self.assertRaisesRegex(AssertionError,'rejected candidate'):
            current_invariants(before,{**before,'speed_mps':.1,DIAGNOSTIC:diagnostic},True)

    def test_single_sample_is_not_presented_as_p95(self):
        self.assertEqual(statistics([]),{'n':0})
        self.assertEqual(statistics([101.]),{'n':1,'single_ms':101.,'over_100ms':1,'over_200ms':0})
        result=statistics([20.,100.,201.])
        self.assertEqual(result['p50_ms'],100.);self.assertEqual(result['max_ms'],201.)
        self.assertEqual(result['over_100ms'],1);self.assertEqual(result['over_200ms'],1)


if __name__=='__main__':unittest.main(verbosity=2)
