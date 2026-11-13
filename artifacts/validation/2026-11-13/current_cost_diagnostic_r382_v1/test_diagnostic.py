"""진단 분류가 변경을 숨기지 않는지 확인한다. 주행/ROS/후보 rebase는 호출하지 않는다."""
import math
import unittest

from diff_utils import comparison, recorded_result
from run import same_arm


class FakePolicy:
    DIAGNOSTIC='retry'


class FakeCompare:
    policy=FakePolicy
    from diff_utils import original_equal as equal_output


class Tests(unittest.TestCase):
    def test_count_change_is_not_float_noise(self):
        delta=comparison({'region':{'queries':3966,'masked_left':259}}, {'region':{'queries':3965,'masked_left':258}})
        self.assertFalse(delta['existing_comparator_equal'])
        self.assertEqual(delta['difference_count'],2)
        self.assertEqual(delta['categories'],{'region_count':2})

    def test_subtolerance_steering_is_retained(self):
        value=.26464391589836417
        delta=comparison({'steering_rad':value},{'steering_rad':math.nextafter(value,math.inf)})
        self.assertTrue(delta['existing_comparator_equal'])
        self.assertFalse(delta['exact_tree_equal'])
        self.assertEqual(delta['differences'][0]['category'],'command_or_measured_state')

    def test_structure_and_array_tail_are_retained(self):
        delta=comparison({'path_points':[[1.,2.],[3.,4.]],'accepted':False}, {'path_points':[[1.,2.]],'accepted':True,'new':7})
        self.assertFalse(delta['existing_comparator_equal'])
        self.assertEqual({tuple(d['path']) for d in delta['differences']},{('path_points','<length>'),('path_points',1),('accepted',),('new',)})

    def test_parent_cap_check_does_not_hide_target_change(self):
        result, check=recorded_result({'speed_mps':.2},{'new_plan_deceleration_cap':{'checked_target_mps':.1,'speed_mps':.05}})
        self.assertEqual(result['speed_mps'],.05)
        self.assertFalse(check['existing_comparator_equal'])

    def test_signed_zero_and_numeric_type_retained(self):
        delta=comparison([0.,1],[ -0.,1.])
        self.assertTrue(delta['existing_comparator_equal'])
        self.assertEqual(delta['difference_count'],2)

    def record(self, label='r371'):
        return dict(label=label,current={'reference_s':1.,'speed_mps':0.},output=[.09,0.,{'normal':{'selector_mode':'RELEASED','selector_choice':'parallel'}}])

    def accepted(self):
        before=dict(speed_mps=0.,steering_rad=.2,reason='stop',reference_s=1.,path_source_s=.9,
            original_reference_s=.9,path_points=[[0.,0.]],current_route_proof={'region':{'queries':3},'accepted':False})
        after={**before,'speed_mps':.09,'reason':'go','retry':{'applied':True,'unknown_filled':False},
            'current_route_proof':{'region':{'queries':3},'accepted':True},'current_side_presence':{'left':False,'right':False},
            'maximum_body_corner_lateral_m':.009}
        return before,after

    def test_same_arm_permission_checks_survive_diagnostic_mode(self):
        import copy
        record=self.record();before,after=self.accepted()
        self.assertTrue(same_arm(FakeCompare,record,copy.deepcopy(record),before,after)['applied'])
        for key,value in [('speed_mps',.11),('steering_rad',.3),('maximum_body_corner_lateral_m',.011)]:
            with self.assertRaises(AssertionError):same_arm(FakeCompare,record,copy.deepcopy(record),before,{**after,key:value})
        with self.assertRaises(AssertionError):
            same_arm(FakeCompare,self.record('holdout'),self.record('holdout'),before,after)

    def test_nonapplied_change_stops(self):
        record=self.record()
        before=dict(speed_mps=0.,reason='stop')
        with self.assertRaises(AssertionError):
            same_arm(FakeCompare,record,record,before,dict(speed_mps=.01,reason='stop'))


if __name__=='__main__':unittest.main()
