"""보고 복구의 kind 충돌/부정확한 성공 판정을 확인하는 데이터 전용 검사."""
import copy
import tempfile
from pathlib import Path
import unittest

from recover import load_diff,read_jsonl,sensitive_differences,summary,validate_same_arm,DIAGNOSTIC


class Tests(unittest.TestCase):
    def test_comparison_and_change_kinds_both_survive(self):
        delta=dict(kind='value',path=['steering_rad'],category='command_or_measured_state',
            expected=.1,actual=.10000000000000002,existing_comparator_equal=True)
        checks=[dict(kind='x86_original_current',index=2,label='r371',differences=[delta])]
        output=sensitive_differences(checks)
        self.assertEqual(output[0]['comparison_kind'],'x86_original_current')
        self.assertEqual(output[0]['change_kind'],'value')
        self.assertEqual(output[0]['difference'],delta)

    def test_partial_line_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'rows.jsonl';path.write_bytes(b'{"a":1}')
            with self.assertRaises(AssertionError):read_jsonl(path)
            path.write_bytes(b'{"a":1}\n')
            self.assertEqual(read_jsonl(path),[{'a':1}])

    def test_percentiles_and_thresholds(self):
        result=summary([{'rebase_wall_ms':n} for n in [0,50,100,150,200,250]])
        self.assertEqual(result['p50_ms'],125)
        self.assertEqual(result['p95_ms'],237.5)
        self.assertEqual(result['above_100ms'],3)
        self.assertEqual(result['above_200ms'],1)

    def test_nonfinite_timing_rejected(self):
        for value in [float('nan'),float('inf'),-1]:
            with self.assertRaises(AssertionError):summary([{'rebase_wall_ms':value}])

    def fixture(self):
        record=dict(label='r371',current={'reference_s':1.,'speed_mps':0.},output=[.09,0.,{'normal':{'selector_mode':'RELEASED','selector_choice':'parallel'}}])
        before=dict(speed_mps=0.,steering_rad=.2,reason='stop',reference_s=1.,path_source_s=.9,
            original_reference_s=.9,path_points=[[0.,0.]],current_route_proof={'region':{'queries':3},'accepted':False})
        after={**before,'speed_mps':.09,'reason':'go',DIAGNOSTIC:{'applied':True,'unknown_filled':False},
            'current_route_proof':{'region':{'queries':3},'accepted':True},'current_side_presence':{'left':False,'right':False},
            'maximum_body_corner_lateral_m':.009}
        return record,dict(index=0,applied=True,before=before,after=after)

    def test_reaudit_still_rejects_permission_violation(self):
        record,row=self.fixture();diff=load_diff()
        self.assertTrue(validate_same_arm(record,row,diff)['applied'])
        for key,value in [('speed_mps',.11),('steering_rad',.3),('maximum_body_corner_lateral_m',.011)]:
            changed=copy.deepcopy(row);changed['after'][key]=value
            with self.assertRaises(AssertionError):validate_same_arm(record,changed,diff)
        record['label']='holdout_r279'
        with self.assertRaises(AssertionError):validate_same_arm(record,row,diff)

    def test_x86_count_mismatch_not_reclassified_as_success(self):
        delta=load_diff().comparison({'queries':3966},{'queries':3965})
        self.assertFalse(delta['existing_comparator_equal'])
        self.assertEqual(delta['differences'][0]['category'],'region_count')


if __name__=='__main__':unittest.main()
