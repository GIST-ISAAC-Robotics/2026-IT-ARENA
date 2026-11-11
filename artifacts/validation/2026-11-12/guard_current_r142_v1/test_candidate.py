import copy
import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np

from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.scene_motion import motion_scene
import candidate


class GuardCurrentTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture = json.loads((Path(__file__).parent/'fixture.json').read_bytes())
        cls.owner, cls.motion = make_controller(cls.fixture['header'])

    def inputs(self):
        return copy.deepcopy(self.fixture)

    def run_candidate(self, fixture):
        current,scan,history = restore_call(fixture['current'],self.motion)
        scene = motion_scene(scan,history,current['reference_s'],timing_verified=current['timing_verified'],
                             epoch=current['epoch'],rear_half_angle_deg=current['rear_half_angle_deg'])
        result = candidate.rebase(fixture['original']['output'],fixture['original']['input'],
                                  current,scan,scene,history,self.owner.config)
        return result

    def test_same_input_and_state_are_unchanged(self):
        data = self.inputs(); before = copy.deepcopy(data)
        result = self.run_candidate(data)
        self.assertEqual(result,self.fixture['output'])
        self.assertEqual(data,before)
        self.assertGreater(result['speed_mps'],0)
        self.assertEqual(result['rebased_selector_mode'],'GUARD')
        self.assertFalse(result['state_transition'])
        self.assertTrue(result['independent_guard_required'])
        self.assertFalse(result['runtime_authorized'])

    def test_path_expiry_is_not_extended(self):
        data=self.inputs()
        data['original']['output'][2]['normal']['path_source_s']=data['current']['reference_s']-.30001
        self.assertEqual(self.run_candidate(data)['reason'],'rebase_path_source_expired_or_invalid')

    def test_epoch_change(self):
        data=self.inputs();data['current']['epoch']+=1
        self.assertEqual(self.run_candidate(data)['reason'],'rebase_epoch_or_side_changed')

    def test_previous_steering_change(self):
        data=self.inputs();data['current']['previous_steering_rad']+=.01
        self.assertEqual(self.run_candidate(data)['reason'],'rebase_current_input_invalid')

    def test_negative_or_high_actual_speed(self):
        for speed in (-.01,.351):
            with self.subTest(speed=speed):
                data=self.inputs();data['current']['speed_mps']=speed
                self.assertEqual(self.run_candidate(data)['reason'],'guard_rebase_speed_outside_scope')

    def test_missing_anchor(self):
        data=self.inputs();data['original']['output'][2]['normal']['selector_guard_candidate']['guarded_sides']={}
        self.assertEqual(self.run_candidate(data)['reason'],'guard_rebase_original_anchor_missing')

    def test_unknown_original_partial_is_not_inherited(self):
        data=self.inputs();data['original']['output'][2]['normal']['partial_observation']['allowed']=False
        self.assertEqual(self.run_candidate(data)['reason'],'guard_rebase_original_partial_outside_scope')

    def test_new_present_side_is_not_released(self):
        original=candidate.ps.threat_view
        def newly_present(*args):
            view=original(*args);view['right']['present']=True
            return view
        with patch.object(candidate.ps,'threat_view',newly_present):
            self.assertEqual(self.run_candidate(self.inputs())['reason'],'guard_rebase_current_side_present')

    def modify_scan(self,lo,hi,value):
        data=self.inputs();scan=data['current']['scan']['measurements']
        for i,a in enumerate(scan['angles_rad']):
            angle=(math.degrees(a)+180)%360-180
            if lo<=angle<=hi:scan['ranges_m'][i]=value
        return data

    def test_new_near_obstacle(self):
        result=self.run_candidate(self.modify_scan(-15,15,.12))
        self.assertEqual(result['speed_mps'],0)

    def test_front_nan_is_not_free(self):
        # freeze의 표식 그대로 사용한다. 수신 값을 정상 반환으로 바꾸지 않는다.
        result=self.run_candidate(self.modify_scan(-10,10,{'nonfinite':'nan'}))
        self.assertEqual(result['speed_mps'],0)

    def test_wide_rear_nonreturn_is_not_free(self):
        result=self.run_candidate(self.modify_scan(101,145,{'nonfinite':'inf'}))
        self.assertEqual(result['speed_mps'],0)

    def test_original_no_positive_command(self):
        data=self.inputs();data['original']['output'][0]=0
        self.assertIsNone(self.run_candidate(data))

    def test_do_not_copy_gap_permission(self):
        result=self.run_candidate(self.inputs())
        self.assertGreater(result['region']['unobserved'],0)
        self.assertTrue(result['current_gap_proof']['accepted'])
        self.assertFalse(result['current_gap_proof']['unknown_filled'])
        self.assertLessEqual(result['region']['masked_tail_extent_m'],.01)
        self.assertEqual(result['region']['masked_front_samples'],0)

    def test_exceeded_anchor_does_not_block_nonapproaching_motion(self):
        data=self.inputs();normal=data['original']['output'][2]['normal']
        normal['selector_sides']['right'].update(seen=True,present=False,passed=False)
        normal['selector_guard_candidate']['guarded_sides']['right']={'anchor_m':-1.}
        result=self.run_candidate(data)
        self.assertGreater(result['speed_mps'],0)
        side=result['guarded_sides']['right']
        self.assertLessEqual(side['predicted_m'],side['current_m']+1e-9)

    def test_curve_toward_unreleased_side_on_straight_reference(self):
        # 저장 장면은 경로를 잘 따라 두 측면 모두 추가 접근이 없다. 직선
        # 기준에 같은 곡선 명령을 적용하는 별도 기하 반례로 누적 접근을 검사한다.
        data=self.inputs();normal=data['original']['output'][2]['normal']
        normal['selector_guard_candidate']['guarded_sides']['left']['anchor_m']=-1.
        original=candidate.ps.threat_view
        def straight_frame(*args):
            view=original(*args)
            view['frame']=candidate.ps.mx.ExecutionFrame(np.array([[-.3,0.],[0.,0.],[.4,0.],[.8,0.]]))
            return view
        with patch.object(candidate.ps,'threat_view',straight_frame):
            self.assertEqual(self.run_candidate(data)['reason'],'guard_rebase_toward_unreleased_side')


if __name__=='__main__':unittest.main()
