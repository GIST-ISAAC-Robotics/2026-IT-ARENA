import copy
import importlib.util
import json
from pathlib import Path
import time
import unittest
from unittest.mock import patch

from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.scene_motion import motion_scene
import arena_autonomy.decision_process as dp
import candidate
import rebase


class RuntimeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture=json.loads((rebase.ROOT/'artifacts/validation/2026-11-12/guard_current_r142_v1/fixture.json').read_bytes())
        cls.owner,cls.motion=make_controller(cls.fixture['header'])

    def test_frozen_input_wrapper_same_output(self):
        f=copy.deepcopy(self.fixture)
        data,scan,history=restore_call(f['current'],self.motion)
        scene=motion_scene(scan,history,data['reference_s'],timing_verified=data['timing_verified'],
            epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
        self.assertTrue(rebase.supported(f['original']['output']))
        actual=rebase.rebase(f['original']['output'],f['original']['input'],f['current'],scene,history,self.owner.config)
        self.assertEqual(actual,f['output'])

    def test_present_side_remains_on_existing_path(self):
        f=copy.deepcopy(self.fixture);f['original']['output'][2]['normal']['selector_sides']['left']['present']=True
        self.assertFalse(rebase.guard_scope(f['original']['output']))
        self.assertFalse(rebase.supported(f['original']['output']))

    def test_original_stop_not_rebased(self):
        f=copy.deepcopy(self.fixture);f['original']['output'][0]=0.
        self.assertFalse(rebase.supported(f['original']['output']))

    def test_spawn_original_planner_and_clean_exit(self):
        with patch.object(dp,'_worker',candidate.worker_entry):
            process=dp.DecisionProcess(self.fixture['header'])
        try:
            process.offer(self.fixture['original']['input'])
            deadline=time.monotonic()+5.;result=None
            while result is None and time.monotonic()<deadline:
                result=process.pump();time.sleep(.005)
            self.assertIsNotNone(result)
            self.assertIsNone(process.fault)
        finally:
            stopped=process.close()
        self.assertEqual(stopped,dict(forced=False,exitcode=0))


if __name__=='__main__':unittest.main()
