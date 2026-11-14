import copy
import unittest
from types import SimpleNamespace
from unittest.mock import patch
from arena_autonomy.race_runtime_policy import current_route_priority as policy
from arena_autonomy.race_runtime_policy.retained_recheck import _geometry
from arena_autonomy.runtime_dual_state import RuntimeDualProcess


class ProofTests(unittest.TestCase):
    def setUp(self):
        self.cfg=SimpleNamespace(traffic_recovery_trial=True,guard_observation_policy='selected_r181')
        self.output=[.4,.2,dict(handoff_state='FOLLOW',normal=dict(reason='local_follow',
            selector_choice='baseline',selector_mode='SOLO',selector_observation_policy=policy.POLICY,path_source_s=.99))]
        self.old=dict(reference_s=1.05,epoch=0,side='left',scan=dict(representation='native_measurements_v1',
            measurements=dict(source_epoch=0,acquired_at_s=[.99])))
        self.current=dict(reference_s=1.22,epoch=0,side='left',previous_steering_rad=.1,
            scan=dict(representation='native_measurements_v1',measurements=dict(source_epoch=0,acquired_at_s=[1.16])))
        self.geometry=dict(speed_mps=.4,steering_rad=.2,reason='rebase_current_follow',
            reference_s=1.22,path_source_s=.99,path_age_s=.23,source_first_s=1.16,
            original_reference_s=1.05,independent_guard_required=True,runtime_authorized=False)
        self.scene=SimpleNamespace(rays=[SimpleNamespace(status='return',direction_xy=(1.,0.))])

    def checked(self):
        with patch.object(_geometry,'rebase',return_value=copy.deepcopy(self.geometry)) as checked:
            out=policy.rebase(self.output,self.old,self.current,self.scene,object(),self.cfg)
            checked.assert_called_once()
            return out

    def reason(self,out,**changes):
        w=RuntimeDualProcess.__new__(RuntimeDualProcess)
        w.ready=dict(sequence=1,generation=0,route_check=dict(revision=0,retained=False),
            plan=dict(input=self.old,output=self.output),input=self.current,output=out)
        w.generation=w.revision=0;w.geometry_age_limit=.5;w.last_application=0.
        state=dict(now_s=1.31,previous_steering_rad=.1,side='left',motion_epoch=0,sensor_epoch=0)
        state.update(changes)
        return w.application_reason(w.ready,**state)

    def test_delayed_application_uses_existing_current_proof(self):
        out=self.checked();self.assertIsNone(self.reason(out))
        missing=copy.deepcopy(out);missing.pop('current_route_proof')
        self.assertEqual(self.reason(missing),'new_current_route_proof_required')
        self.assertEqual(out['speed_mps'],self.geometry['speed_mps'])
        self.assertEqual(out['reference_s'],self.geometry['reference_s'])

    def test_proof_identity_and_no_renewal_are_required(self):
        for key,value in [('accepted',False),('original_path_source_s',1.),('current_reference_s',1.23),
                ('current_evidence_first_s',1.17),('source_timestamp_renewed',True),('old_occupancy_permission_reused',True)]:
            with self.subTest(key=key):
                out=self.checked();out['current_route_proof'][key]=value
                self.assertEqual(self.reason(out),'new_current_route_proof_required')

    def test_actual_stop_does_not_create_positive_proof(self):
        self.geometry.update(speed_mps=0.,reason='rebase_current_gap_hold')
        out=self.checked();self.assertEqual(out['speed_mps'],0.);self.assertNotIn('current_route_proof',out)

    def test_lost_front_observation_keeps_stop(self):
        self.scene.rays=[]
        out=self.checked();self.assertEqual(out['reason'],'current_route_front_observation_lost')
        self.assertEqual(out['speed_mps'],0.);self.assertNotIn('current_route_proof',out)

    def test_source_command_geometry_and_epoch_boundaries_remain(self):
        out=self.checked()
        self.assertEqual(self.reason(out,now_s=1.43),'current_command_expired')
        self.assertEqual(self.reason(out,now_s=1.47),'current_source_expired')
        self.assertEqual(self.reason(out,motion_epoch=1),'current_input_epoch_or_side_changed')
        self.assertEqual(self.reason(out,previous_steering_rad=.2),'current_previous_request_changed')
        self.output[2]['normal']['path_source_s']=.8
        self.assertEqual(self.reason(out),'route_geometry_expired')

    def test_input_objects_are_unchanged(self):
        before=copy.deepcopy((self.output,self.old,self.current,self.geometry))
        self.checked();self.assertEqual(before,(self.output,self.old,self.current,self.geometry))


if __name__=='__main__':unittest.main()
