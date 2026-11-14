"""추정 입력 선택/원천 대조/단일 현재 목표. 합성 geometry이며 센서 시각 실측이 아니다."""
import copy
from dataclasses import asdict, replace
import importlib.util
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np
from arena_autonomy.crop_edge_selection import (SIMULATION_SCOPE as SIM, ESTIMATED_SCOPE as EST,
    validate_node_selection, component_scope, timing_matches, configure_follow_observation)
from arena_autonomy.runtime_node_backend import (evaluator_header, validate_corner_selection,
    POLICY, CORNER_POLICY)
from arena_autonomy.runtime_dual_state import current_request
from arena_autonomy.race_runtime_policy import crop_current_candidate as crop
from arena_autonomy.race_runtime_policy import corner_candidate as corner
from arena_autonomy.race_runtime_policy.crop_current_scope import same_scan_scope
from arena_autonomy.race_runtime_policy.crop_current_proof import check
from arena_autonomy.race_runtime_policy.profile_evidence import profile_evidence
from arena_autonomy.race_runtime_policy.evaluator import Evaluator
from arena_autonomy.measurement_scan import MeasurementScan
from arena_autonomy.local_scene import scene_from_measurements, blind_spec
from arena_autonomy.partial_observation import PartialObservationPolicy
from arena_autonomy.follow_trace import FollowTrace, freeze
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.bypass_handoff import HandoffExecutionConfig


def node(scope=EST):
    estimated=scope==EST
    params={'use_sim_time':not estimated,'lidar_input_representation':
            'native_estimated_v1' if estimated else 'native_simulation_v1'}
    return NS(observation_input_scope=scope, execution_environment='hardware_runtime_candidate_v1',
        hardware_inputs=estimated,motion=NS(mode='both',verified=not estimated,
        timing_policy='host_estimated_v1' if estimated else 'verified_only'),
        runtime_dual_selected=True,current_corner_budget=CORNER_POLICY,
        has_parameter=lambda k:k in params,get_parameter=lambda k:NS(value=params[k]),params=params)


def options():
    return dict(runtime_selected=True,use_sim_time=False,motion_verified=False,
        motion_time_policy='host_estimated_v1',observation_scope=EST,
        execution_environment='hardware_runtime_candidate_v1',
        lidar_input_representation='native_estimated_v1',motion_mode='both')


def fixture(scope=EST):
    n=500;values=[2.]*n
    members=[124,125,126]
    for i in members:values[i]=math.inf
    raw=MeasurementScan(tuple(values),tuple(i*math.tau/n for i in range(n)),
        tuple(1+i*.1/n for i in range(n)),time_basis='unverified_estimate' if scope==EST else 'simulation_acquisition')
    part=dict(scope=component_scope(scope),source_indices=members,inside_source_indices=[124,125],
              flank_source_indices=[123,127])
    output=[.5,.1,dict(handoff_state='FOLLOW',normal=dict(reason='local_follow',selector_mode='SOLO',
        selector_choice='baseline',boundary_from_memory=False,partial_observation=dict(allowed=True,
            reason='bounded_side_missing',unknown_filled=False,latch_reason=None,crop_edge_component=part)))]
    current=dict(reference_s=1.12,epoch=0,side='left',previous_steering_rad=.1,dt_s=.1,speed_mps=0.,
        timing_verified=scope!=EST,timing_policy='host_estimated_v1' if scope==EST else 'verified_only',
        scan=dict(representation='native_measurements_v1',measurements=freeze(asdict(raw))))
    data=current_request(output,copy.deepcopy(current),current,retained=False,crop_current_selection=True)
    before=dict(speed_mps=0.,steering_rad=.1,reference_s=1.12,path_source_s=1.,original_reference_s=1.12,
        path_points=[[0.,0.],[.3,0.]],reason='route_current_large_motion_region_unobserved',current_route_proof={})
    cfg=NS(crop_edge_component_trial=True,max_steering_rad=.45,length_m=.3,width_m=.2)
    return raw,part,data,before,cfg


def scene(raw,profile=0.):
    return scene_from_measurements(raw.ranges_m,angles_rad=raw.angles_rad,acquired_at_s=raw.acquired_at_s,
        t_ref_s=1.12,range_min=raw.range_min_m,range_max=raw.range_max_m,
        source_frame_id=raw.source_frame_id,source_epoch=raw.source_epoch,
        sensor_xy_m=(0.,0.),rear_half_angle_deg=profile,preserve_sampling=True)


class EstimatedObservationTests(unittest.TestCase):
    def test_nodes_require_exact_physical_estimated_combination(self):
        validate_node_selection(node(),True,observation_scope=EST)
        validate_node_selection(node(SIM),True)
        for fault in ('sim_clock','verified','policy','representation','environment','motion','unknown_scope'):
            n=node();scope=EST
            if fault=='sim_clock':n.params['use_sim_time']=True
            elif fault=='verified':n.motion.verified=True
            elif fault=='policy':n.motion.timing_policy='verified_only'
            elif fault=='representation':n.params['lidar_input_representation']='native_simulation_v1'
            elif fault=='environment':n.execution_environment='hardware_shadow_v1'
            elif fault=='motion':n.motion.mode='gyro'
            elif fault=='unknown_scope':scope='estimated'
            with self.subTest(fault=fault),self.assertRaises(ValueError):
                validate_node_selection(n,True,observation_scope=scope)
        with self.assertRaises(ValueError):validate_node_selection(node(),True)  # default remains simulation

    def test_corner_requires_explicit_scope_and_registered_runtime(self):
        self.assertTrue(validate_corner_selection(CORNER_POLICY,**options()))
        for key,value in [('runtime_selected',False),('use_sim_time',True),('motion_verified',True),
            ('motion_verified',0),('motion_time_policy','verified_only'),('observation_scope',SIM),
            ('execution_environment','hardware_shadow_v1'),('lidar_input_representation','laser_scan'),
            ('motion_mode','wheel')]:
            args=options();args[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):validate_corner_selection(CORNER_POLICY,**args)
        with self.assertRaises(ValueError):validate_corner_selection('off',**options())

    def test_sim_header_unchanged_estimated_header_records_actual_selection(self):
        initial={'schema':5,'numeric_execution':'native_v1'};before=copy.deepcopy(initial)
        sim=evaluator_header(node(SIM),initial)
        self.assertEqual(sim,dict(initial,runtime_policy=POLICY,runtime_time_policy='verified_only',
            current_corner_budget=CORNER_POLICY,current_corner_environment=dict(use_sim_time=True,
                motion_timing_verified=True,backend='native2slot_v1')))
        est=evaluator_header(node(),initial)
        self.assertEqual(est['observation_input_scope'],EST)
        self.assertEqual(est['runtime_time_policy'],'host_estimated_v1')
        self.assertIs(est['current_corner_environment']['use_sim_time'],False)
        self.assertIs(est['current_corner_environment']['motion_timing_verified'],False)
        self.assertEqual(initial,before)

    def test_worker_rechecks_header_before_controller_creation(self):
        selected=evaluator_header(node(),{'numeric_execution':'native_v1'})
        for key,value in [('use_sim_time',True),('motion_timing_verified',True),
            ('execution_environment','default'),('lidar_input_representation','native_simulation_v1'),
            ('backend','other'),('motion_mode','gyro')]:
            bad=copy.deepcopy(selected);bad['current_corner_environment'][key]=value
            with self.subTest(key=key),patch('arena_autonomy.decision_process.make_controller') as create:
                with self.assertRaises(ValueError):Evaluator('current',bad)
                create.assert_not_called()
        dummy=NS()
        with patch('arena_autonomy.numeric_kernels.configure') as numeric,patch(
                'arena_autonomy.decision_process.make_controller',return_value=(dummy,dummy)):
            e=Evaluator('current',selected);self.assertEqual(e.observation_scope,EST);e.close()
            numeric.assert_called_once_with('native_v1')  # contract only; no Windows native build

    def test_worker_rejects_input_timing_forgery_before_rebase(self):
        e=Evaluator.__new__(Evaluator);e.stage='current';e.time_policy='host_estimated_v1'
        e.corner_budget=True;e.observation_scope=EST
        _,_,data,_,_=fixture()
        for field,value in [('timing_verified',True),('timing_verified',0),('timing_policy','verified_only')]:
            bad=copy.deepcopy(data);bad['current'][field]=value
            with self.subTest(field=field),self.assertRaises(ValueError):e(bad)
        bad=copy.deepcopy(data);bad['current']['scan']['measurements']['time_basis']='simulation_acquisition'
        with self.assertRaises(ValueError):e(bad)

    def test_full_parent_scan_equality_and_false_metadata_preserved(self):
        raw,_,data,_,cfg=fixture();original=copy.deepcopy(data)
        self.assertTrue(same_scan_scope(data,cfg,registered_environment=True,observation_scope=EST))
        self.assertFalse(same_scan_scope(data,cfg,registered_environment=True))
        for key,value in [('submitted_timing_verified',True),('same_scan',False),
            ('submitted_time_basis','simulation_acquisition'),('submitted_timing_policy','verified_only')]:
            bad=copy.deepcopy(data);bad['crop_current_input'][key]=value
            self.assertFalse(same_scan_scope(bad,cfg,registered_environment=True,observation_scope=EST))
        submitted=copy.deepcopy(data['current']);changed=copy.deepcopy(submitted)
        changed['scan']['measurements']['ranges_m'][0]=.11
        proof=current_request(data['output'],submitted,changed,retained=False,crop_current_selection=True)
        self.assertFalse(same_scan_scope(proof,cfg,registered_environment=True,observation_scope=EST))
        self.assertEqual(data,original);self.assertEqual(raw.time_basis,'unverified_estimate')

    def test_crop_proof_accepts_estimated_geometry_without_changing_source(self):
        raw,component,_,_,_=fixture();s=scene(raw);q=np.array([[0.,.2]])
        result=check(s,raw,q,np.full(500,.03),component,all_queries=q,motion_epoch=0,
            observation_scope=EST,timing_verified=False,timing_policy='host_estimated_v1')
        self.assertTrue(result['accepted'],result)
        self.assertFalse(result['unknown_filled']);self.assertTrue(result['residual_hidden_object_risk'])
        self.assertFalse(check(s,raw,q,np.full(500,.03),component,all_queries=q,motion_epoch=0)['accepted'])
        for epoch,verified in [(1,False),(0,True)]:
            bad=check(s,raw,q,np.full(500,.03),component,all_queries=q,motion_epoch=epoch,
                observation_scope=EST,timing_verified=verified,timing_policy='host_estimated_v1')
            self.assertFalse(bad['accepted'])
        self.assertEqual(raw.time_basis,'unverified_estimate')

    def test_crop_proof_keeps_finite_obstacle_and_invalid_nonreturn_rejections(self):
        raw,component,_,_,_=fixture();q=np.array([[0.,.2]])
        for value in (.2,math.nan,-math.inf):
            ranges=list(raw.ranges_m);ranges[125]=value;bad=replace(raw,ranges_m=tuple(ranges))
            result=check(scene(bad),bad,q,np.full(500,.03),component,all_queries=q,motion_epoch=0,
                observation_scope=EST,timing_verified=False,timing_policy='host_estimated_v1')
            with self.subTest(value=value):self.assertFalse(result['accepted'])

    def test_profile_evidence_preserves_range_mask_and_epoch_checks(self):
        raw,_,_,_,_=fixture();profile={'sectors_deg':[[160.,2.]],'range_limits_m':[[.16,.18]]}
        s=scene(raw,profile);sectors=blind_spec(profile)[1]
        bands=profile_evidence(s,raw,sectors,profile,0,False,observation_scope=EST,timing_policy='host_estimated_v1')
        self.assertEqual(bands,((.16,.18),))
        for epoch,verified in [(1,False),(0,True)]:
            with self.subTest(epoch=epoch),self.assertRaises(ValueError):
                profile_evidence(s,raw,sectors,profile,epoch,verified,observation_scope=EST,timing_policy='host_estimated_v1')
        with self.assertRaises(ValueError):profile_evidence(s,raw,sectors,profile,0,False)

    def test_current_crop_one_tenth_applied_before_core_region_and_proof(self):
        _,_,data,before,cfg=fixture();saved=copy.deepcopy(data);calls=[]
        def core(output,*args):
            calls.append(('core',output[0]));return {**before,'speed_mps':output[0],'reason':'rebase_current_follow'}
        def region(scene,measured,target,*args,context):
            calls.append(('region',target));context.update(queries=np.array([[0.,.2]]),free=np.array([False]),
                masked=np.array([False]),padding=np.zeros(500))
            return dict(queries=1,unobserved=True)
        def proof(*args,**kwargs):
            calls.append(('proof',kwargs['timing_verified'],kwargs['timing_policy']));return {'accepted':True}
        with patch.object(crop.ps,'motion_region',side_effect=region),patch.object(crop.ps,'lateral_config',return_value=None),\
             patch.object(crop.ps,'region_blocked',return_value=None),patch.object(crop,'check',side_effect=proof):
            out=crop.try_from_current_result(before,data,None,None,cfg,registered_environment=True,
                core_rebase=core,observation_scope=EST)
        self.assertEqual(calls,[('core',.1),('region',.1),('proof',False,'host_estimated_v1')])
        self.assertEqual(out['speed_mps'],.1);self.assertEqual(out[crop.DIAGNOSTIC]['attempted_targets'],1)
        self.assertEqual(out[crop.DIAGNOSTIC]['final_full_checks'],1);self.assertEqual(data,saved)
        self.assertIs(out[crop.DIAGNOSTIC]['timing_verified'],False)
        self.assertEqual(out[crop.DIAGNOSTIC]['timing_policy'],'host_estimated_v1')
        self.assertEqual(out[crop.DIAGNOSTIC]['time_basis'],'unverified_estimate')

    def test_positive_and_outside_measured_scope_do_not_call_new_candidate(self):
        _,_,data,before,cfg=fixture()
        for speed,measured in [(.4,0.),(0.,-.061225),(0.,.150001),(0.,math.nan)]:
            old={**before,'speed_mps':speed};data['current']['speed_mps']=measured
            with patch.object(crop,'check') as proof,patch.object(crop.ps,'motion_region') as region:
                core=unittest.mock.Mock()
                result=crop.try_from_current_result(old,data,None,None,cfg,registered_environment=True,
                    core_rebase=core,observation_scope=EST)
                self.assertIs(result,old);core.assert_not_called();region.assert_not_called();proof.assert_not_called()

    def test_rejected_estimated_candidate_does_not_try_second_target(self):
        _,_,data,before,cfg=fixture();core=unittest.mock.Mock(return_value={**before,'speed_mps':0.})
        result=crop.try_from_current_result(before,data,None,None,cfg,registered_environment=True,
            core_rebase=core,observation_scope=EST)
        self.assertEqual(result['speed_mps'],0.);self.assertEqual(core.call_count,1)
        self.assertEqual(core.call_args.args[0][0],.1)

    def test_corner_one_tenth_precedes_full_recheck(self):
        _,_,data,before,cfg=fixture();calls=[]
        def core(output,*args):calls.append(output[0]);return {**before,'speed_mps':output[0],'reason':'rebase_current_follow'}
        def region(scene,measured,target,*args,context):
            calls.append(target);return dict(queries=1,unobserved=False,distance_m=.03)
        with patch.object(corner,'eligible',return_value=True),patch.object(corner,'propose',return_value=dict(
            accepted=True,proposed_target_mps=.14,allowed_distance_m=.05)),patch.object(corner.ps,'motion_region',side_effect=region),\
            patch.object(corner.ps,'lateral_config',return_value=None),patch.object(corner.ps,'region_blocked',return_value=None):
            result=corner.try_from_current_result(before,data['output'],data['submitted'],data['current'],None,None,cfg,
                core_rebase=core,proof_callback=unittest.mock.Mock(),require_current_region=True,target_cap=.1)
        self.assertEqual(calls,[.1,.1]);self.assertEqual(result['speed_mps'],.1)

    def test_default_instance_and_trace_shape_preserved(self):
        p=PartialObservationPolicy();follow=NS(partial_policy=p);before=dict(vars(p))
        configure_follow_observation(follow,SIM);self.assertEqual(vars(p),before)
        configure_follow_observation(follow,EST);self.assertEqual(p.observation_input_scope,EST)
        p.reset();self.assertEqual(p.observation_input_scope,EST)
        with tempfile.TemporaryDirectory() as directory:
            def record(name,scope=SIM):
                path=Path(directory)/name
                t=FollowTrace(str(path),MotionConfig(),FollowConfig(),mode='g1_g2_handoff_trial',
                    execution_config=HandoffExecutionConfig(),current_corner_budget=CORNER_POLICY,
                    observation_input_scope=scope);t.close()
                return json.loads(path.read_text().splitlines()[0])['data']
            a=record('sim');b=record('est',EST)
            self.assertNotIn('observation_input_scope',a)
            self.assertEqual(b.pop('observation_input_scope'),EST);self.assertEqual(a,b)


if __name__=='__main__':unittest.main()
