"""PC fixture/AST만 검사한다. rclpy·ROS·ARM·WSL을 실행하지 않는다."""
import ast
import copy
from pathlib import Path
from types import SimpleNamespace as NS
import sys
import unittest

A=Path(__file__).resolve().parent
sys.path.insert(0,str(A/'stage'))
from observation import ControlReceipt,GuardReceipt,ObservedPublisher,ScalarProbe,hook_controller
from contracts import prepared_profile


def message():
    return NS(header=NS(stamp=NS(sec=7,nanosec=123456789)),drive=NS(speed=.4,steering_angle=.1))


def clock():
    return NS(now=lambda:NS(nanoseconds=7300000000))


class StageTests(unittest.TestCase):
    def test_super_control_once_and_only_production_drains(self):
        counts=NS(control=0,drain=0)
        class Base:
            def control(self):
                counts.control+=1
                self.motion.drain_pending(budget_s=.006)
                return 'original-result'
        class Owner(ControlReceipt,Base):pass
        owner=Owner();owner._receipt=ScalarProbe('controller');owner.get_clock=clock
        def drain(**kw):
            counts.drain+=1
            owner.motion.last_drain=dict(total=3,duration_ms=.4,bounded_stop=False)
        owner.motion=NS(drain_pending=drain);owner.command=message()
        before=copy.deepcopy(vars(owner.command.header.stamp))
        self.assertEqual(owner.control(),'original-result')
        self.assertEqual((counts.control,counts.drain),(1,1))
        self.assertEqual(vars(owner.command.header.stamp),before)

    def test_guard_receives_original_message_once(self):
        calls=[]
        class Base:
            def on_command(self,msg):
                calls.append(msg);self.command=msg;return 29
        class Owner(GuardReceipt,Base):pass
        owner=Owner();owner._receipt=ScalarProbe('guard');owner.get_clock=clock
        msg=message();before=copy.deepcopy(vars(msg.header.stamp))
        self.assertEqual(owner.on_command(msg),29)
        self.assertEqual(calls,[msg]);self.assertIs(owner.command,msg)
        self.assertEqual(vars(msg.header.stamp),before)
        self.assertEqual(owner._receipt.rows[-1]['stored_command_source_ns'],7123456789)
        self.assertEqual(owner._receipt.rows[-1]['stored_command_source_s'],
                         msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)

    def test_publisher_forwards_same_object_once_without_restamp(self):
        calls=[];owner=NS(get_clock=clock,_receipt=ScalarProbe('controller'),decision_delivery=None)
        publisher=NS(publish=lambda msg:calls.append(msg) or 14,topic_name='/replay_r508/drive')
        proxy=ObservedPublisher(publisher,owner,'controller');msg=message()
        before=copy.deepcopy(vars(msg.header.stamp))
        self.assertEqual(proxy.publish(msg),14)
        self.assertEqual(len(calls),1);self.assertIs(calls[0],msg)
        self.assertEqual(vars(msg.header.stamp),before)
        self.assertEqual(proxy.topic_name,publisher.topic_name)

    def test_runtime_hooks_preserve_results_and_call_counts(self):
        calls=[]
        row=dict(stage='current',generation=2,sequence=9,input={'reference_s':7.1},
                 output={'reference_s':7.1,'speed_mps':.4,'steering_rad':.1},received_wall_s=1.2)
        original=copy.deepcopy(row)
        candidate={'kind':'candidate','result':row};ack_result={'retained':True,'reason':'published_checked_route'}
        worker=NS(ready=row,revision=3,last_application=7.,
            collect=lambda:calls.append('collect') or [row],
            prepared=lambda **s:calls.append('prepared') or candidate)
        delivery=NS(worker=worker,pending_publication={'row':row},last_application=7.3,
            acknowledge_publication=lambda **s:calls.append('ack') or ack_result)
        owner=NS(get_clock=clock,_receipt=ScalarProbe('controller'),decision_delivery=delivery)
        hook_controller(owner)
        self.assertIs(worker.collect()[0],row)
        self.assertIs(worker.prepared(now_s=7.3,output_dt_s=.03),candidate)
        self.assertIs(delivery.acknowledge_publication(reference_s=7.1),ack_result)
        self.assertEqual(calls,['collect','prepared','ack']);self.assertEqual(row,original)
        self.assertEqual(owner._receipt.errors,0)

    def test_explicit_compact_preserved_and_no_silent_override(self):
        params=dict(use_sim_time=True,execution_environment='default',lidar_input_representation='native_simulation_v1',
                    motion_imu_frame_id='r506_reconstructed_sim_imu')
        profile={n:{'ros__parameters':copy.deepcopy(params)} for n in ('local_pursuit','lidar_safety')}
        profile['local_pursuit']['ros__parameters'].update(g1_trace_detail='compact_v1',g1_complete_plan_trace=False,race_control_mode='legacy')
        original=copy.deepcopy(profile)
        output=prepared_profile(profile,A/'future_result')
        self.assertEqual(output['local_pursuit']['ros__parameters']['g1_trace_detail'],'compact_v1')
        self.assertEqual(profile,original)
        for value in ('full',None):
            profile['local_pursuit']['ros__parameters']['g1_trace_detail']=value
            with self.assertRaisesRegex(ValueError,'explicit compact'):
                prepared_profile(profile,A/'future_result')

    def test_ast_entry_has_no_old_overrides_or_extra_drain(self):
        node=ast.parse((A/'stage/node_runner.py').read_text(encoding='utf8'))
        imports=[n.module for n in ast.walk(node) if isinstance(n,ast.ImportFrom)]
        self.assertNotIn('proof_candidate_bootstrap',imports)
        self.assertNotIn('trace_off_sink',imports)
        self.assertNotIn('motion_ingress_candidate',imports)
        classes=[n for n in ast.walk(node) if isinstance(n,ast.ClassDef) and n.name=='ObservedG1']
        self.assertEqual(len(classes),1)
        self.assertFalse(any(isinstance(n,ast.FunctionDef) and n.name=='create_follow_trace' for n in classes[0].body))
        tree=ast.parse((A/'stage/observation.py').read_text(encoding='utf8'))
        control=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='ControlReceipt').body[0]
        calls=[n for n in ast.walk(control) if isinstance(n,ast.Call)]
        self.assertEqual(sum(isinstance(n.func,ast.Attribute) and n.func.attr=='control' for n in calls),1)
        self.assertFalse(any(isinstance(n.func,ast.Attribute) and n.func.attr=='drain_pending' for n in calls))
        run=(A/'stage/run_once.py').read_text(encoding='utf8')
        self.assertNotIn('/home/q/it-arena-field-r500',run)
        self.assertIn('--execute-once',run)

    def test_zero_target_intervals_exclude_startup_and_censor_eof(self):
        from summarize import zero_intervals
        rows=[{'received_s':at,'speed':speed} for at,speed in
              [(7.7,0),(8.,.4),(8.1,0),(8.12,0),(8.14,.4),(27.5,0)]]
        result=zero_intervals(rows)
        self.assertEqual(result['first_positive'],8.)
        self.assertEqual(len(result['intervals']),2)
        self.assertAlmostEqual(result['total_zero_s'],.06)
        self.assertTrue(result['intervals'][-1]['right_censored'])
        self.assertEqual(result['intervals'][-1]['end_s'],27.52)


if __name__=='__main__':unittest.main()
