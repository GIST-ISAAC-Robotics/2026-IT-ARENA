"""실제 ROS 두 생성자·native 두 자식 선택. 센서/host/serial/spin/양수발행 없음."""
from pathlib import Path
import hashlib,json,sys,traceback
from unittest.mock import patch

ROOT=Path(__file__).resolve().parent


def main():
    import rclpy
    from rclpy.node import Node
    from arena_autonomy.local_pursuit import LocalPursuit
    from rclpy.publisher import Publisher
    from ackermann_msgs.msg import AckermannDriveStamped
    from arena_autonomy.g1_g2_node import G1G2Node
    from arena_autonomy.lidar_safety import LidarSafety
    from arena_autonomy.crop_edge_selection import ESTIMATED_SCOPE
    from arena_autonomy.runtime_node_backend import evaluator_header
    trace=ROOT/'trace.jsonl'
    args=['--ros-args','--params-file',str(ROOT/'runtime_profile.yaml'),
          '-p','g1_trace_path:='+str(trace),'-p','g1_current_corner_budget:=r396_v1',
          '-p','g1_crop_edge_component_trial:=true','-p','g1_observation_input_scope:='+ESTIMATED_SCOPE,
          '-p','race_control_mode:=b3_candidate']
    bindings=json.loads((ROOT/'bindings.json').read_bytes())
    for a,b in bindings['ros_remappings']['local_pursuit']:args+=['-r',a+':='+b]
    rclpy.init(args=args)
    node=guard=worker=vision=None;processes=[];published=[];error=None;selected=None;base_rejection=None
    original=Publisher.publish
    def observed(self,message):
        if isinstance(message,AckermannDriveStamped):
            published.append(dict(topic=self.topic_name,speed=message.drive.speed))
            assert message.drive.speed<=0,'positive publication prohibited in constructor check'
        return original(self,message)
    with patch.object(Publisher,'publish',observed):
        try:
            base=LocalPursuit.__new__(LocalPursuit)
            try:
                base.__init__()
                raise AssertionError('base planner accepted estimated scope')
            except ValueError as exc:
                base_rejection=str(exc)
                assert 'registered runtime-capable planner' in base_rejection
            finally:
                Node.destroy_node(base)
            node=G1G2Node();worker=node.decision_delivery.worker;vision=node.vision_pipeline.worker.process
            guard=LidarSafety()
            for owner in (node,guard):
                assert owner.observation_input_scope==ESTIMATED_SCOPE
                assert owner.motion.verified is False and owner.motion.timing_policy=='host_estimated_v1'
                assert not owner.motion.history.wheels.values and not owner.motion.history.gyro.values
                assert owner.get_parameter('use_sim_time').value is False
            assert node.follow.normal.partial_policy.observation_input_scope==ESTIMATED_SCOPE
            assert guard.partial_policy.observation_input_scope==ESTIMATED_SCOPE
            assert node.runtime_dual_selected and node.current_corner_budget=='r396_v1'
            assert not node.signal.started and node.count_publishers('/actuation/tx')==0
            assert node.publisher.topic_name=='/drive' and guard.publisher.topic_name=='/drive/safe'
            selected=evaluator_header(node,dict(numeric_execution='native_v1'))
            processes=[worker.plan.process,worker.current.process]
            assert all(p.is_alive() for p in processes)
        except BaseException as exc:
            error=repr(exc);(ROOT/'traceback.txt').write_text(traceback.format_exc())
        finally:
            if node is not None:node.destroy_node()
            if guard is not None:guard.destroy_node()
            rclpy.shutdown()
    children=[dict(pid=p.pid,exitcode=p.exitcode,alive=p.is_alive()) for p in processes]
    if vision is not None:children.append(dict(pid=vision.pid,exitcode=vision.exitcode,alive=vision.is_alive(),kind='vision'))
    cleanup=worker.cleanup if worker is not None else None
    report=dict(passed=error is None and base_rejection is not None and bool(cleanup) and not cleanup['forced'] and cleanup['exitcode']==0
        and len(children)==3 and all(not p['alive'] and p['exitcode']==0 for p in children)
        and not any(p['speed']>0 for p in published),error=error,base_rejection=base_rejection,selected=selected,children=children,
        cleanup=cleanup,published=published,spin_called=False,sensor_messages_supplied=False,devices_opened=False)
    (ROOT/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps(report,ensure_ascii=False));return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
