"""root/runner가 r560 종료 뒤 실행할 실제 launch_ros/rclpy 전달 검사 1개.

하위 IncludeLaunchDescription만 대역으로 제외한다. 실제 GroupAction(scoped=False)와
SetParameter를 거쳐 시작하는 parameter probe뿐이며 제어기/센서/시뮬레이터는 없다.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import sys


def child(output,expected_trace,mode,ros_args):
    import rclpy
    from rclpy.node import Node
    initialized=False
    node=None
    try:
        rclpy.init(args=ros_args)
        initialized=True
        node=Node('r559_trace_parameter_probe')
        trace=node.declare_parameter('g1_trace_path','').value
        g1=node.declare_parameter('g1_follow_enabled',False).value
        r0=node.declare_parameter('r0_plus_enabled',False).value
        assert type(trace) is str and trace==expected_trace
        assert type(g1) is bool and g1==(mode=='g1')
        assert type(r0) is bool and r0==(mode=='r0')
        assert 'g1_trace_path:=' not in ros_args
        with output.open('x',encoding='utf-8') as stream:
            json.dump(dict(passed=True,trace=trace,g1_follow_enabled=g1,r0_plus_enabled=r0,
                actual_ros_args=ros_args,trace_file_opened=False),stream,indent=2)
    finally:
        if node is not None:node.destroy_node()
        if initialized:rclpy.shutdown()


def parent(output_dir):
    from launch import LaunchDescription,LaunchService
    from launch.actions import OpaqueFunction,SetLaunchConfiguration
    from launch_ros.actions import Node
    output_dir.mkdir(parents=True,exist_ok=False)
    results=[]
    for mode,name in [('g1','g1_follow.launch.py'),('r0','r0_plus.launch.py')]:
        spec=importlib.util.spec_from_file_location('r559_trace_'+mode,Path(__file__).with_name(name))
        module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        includes=[]
        def omit_include(*args,**kwargs):
            includes.append(args)
            return OpaqueFunction(function=lambda context:[])
        module.IncludeLaunchDescription=omit_include
        for selection,trace in [('off',''),('on',"/tmp/r559 trace's path.jsonl")]:
            actions=module.generate_launch_description().entities
            assert len(includes)==1
            includes.clear()
            output=output_dir/(mode+'_'+selection+'.json')
            probe=Node(executable=sys.executable,arguments=[str(Path(__file__).resolve()),'--child',
                '--output',str(output.resolve()),'--expected-trace',trace,'--mode',mode],output='screen')
            service=LaunchService(argv=[])
            service.include_launch_description(LaunchDescription([
                SetLaunchConfiguration('g1_trace_path',trace),*actions,probe]))
            code=service.run()
            assert code==0 and output.is_file(),(mode,selection,code)
            row=json.loads(output.read_text(encoding='utf-8'))
            assert row['passed']
            results.append(dict(mode=mode,selection=selection,**row))
    (output_dir/'result.json').write_text(json.dumps(dict(passed=True,cases=results,
        scope='real launch_ros global argument transport and rclpy parser; no downstream includes'),indent=2),encoding='utf-8')
    print(json.dumps(dict(passed=True,cases=len(results))))


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--child',action='store_true')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--output-dir',type=Path)
    parser.add_argument('--expected-trace',default='')
    parser.add_argument('--mode',choices=['g1','r0'])
    args,ros_args=parser.parse_known_args()
    if args.child:
        assert args.output is not None and args.mode is not None
        child(args.output,args.expected_trace,args.mode,ros_args)
    else:
        assert args.output_dir is not None and not ros_args
        parent(args.output_dir)
