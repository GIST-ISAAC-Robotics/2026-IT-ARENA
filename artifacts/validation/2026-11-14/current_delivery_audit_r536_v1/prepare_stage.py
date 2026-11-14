"""검증된 r523 재생기의 입출력/회수 절차를 복사하고 시험 개입만 제거한다."""
import ast
import hashlib
import json
from pathlib import Path

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
BASE = A.parent/'jetson_drain_budget_r523_v1/bundle'
OUT = A/'stage'


def replace_once(text, old, new):
    assert text.count(old)==1, old[:100]
    return text.replace(old,new,1)


node = (BASE/'node_runner.py').read_text(encoding='utf8')
node = replace_once(node, 'from bisect import bisect_right\n\nfrom proof_candidate_bootstrap import CANDIDATE_SHA',
    'from observation import ControlReceipt, GuardReceipt, ScalarProbe, ObservedPublisher, hook_controller\nfrom contracts import assert_installed_package')
node = replace_once(node, "    args = p.parse_args()", "    p.add_argument('--installed-root', type=Path, required=True)\n    args = p.parse_args()")
node = replace_once(node, "    assert str(Path(arena_autonomy.__file__).resolve()).startswith('/home/q/it-arena-field-r500-20261114-a/install/')",
    "    assert_installed_package(arena_autonomy.__file__, args.installed_root)")
start, end = node.index('    completed, controls, summary = '), node.index('    try:\n        if args.role ==')
node = node[:start]+"    summary = {}\n    probe = ScalarProbe(args.role)\n"+node[end:]
start, end = node.index('            class ObservedG1('), node.index('            owner = ObservedG1()')
node = node[:start]+"            class ObservedG1(ControlReceipt,G1G2Node):pass\n\n"+node[end:]
start, end = node.index('            collect = worker.collect'), node.index("            summary['configuration']")
node = node[:start]+node[end:]
node = replace_once(node,'class ObservedGuard(ControlReceipt,LidarSafety):pass',
                    'class ObservedGuard(GuardReceipt,ControlReceipt,LidarSafety):pass')
start, end = node.index("        if hasattr(owner,'motion'):\n            owner._pending_motion_subscriptions"), node.index('        from replay_transport import audit_current')
node = node[:start]+'''        owner._receipt = probe
        if args.role in ('controller','guard'):
            owner.publisher = ObservedPublisher(owner.publisher, owner, args.role)
            subscriptions = owner.motion._ready_motion_subscriptions
            assert len(subscriptions)==2
            assert {s.topic_name for s in subscriptions}=={PREFIX+'/drive_motor/encoder',PREFIX+'/imu/data'}
        if args.role=='controller':
            assert owner.follow_trace.detail=='compact_v1' and not owner.complete_plan_trace
            assert owner.get_parameter('g1_trace_detail').value=='compact_v1'
            hook_controller(owner)
'''+node[end:]
node = replace_once(node,'summary.update(route_proof_candidate_sha256=CANDIDATE_SHA,role=args.role,',
                    'summary.update(role=args.role,')
node = replace_once(node,"            summary['trace_off']=dict(owner.follow_trace.snapshot(),creation=owner.trace_creation_receipt)",
    "            summary['trace_detail']=owner.follow_trace.detail")
node = replace_once(node,"            summary['trace_off'] = dict(owner.follow_trace.snapshot(),creation=owner.trace_creation_receipt)",
    "            summary['trace_detail'] = owner.follow_trace.detail")
node = replace_once(node,"        if controls:\n            write(out/(args.role+'_controls.json'),controls)\n        if completed:\n            write(out / 'calculation_receipts.json', completed)",
    "        write(out/(args.role+'_scalar_summary.json'), probe.write(out))")
node = node.replace('설치된 노드만 호출하는 r508 계측 경계.', '최신 명시 설치 노드의 r536 scalar 계측 경계.')
(OUT/'node_runner.py').write_text(node,encoding='utf8')

runner = (BASE/'run_replay.py').read_text(encoding='utf8')
runner = replace_once(runner, 'from collections import Counter', 'import argparse\nfrom collections import Counter')
runner = replace_once(runner,"INSTALLED = Path('/home/q/it-arena-field-r500-20261114-a')", 'from contracts import INPUT_SHA, prepared_profile, source_snapshot')
start, end = runner.index('\ndef source_snapshot():'), runner.index('\ndef main():')
runner = runner[:start]+runner[end:]
runner = replace_once(runner,'def main():\n', '''def main():
    parser=argparse.ArgumentParser(description='One bounded saved-input simulation replay; no physical host or output')
    parser.add_argument('--installed-root',type=Path,required=True)
    parser.add_argument('--source-manifest-sha256',required=True)
    parser.add_argument('--profile',type=Path,required=True)
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--dds-library',type=Path,required=True)
    parser.add_argument('--execute-once',action='store_true')
    args=parser.parse_args()
    if not args.execute_once:parser.error('explicit --execute-once required')
    if not sys.platform.startswith('linux'):raise RuntimeError('ROS execution is not enabled on this platform')
    args.installed_root=args.installed_root.resolve(strict=True)
    out=args.output.resolve()
''')
runner = replace_once(runner,"(A / 'manifest.json')", "(A / 'stage_manifest.json')")
runner = replace_once(runner,'    before = source_snapshot()',
    '    before = source_snapshot(args.installed_root, args.source_manifest_sha256)')
runner = replace_once(runner,"    data = json.loads(gzip.decompress((A / 'reconstructed_inputs.json.gz').read_bytes()))\n    out = A / 'result'",
    "    assert SHA(args.input)==INPUT_SHA,'frozen original replay input required'\n    data = json.loads(gzip.decompress(args.input.read_bytes()))")
runner = replace_once(runner,"    profiles = json.loads((A / 'profile.json').read_bytes())",
    "    profiles = prepared_profile(json.loads(args.profile.read_bytes()),out)\n    write(out/'invocation.json',dict(installed_root=str(args.installed_root),profile_sha256=SHA(args.profile),\n        input_sha256=SHA(args.input),source_manifest_sha256=args.source_manifest_sha256))")
runner = replace_once(runner,"    profiles['local_pursuit']['ros__parameters']['g1_trace_path'] = str(out / 'controller_trace.jsonl')", '')
runner = replace_once(runner,"    probe=Path('/home/q/it_arena_a3_20260928/build/libarena_dds_probe_v1.so')", '    probe=args.dds_library')
runner = replace_once(runner,"                '--output', str(out)], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)",
    "                '--output', str(out), '--installed-root', str(args.installed_root)], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)")
runner = replace_once(runner,"        for role in children:\n            assert (out/('dds_active_'+role+'.json')).is_file(),role", """        for role in children:
            assert (out/('dds_active_'+role+'.json')).is_file(),role
        write(out/'start_receipt.json',dict(main_pid=os.getpid(),
            node_pids={role:child.pid for role,child in children.items()},
            controller_child_pids=json.loads((out/'controller_ready.json').read_bytes())['child_pids'],
            started_once=True,physical_output=False))""")
runner = replace_once(runner,'        after=source_snapshot()',
    '        after=source_snapshot(args.installed_root,args.source_manifest_sha256)')
runner = replace_once(runner,"str(A/'analyze.py')", "str(A/'summarize.py')")
runner = runner.replace('r500 고정 설치본, simulation-clock ROS 재구성 20초 단1회.',
                        '명시한 최신 설치본, simulation-clock ROS 재구성 20초 단1회.')
(OUT/'run_once.py').write_text(runner,encoding='utf8')

profile=json.loads((BASE/'profile.json').read_bytes())
profile['local_pursuit']['ros__parameters']['g1_trace_detail']='compact_v1'
(OUT/'profile.compact.json').write_text(json.dumps(profile,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
for filename in ('replay_transport.py','audit_start_signal_layout.py'):
    (OUT/filename).write_bytes((ROOT/'scripts'/filename).read_bytes())
for path in OUT.glob('*.py'):
    ast.parse(path.read_text(encoding='utf8'))
manifest=dict(scope='future one-run helper; PC fixture validation only',
    derived_from=str(BASE.relative_to(ROOT)).replace('\\','/'),
    files={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(OUT.iterdir())
           if p.is_file() and p.suffix in ('.py','.json') and p.name!='stage_manifest.json'})
(OUT/'stage_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf8')
print(json.dumps(dict(stage_files=len(manifest['files']),generated_only=True,ros_executed=False)))
