"""r452 동결 평가기를 차량0 평가로 파생한다. ROS/빌드/시뮬을 실행하지 않는다."""
from pathlib import Path
import ast
import copy
import difflib
import hashlib
import json
import zipfile
import xml.etree.ElementTree as ET

A=Path(__file__).resolve().parent; ROOT=A.parents[3]
BASE=A.parent/'guard_crawl_drive_r452_v1'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,s):
    p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(s.encode('utf8'))
def dump(p,v):write(p,json.dumps(v,ensure_ascii=False,indent=2)+'\n')

def derive():
    frozen=json.loads((BASE/'prepared.json').read_bytes())
    assert sha(BASE/'sources.zip')==frozen['source_archive_sha256']
    with zipfile.ZipFile(BASE/'sources.zip') as z:
        src=z.read('scripts/validate_g1_traffic.py').decode('utf8').replace('\r\n','\n')
    assert hashlib.sha256((ROOT/'scripts/validate_g1_traffic.py').read_bytes()).hexdigest()==frozen['frozen_inputs']['scripts/validate_g1_traffic.py']
    write(A/'original/validate_g1_traffic.py',src)
    s=src; edits=[]
    def replace(old,new,label):
        nonlocal s
        assert s.count(old)==1,(label,s.count(old));s=s.replace(old,new);edits.append(label)
    def section(start,end,new,label):
        nonlocal s
        assert s.count(start)==1,label
        i=s.index(start);j=s.index(end,i);replace(s[i:j],new,label)
    replace('import g1_traffic_eval as ev\n','import g1_traffic_eval as ev\nimport solo_evaluation\n','solo evaluation import')
    replace("    specs = actor_specs(config, case)\n", "    specs = []  # r457: no traffic models, no hidden or remote actor.\n",'no specs')
    section('    route = check_route(config, centerline, case, s0,', '    sources = {rel: sha256(REPO/rel)',
        "    route = dict(kind='solo_full_course', lap_length_m=centerline.lap, actor_count=0)\n"
        "    starts, additional_routes = [], None\n"
        "    output.mkdir(parents=True, exist_ok=False)\n"
        "    actor_path = output/'facilities_only.sdf'\n"
        "    actor_path.write_bytes((Path(__file__).resolve().parent/'facilities_only.sdf').read_bytes())\n",'no actor generation/route')
    replace("    sources.update(facility_sources)\n", "    sources.update(facility_sources)\n    sources.update({p.relative_to(REPO).as_posix(): sha256(p) for p in Path(__file__).resolve().parent.glob('*.py')})\n",'record adapter sources')
    section('    def receive_actor(message):\n','    def receive_actuation(message):\n','', 'no actor receiver')
    replace('        node.create_subscription(TFMessage, f"/model/{actor_cfg[\'model_name\']}/pose", receive_actor, reliable),\n','', 'no actor subscription')
    replace("    actor_client = node.create_client(SetEntityPose, SET_POSE_SERVICE)\n",'', 'no actor client')
    replace("    extras = ExtraActors(specs[1:], starts[1:], centerline, actor_pose_at, output, began)\n",'', 'no extra actors')
    section('    def schedule_pose(t):\n','    try:\n        extra = [', '', 'no actor schedule completion')
    section('        from ament_index_python.packages import get_package_prefix\n',"        readers.append(json_reader(['gz', 'topic', '-e', '-t', f'/world/{WORLD_NAME}/dynamic_pose/info'",'', 'no actor bridge')
    section("        readers.append(json_reader(['gz', 'topic', '-e', '-t', CONTACT_TOPIC",'        if args.record_wheel_contacts:\n','', 'no actor contact reader')
    replace('        extras.connect(node, reliable, json_reader, capture_gate, reader_log, readers)\n','', 'no extra connect')
    replace("            complete_actor_future()\n            extras.complete()\n            if 'actuation' in state", "            if 'actuation' in state", 'no actor complete tick')
    replace('            if not required.issubset(state) or not actor or not extras.ready:\n','            if not required.issubset(state):\n', 'no actor startup wait')
    replace("                    raise TimeoutError(f'startup data missing: {sorted(required-set(state))}, actor={len(actor)}, extras_ready={extras.ready}')", "                    raise TimeoutError(f'startup data missing: {sorted(required-set(state))}')", 'solo startup diagnostic')
    section('            extras.tick(actor_client, SetEntityPose.Request, now, t_green)\n','            if now-last_record < .10:\n','', 'no actor pose commands')
    section('            latest_actor = actor[-1]\n',"            dynamics = state['dynamics']\n",'', 'no actor gap')
    replace("                'actor_x': latest_actor[1], 'actor_y': latest_actor[2], 'actor_stamp_s': latest_actor[0],\n                'sampled_gap_m': gap,\n",'', 'no fake actor trace')
    replace("            trace_stream.write(json.dumps(record, ensure_ascii=False)+'\\n')\n", "            trace_stream.write(json.dumps(record, ensure_ascii=False)+'\\n')\n"
        "            if t_green is not None and progress >= centerline.lap:\n"
        "                report['solo_lap_finish_sim_s'] = now\n"
        "                break\n",'one lap then original STOP')
    section('        # 일정이 끝난 뒤 actor는 마지막 명령 자세에 정지한다.',"        controller = 'local_pursuit'",'', 'no actor ACK drain')
    section('        actor_topic = f"/model/{actor_cfg[\'model_name\']}/pose"',"        report['drive_publishers']", "        report['actor_pose_publishers'] = {name: sorted(i.node_name for i in node.get_publishers_info_by_topic(f'/model/{name}/pose')) for name in ('eval_g1b_actor','eval_second_lead')}\n", 'audit actor publishers absent')
    section("        contact_info = subprocess.run(['gz', 'topic', '-i', '-t', CONTACT_TOPIC]",'        host_inputs = sorted(', '', 'no actor contact graph')
    section('        # 자차는 transport thread, 상대차는 ROS spin으로 수신한다.',"        report['abort'] = outcome['abort']",'', 'no actor tail drain')
    section("        probe = json.loads(CONTACT_PROBE_REPORT.read_text",'        node.destroy_node()\n',
        "        report['contact_evidence'] = dict(applicability='no_traffic_actor',\n"
        "            scope='Actor contact sensor removed with actor. Wall/old610 overlap and STOP checks remain; no physical contact-free claim from silence.')\n", 'actor contact explicitly inapplicable')
    replace("            report['evaluation'] = ev.evaluate_run(\n", "            report['evaluation'] = solo_evaluation.evaluate_run(\n",'solo evaluator')
    replace('                schedule_pose=schedule_pose, centerline=centerline, s0=s0,\n','                schedule_pose=None, centerline=centerline, s0=s0, obstacles=obstacles,\n','no fictitious actor schedule')
    replace('        ev.finalize_verdict(report, args.mode)\n','        solo_evaluation.finalize_verdict(report, args.mode)\n','solo finalizer')
    section('        if extras.specs:\n','        if native_audit:\n','', 'no actor mutual evaluation')
    replace("for name in ('ego', 'contact')", "for name in ('ego',)", 'only actual readers')
    # record_wheel_contacts is not selected in the fixed r452 command; preserve its optional reader shape.
    replace("        reader_audit['wheel_contact'] = dict(reader_audit['contact'])", "        reader_audit['wheel_contact'] = dict(reader_audit['ego'])",'optional wheel reader initialization')
    ast.parse(s);write(A/'solo_runner.py',s)
    write(A/'solo_runner.patch',''.join(difflib.unified_diff(src.splitlines(True),s.splitlines(True),fromfile='r452/scripts/validate_g1_traffic.py',tofile='r457/solo_runner.py')))
    with zipfile.ZipFile(BASE/'sources.zip') as z:
        evaluation=z.read('scripts/g1_traffic_eval.py').decode('utf8')
    node=next(n for n in ast.parse(evaluation).body if isinstance(n,ast.FunctionDef) and n.name=='finalize_verdict')
    original='\n'.join(evaluation.splitlines()[node.lineno-1:node.end_lineno])+'\n'
    final=original.replace("'contact_source_instrumented': report.get('contact_evidence', {}).get('validated_source') is True,", "'no_traffic_actor_in_world': evaluation.get('no_actor', {}).get('passed') is True,")
    final=final.replace("'actor_pose_only_evaluator': report.get('actor_pose_subscribers') == ['g1b_traffic_validator'],", "'no_actor_pose_publishers': report.get('actor_pose_publishers') == {'eval_g1b_actor': [], 'eval_second_lead': []},")
    assert final!=original
    write(A/'solo_finalize.py',final)
    write(A/'solo_finalize.patch',''.join(difflib.unified_diff(original.splitlines(True),final.splitlines(True),fromfile='r452/finalize_verdict',tofile='r457/solo_finalize.py')))
    # Preserve exact fixture elements; only two model subtrees are removed.
    original=ET.parse(BASE/'run1/actors_and_old610.sdf');tree=copy.deepcopy(original)
    removed=[]
    for model in list(tree.getroot().findall('model')):
        if model.get('name') in ('eval_g1b_actor','eval_second_lead'):
            removed.append(model.get('name'));tree.getroot().remove(model)
    assert removed==['eval_g1b_actor','eval_second_lead']
    names=[m.get('name') for m in tree.getroot().findall('model')]
    assert len(names)==3 and all(n.startswith('eval_r406_trial_old610_') for n in names)
    for name in names:
        assert ET.tostring(tree.getroot().find(f"model[@name='{name}']"))==ET.tostring(original.getroot().find(f"model[@name='{name}']"))
    write(A/'facilities_only.sdf',ET.tostring(tree.getroot(),encoding='unicode'))
    dump(A/'derivation.json',dict(baseline_archive_sha256=sha(BASE/'sources.zip'),original_runner_sha256=sha(ROOT/'scripts/validate_g1_traffic.py'),solo_runner_sha256=sha(A/'solo_runner.py'),edits=edits,removed_models=removed,retained_models=names,retained_models_element_exact=True,actor_count=0,common_modified=False))
    config=json.loads((BASE/'config.json').read_bytes());config['scope']='r457 latest CAD solo full lap, actor0; r452 controller/sensor/facility/physics unchanged'
    config['official_multi_actor_trial']=False
    config['launch']['entry_g1']=(A/'course.launch.py').relative_to(ROOT).as_posix()
    old_case=config['cases'][0]['name'];config['cases'][0]['name']='solo_course_lap';config['cases'][0]['behavior_kind']='solo'
    config['cases'][0]['schedule']=[];config['cases'][0]['expected']='actor0, one complete lap within80sim seconds, original ownedSTOP'
    config['acceptance']['solo_course_lap']=config['acceptance'].pop(old_case)
    config['matrix']={'modes':['g1g2'],'cases':['solo_course_lap'],'seeds':[0]}
    dump(A/'config.json',config)
    for rel in ('launch_adapter.py','course.launch.py','facility_geometry.py','header_comparison.py','audit_runtime.py','expected_header.json','stop_owned.py','launch/demo.launch.py','launch/local_pursuit.launch.py','launch/simulation.launch.py','execution_v1/observe.py','execution_v1/stop_once.sh','execution_v1/check_owned_processes.py'):
        path=BASE/rel
        if rel=='expected_header.json':
            (A/rel).write_bytes(path.read_bytes())
            continue
        text=path.read_text(encoding='utf8').replace('guard_crawl_drive_r452_v1','solo_full_lap_r457_v1').replace('r452_owned_stop','r457_owned_stop')
        write(A/rel,text)
    return dict(derived=True,edits=len(edits),actor_count=0,simulator_started=False)

if __name__=='__main__':print(json.dumps(derive()))
