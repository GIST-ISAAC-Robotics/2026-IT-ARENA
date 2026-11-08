"""등록 설정의 누락된 case별 별칭만 사후 보충한다. 원 실행/판정/설정은 수정하지 않는다."""
from pathlib import Path
import copy,hashlib,json
import validate_g1_traffic as launch_eval
from g1_traffic_eval import evaluate_run,finalize_verdict
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
run=HERE/'right_side_start/run1';read=lambda p:json.loads(p.read_bytes())
rows=lambda n:[json.loads(l) for l in (run/n).read_bytes().splitlines() if l.strip()]
original=read(run/'report.json');cfg=read(HERE/'new_track_course.json');case=cfg['cases'][0]
assert case['behavior_kind']=='slow_lead' and case['name'] not in cfg['acceptance']
cfg['acceptance'][case['name']]=copy.deepcopy(cfg['acceptance']['slow_lead'])
world=ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06'
line,scene=launch_eval.load_centerline(world);s0=launch_eval.actor_start_s(cfg,scene,0,case)
green=original['case_window']['green_sim_s']
def schedule(t):return launch_eval.actor_pose_at(line,s0,case['schedule'],max(0.,t-green),case['actor_lateral_offset_m'])[:2]
r=copy.deepcopy(original)
r['evaluation']=evaluate_run(config=cfg,case=case,mode=r['mode'],t_green=green,
    trace=read(run/'trace.json'),ego=rows('ego_poses.jsonl'),actor=rows('actor_poses.jsonl'),
    contacts=rows('actor_contacts.jsonl'),commands=read(run/'actor_commands.json'),stop_start=r['stop_request_sim_s'],
    schedule_pose=schedule,centerline=line,s0=s0,road=launch_eval.road_polygon(run/'runtime_world.sdf'),
    runtime_world=(run/'runtime_world.sdf').read_text(),contact_evidence=r['contact_evidence'])
finalize_verdict(r,r['mode'])
r['review_only']=dict(reason='case별 acceptance 별칭 누락. 등록된 slow_lead 동일값 사용, 원 report/설정/종료값 불변.',original_report_sha256=hashlib.sha256((run/'report.json').read_bytes()).hexdigest(),config_sha256=hashlib.sha256((HERE/'new_track_course.json').read_bytes()).hexdigest())
(HERE/'review_report.json').write_text(json.dumps(r,ensure_ascii=False,indent=1)+'\n',encoding='utf-8')
source=(HERE/'goal_eval.py').read_text()
old="report = json.loads((run/'report.json').read_bytes())"
assert source.count(old)==1
source=source.replace(old,'report = REVIEWED_REPORT')
old="output = run.parent/f'{run.name}_goal.json'"
assert source.count(old)==1
source=source.replace(old,"output = REVIEW_OUTPUT")
ns=dict(__name__='review_goal',__file__=str(HERE/'goal_eval.py'),REVIEWED_REPORT=r,REVIEW_OUTPUT=HERE/'review_goal.json')
exec(compile(source,str(HERE/'goal_eval.py'), 'exec'),ns)
ns['evaluate'](run,world)
print(json.dumps(dict(raw_review_passed=r['passed'],run_valid=r['run_valid'],harness_valid=r['harness_valid'],registered_run_pass_not_rewritten=True)))
assert hashlib.sha256((run/'report.json').read_bytes()).hexdigest()==r['review_only']['original_report_sha256']
