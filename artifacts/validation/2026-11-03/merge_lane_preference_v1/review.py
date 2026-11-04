"""기존 합류 목표와 신규 옵션 실제 사용 여부를 별개로 회수한다."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
path=ROOT/'artifacts/validation/2026-11-02/official_traffic_v1/goal_eval.py'
spec=importlib.util.spec_from_file_location('prior_goal',path)
goal=importlib.util.module_from_spec(spec); spec.loader.exec_module(goal)


def main():
    run=HERE/'run1'
    goal.evaluate(run,ROOT/'src/arena_gazebo/worlds/it_arena_official')
    result=json.loads((HERE/'run1_goal.json').read_bytes())
    report=json.loads((run/'report.json').read_bytes())
    hashes=json.loads((HERE/'registered.json').read_bytes())['source_sha256']
    checks=report.get('handoff_execution',{}).get('checks',{})
    source_ok=all(hashlib.sha256((ROOT/n).read_bytes()).hexdigest()==h for n,h in hashes.items())
    green=report['evaluation']['t_green']; end=report['case_window']['end_sim_s']
    applied=[]; zero=0.; last=None
    with (run/'follow_decisions.jsonl').open(encoding='utf-8') as f:
        hdr=json.loads(next(f))['data']['follow_config']
        for line in f:
            e=json.loads(line)
            if e['kind']!='call':continue
            i=e['data']['input']; o=e['data']['output']; t=i['reference_s']
            if not green<=t<=end:continue
            d=o[2].get('normal') or o[2]; info=d.get('current_lane_preference') or {}
            if info.get('applied'):
                applied.append(dict(t=t-green, seq=e['seq'], dt=i['dt_s'], command=o[:2], replaced=info.get('previous_output'),state=o[2].get('handoff_state')))
            if o[0]<=1e-9:zero+=i['dt_s']
            if last is None or o[0]>last['speed']: last=dict(t=t-green,speed=o[0])
    flags=all(checks.get(k) is True for k in ('side_departure','current_lane','lane_headroom'))
    summary=dict(passed=result['passed'] and source_ok and flags,
        original_passed=report['passed'],registered_goal_passed=result['passed'],
        established=result['established'], source_match=source_ok,source_files=len(hashes),
        options_match=flags,headroom_mps=hdr.get('current_lane_headroom_mps'),
        preference_calls=len(applied),preference_sample_s=sum(x['dt'] for x in applied),
        first_preference=applied[:1],last_preference=applied[-1:],zero_sample_s=zero,
        max_command=last,metrics=result['metrics'],contact=report['evaluation']['gap']['actor_contact_messages_with_ego'],
        overlap=report['evaluation']['gap']['sampled_overlap_samples'])
    (HERE/'review.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in summary.items() if k!='metrics'},ensure_ascii=False))
    return 0 if summary['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
