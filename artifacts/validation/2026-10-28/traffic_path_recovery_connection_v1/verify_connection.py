"""선택형 연결 출력이 검토한 미연결 v2와 같은지 전체 기록에서 대조한다."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import time
import numpy as np
BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
sys.path.insert(0,str(BASE.parent/'traffic_residual_v1'))
from arena_autonomy.follow_trace import equal_output,freeze,replay_call
from arena_autonomy.local_follow import FollowConfig,LocalFollow
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
from safety_shadow import ShadowSafety

def digest(path):
    with path.open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()

def main():
    trace=BASE.parent/'traffic_release_fringe_v1/run1_course/follow_decisions.jsonl'
    prototype=BASE.parent/'traffic_corridor_roles_v1/history_comparison_v2'
    proof=json.loads((prototype/'report.json').read_text())
    assert proof['comparison_valid'] and proof['input_unchanged'] and proof['source_unchanged']
    before=digest(trace)
    assert before==proof['input_sha256']
    changed=json.loads((prototype/'changed_commands.json').read_text())
    expected={round(r['t_s'],9):r for r in changed}
    assert len(expected)==len(changed)
    out=BASE/'connection_comparison'
    out.mkdir(exist_ok=False)
    sources={p.relative_to(ROOT).as_posix():digest(p)
             for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py')}
    counts,reasons=Counter(),Counter()
    mismatches,costs=[],[]
    gate=None
    with trace.open() as stream:
        header=json.loads(next(stream))['data']
        cfg=FollowConfig(**{**header['follow_config'],'traffic_path_recovery_trial':True})
        follow,motion=LocalFollow(cfg),MotionConfig(**header['motion_config'])
        for line in stream:
            event=json.loads(line)
            if event['kind']=='reset':
                follow.reset();counts['resets']+=1;continue
            if event['kind']!='call':continue
            data=event['data'];stamp=data['input']['reference_s']
            start=time.perf_counter()
            result=replay_call(follow,data['input'],motion)
            costs.append(1000*(time.perf_counter()-start))
            wanted=expected.get(round(stamp,9),{}).get('after',data['output'][:2])
            equal=equal_output(freeze(result[:2]),wanted)
            counts['matched']+=equal;counts['calls']+=1
            if not equal:mismatches.append(dict(t_s=stamp,expected=wanted,actual=result[:2]))
            if gate is None:
                gate=ShadowSafety(RecoveryPolicy(True,True,True),cfg,data['input']['rear_half_angle_deg'])
                assert gate.node.partial_policy is not follow.partial_policy
            safe,status=gate.step(data['input'],motion,result[:2])
            assert 0.<=safe[0]<=result[0]+1e-12
            if result[0]==0.:
                assert safe[0]==0.;counts['zero_preserved']+=1
            reasons[result[2]['reason']]+=1
            if 129.76<=stamp<=154.:
                counts['late_calls']+=1
                counts['late_positive']+=int(result[0]>0)
                counts['late_safe_positive']+=int(safe[0]>0)
    source_ok=all(digest(ROOT/p)==h for p,h in sources.items())
    input_ok=digest(trace)==before
    report=dict(scope='complete saved-input runtime/prototype command equivalence; no physics/ROS loop',
        passed=not mismatches and source_ok and input_ok,
        counts=dict(counts),reasons=dict(reasons),mismatch_count=len(mismatches),
        input_sha256=before,input_unchanged=input_ok,source_sha256=sources,source_unchanged=source_ok,
        prototype_report_sha256=digest(prototype/'report.json'),
        cost_ms=dict(median=float(np.median(costs)),p99=float(np.percentile(costs,99)),maximum=max(costs)))
    (out/'report.json').write_text(json.dumps(freeze(report),ensure_ascii=False,indent=2),encoding='utf-8')
    (out/'mismatches.json').write_text(json.dumps(freeze(mismatches),indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='source_sha256'}))
    return 0 if report['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
