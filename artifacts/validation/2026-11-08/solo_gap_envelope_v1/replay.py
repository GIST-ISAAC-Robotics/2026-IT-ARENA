"""r19 제어 출력 재현과 대조 후보. 기본 로직·실제 주행은 바꾸지 않는다."""
import hashlib
import json
from pathlib import Path

from arena_autonomy.follow_trace import replay_call, comparable_decision, equal_output, freeze
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.lidar_motion import MotionConfig
from candidate import Candidate, PATH, recovery_policy

HERE=Path(__file__).resolve().parent
path=HERE.parent/'solo_corner_approach_connected_v1/run1/follow_decisions.jsonl'
events=[json.loads(line) for line in path.read_bytes().splitlines()]
h=events[0]['data'];cfg=FollowConfig(**h['follow_config']);mc=MotionConfig(**h['motion_config'])
base=LocalFollow(cfg);candidate=LocalFollow(cfg)
policy=recovery_policy() if cfg.traffic_recovery_trial else Candidate
if cfg.known_structure_policy!='off':
    from arena_autonomy.known_structure import structure_policy
    policy=structure_policy(policy,cfg.known_structure_sectors_json)
candidate.partial_policy=policy(allow_narrow_forward=True,
    surface_translation_enabled=cfg.partial_observation_policy in ('bounded_local_v3','bounded_local_v4'),
    lateral_metric_enabled=cfg.partial_observation_policy=='bounded_local_v4',
    oblique_metric_enabled=cfg.oblique_gap_trial)
n=0;changed=[]
for e in events[1:]:
    if e['kind']=='reset':base.reset();candidate.reset()
    if e['kind']!='call':continue
    d=e['data'];n+=1
    a=freeze(replay_call(base,d['input'],mc));b=freeze(replay_call(candidate,d['input'],mc));old=d['output']
    assert equal_output(comparable_decision(a),comparable_decision(old)),('baseline mismatch',d['input']['reference_s'])
    if not equal_output(a[:2],b[:2]):
        changed.append(dict(t=d['input']['reference_s'],before=a[:2],after=b[:2],
                            reason=b[2].get('reason'),partial=b[2].get('partial_observation')))
r=dict(scope='saved motion only; future scan/pose not simulated; no runtime code edited',calls=n,
       baseline_full_match=True,changes=len(changed),first_changes=changed[:3],last_changes=changed[-2:],
       source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),policy_sha256=hashlib.sha256(PATH.read_bytes()).hexdigest())
(HERE/'replay.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
(HERE/'changed.json').write_text(json.dumps(changed,ensure_ascii=False,indent=1)+'\n',encoding='utf8')
print(json.dumps({k:v for k,v in r.items() if k not in ('first_changes','last_changes')},ensure_ascii=False))
