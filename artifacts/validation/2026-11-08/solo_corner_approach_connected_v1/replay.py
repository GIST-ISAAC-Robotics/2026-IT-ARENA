import hashlib,json
from dataclasses import replace
from pathlib import Path
from arena_autonomy.follow_trace import replay_call,equal_output,comparable_decision,freeze
from arena_autonomy.local_follow import FollowConfig,LocalFollow
from arena_autonomy.lidar_motion import MotionConfig
HERE=Path(__file__).resolve().parent;run=HERE.parent/'solo_contact_diagnosis_v1/run1';source=run/'follow_decisions.jsonl';shadow=json.loads((HERE.parent/'solo_corner_approach_v1/contact.json').read_text());assert hashlib.sha256(source.read_bytes()).hexdigest()==shadow['source_sha256']
expected={r['t']:r['candidate_mps'] for r in shadow['changes']};n=0;fixed=0
with source.open() as f:
    h=json.loads(next(f))['data'];cfg=FollowConfig(**h['follow_config']);mc=MotionConfig(**h['motion_config']);off=LocalFollow(cfg);on=LocalFollow(replace(cfg,solo_corner_approach_trial=True))
    for line in f:
        event=json.loads(line)
        if event['kind']=='reset':off.reset();on.reset()
        if event['kind']!='call':continue
        d=event['data'];ref=d['input']['reference_s'];old=d['output'];baseline=freeze(replay_call(off,d['input'],mc));now=freeze(replay_call(on,d['input'],mc));n+=1
        assert equal_output(comparable_decision(baseline),comparable_decision(old)),('off',ref)
        assert equal_output(now[:2],[expected.get(ref,old[0]),old[1]]),('on',ref,old[:2],now[:2],expected.get(ref))
        fixed+=now[0]<old[0]-1e-6
r=dict(calls=n,changed=fixed,off_full_match=True,on_numeric_matches_shadow=True,source_sha256=shadow['source_sha256']);print(json.dumps(r));(HERE/'replay_result.json').write_text(json.dumps(r,indent=2)+'\n')
