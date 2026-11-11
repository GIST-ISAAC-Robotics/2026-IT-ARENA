"""r75 최초 latch의 과거 점이 기존 구조 기억에 들어 있는지 확인한다. 코드 변경 없음."""
import hashlib,json
from pathlib import Path
import numpy as np
from arena_autonomy.local_follow import LocalFollow,FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.follow_trace import replay_call,restore_call,freeze,equal_output,comparable_decision
from arena_autonomy.known_structure import classify

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
TRACE=HERE.parent/'longitudinal_surface_r75_v1/solo_run1/follow_decisions.jsonl'
OUT=HERE/'memory_probe.json'
if OUT.exists():raise SystemExit('preserve prior probe')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
paths=[TRACE,ROOT/'src/arena_autonomy/arena_autonomy/known_structure.py',ROOT/'src/arena_autonomy/arena_autonomy/partial_surface_alignment.py']
hashes={str(p):sha(p) for p in paths};mismatches=[];calls=0
for line in TRACE.open(encoding='utf-8'):
    e=json.loads(line);d=e['data']
    if e['kind']=='header':
        cfg=FollowConfig(**d['follow_config']);motion=MotionConfig(**d['motion_config']);ctl=LocalFollow(cfg)
    elif e['kind']=='reset':ctl.reset()
    elif e['kind']=='call':
        actual=freeze(replay_call(ctl,d['input'],motion));calls+=1
        if not equal_output(comparable_decision(actual),comparable_decision(d['output'])):mismatches.append(e['seq'])
        if actual[2].get('reason')=='follow_known_obstacle_latched':
            p=ctl.partial_policy
            arrays=json.loads((HERE.parent/'longitudinal_surface_r75_v1/latch_context_arrays.json').read_text(encoding='utf-8'))
            corrected=np.asarray(arrays['anchor_corrected_points'])
            offset=np.asarray(actual[2]['partial_observation']['surface_correspondence']['translation_at_reference_m'])
            old=corrected-offset
            hidden,when=p.structure_hidden
            _,scan,history=restore_call(d['input'],motion)
            _,_,rows=classify(scan,p.structure_sectors)
            shadows=p._shadow(scan,rows,history,d['input']['reference_s'],scan.acquired_at_s[0])
            inside=p._inside(old,[s for s in shadows if s['explained']])
            distances=np.min(np.linalg.norm(old[:,None,:]-hidden[None,:,:],axis=2),axis=1)
            result=dict(scope='saved sensors and existing memory only; no policy change or new trajectory',
                calls=calls,seq=e['seq'],reference_s=d['input']['reference_s'],baseline_mismatches=mismatches,
                policy_class=type(p).__name__,hidden_count=len(hidden),hidden_reference_s=when,
                points=[dict(point_xy=a.tolist(),hidden_nearest_m=float(b),inside_explained_structure=bool(c),
                    behind_body=bool(a[0]<-cfg.length_m/2-.025)) for a,b,c in zip(old,distances,inside)],
                structure_result=actual[2]['partial_observation'].get('known_structure'),
                input_sha256=hashes,unchanged=all(sha(Path(k))==v for k,v in hashes.items()))
            OUT.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
            print(json.dumps({k:v for k,v in result.items() if k not in ('input_sha256','structure_result')},ensure_ascii=False))
            break
else:raise SystemExit('first latch not found')
