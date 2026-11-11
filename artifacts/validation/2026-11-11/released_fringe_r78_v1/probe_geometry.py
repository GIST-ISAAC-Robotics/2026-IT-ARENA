"""r76 저장 장면에 이미 사용 중인 저속 후방 띠 검사를 적용한다. 연결/새 주행 없음."""
import hashlib,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent
FROZEN=ROOT/'build/r76_frozen_diagnosis'
sys.path[:0]=[str(FROZEN/'src/arena_autonomy'),str(FROZEN/'src/arena_vehicle_interface'),str(FROZEN/'scripts')]
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.pass_release_trial import PassReleaseSelector,motion_region,region_blocked
from arena_autonomy.released_rear_sweep import bounded_released_rear_sweep
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.scene_motion import motion_scene
BASE=HERE.parent/'facility_noise_r76_v1'
rows=json.loads((BASE/'diagnosis_details.json').read_text(encoding='utf-8'))
picked={round(r['time_s'],6):r for r in rows}
trace=BASE/'two_actor_run1/run1/follow_decisions.jsonl';sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
digest=sha(trace);results=[]
for line in trace.open(encoding='utf-8'):
    e=json.loads(line);d=e['data']
    if e['kind']=='header':
        cfg=FollowConfig(**d['follow_config']);motion=MotionConfig(**d['motion_config']);owner=PassReleaseSelector(cfg)
    elif e['kind']=='call' and round(d['input']['reference_s'],6) in picked:
        saved=picked[round(d['input']['reference_s'],6)]
        baseline=next(v['output'] for v in saved['parts'] if v['stage']=='before_parallel')
        inputs,raw,history=restore_call(d['input'],motion)
        scene=motion_scene(raw,history,inputs['reference_s'],timing_verified=inputs['timing_verified'],
            epoch=inputs['epoch'],rear_half_angle_deg=inputs['rear_half_angle_deg'])
        target=min(baseline[0],owner.config.guard_speed_cap_mps);context={}
        region=motion_region(scene,inputs['speed_mps'],target,baseline[1],inputs['previous_steering_rad'],
            inputs['dt_s'],cfg,owner.lat,context=context)
        q=context['queries'][~context['free'] & ~context['masked']]
        proof=bounded_released_rear_sweep(scene,raw,q,context['padding'],cfg.length_m,cfg.width_m,
            inputs['speed_mps'],target,owner.config.small_excursion_m)
        blocked=region_blocked(region,owner.config,bounded_unknown_risk=proof['accepted'])
        results.append(dict(time_s=scene.t_ref_s,mode=saved['output'][2]['normal'].get('selector_mode'),
            target=target,original_target=baseline[0],measured=inputs['speed_mps'],region=region,proof=proof,
            blocked=blocked,eligible_released= saved['output'][2]['normal'].get('selector_mode')=='RELEASED'))
out=HERE/'geometry_probe.json'
if out.exists():raise SystemExit('preserve prior probe')
report=dict(scope='same frozen scene, existing rear proof at reduced target; NOT runtime connection or new trajectory',
    trace_sha256=digest,unchanged=sha(trace)==digest,results=results)
out.write_text(json.dumps(freeze(report),ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(freeze(report),ensure_ascii=False))
