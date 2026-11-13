"""r457 실제 current 전부와 직접 장애물/구조 가림 반례. 센서/발행 없음."""
import copy
import hashlib
import json
import math
import sys
import time
from pathlib import Path
import numpy as np

A=Path(__file__).resolve().parent
sys.path[:0]=[str(A/'stage/src/arena_autonomy'), str(A/'stage/src/arena_vehicle_interface')]
from arena_autonomy.race_runtime_policy.evaluator import Evaluator
from arena_autonomy.runtime_dual_state import current_request
from arena_autonomy.follow_trace import freeze, thaw, restore_call
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.known_structure import current_mask_shadow, parse_sectors
from arena_autonomy import pass_release_trial as ps

O=A/'verification_v2'
O.mkdir(exist_ok=False)
data=json.loads((A/'inputs.json').read_bytes())
header=data['header']
sha=lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
source_before={p.relative_to(A).as_posix():sha(p) for p in (A/'stage').rglob('*') if p.is_file()}
source_before['inputs.json']=sha(A/'inputs.json')
e=Evaluator('current', header)
rows=[]; samples={}; tests=[]

def request(row):
    d=row['data']
    return current_request(d['output'],d['compute']['input'],d['current_rebase']['input'],
        retained=d['compute']['route_check']['retained'],crop_current_selection=True)

def record(name, passed, **detail):
    tests.append(dict(name=name, passed=bool(passed), **detail))

try:
    rows=json.loads((A/'verification_v1/rows.json').read_bytes())
    for row in data['calls']:
        samples[row['seq']]=e(request(row))
    modes={}
    for line in (A.parent/'solo_full_lap_r457_v1/run1/follow_decisions.jsonl').open(encoding='utf-8'):
        row=json.loads(line)
        if row['kind']=='call':modes[row['seq']]=row['data']['output'][2].get('normal',{}).get('selector_mode')
    record('all_non_solo_outputs_exact',all(r['exact'] for r in rows if modes.get(r['sequence'])!='SOLO'))
    record('all_old_positive_stay_positive',all(not (r['old_speed']>0 and r['speed']<=0) for r in rows))
    record('same_speed_keeps_command_path_time',all(r['geometry_same'] for r in rows if r['speed']==r['old_speed']))
    record('guard1360_exact', next(r for r in rows if r['sequence']==1360)['exact'])
    extra=A.parent/'estimated_guard_transfer_r462_v1/bundle/inputs'
    guard_e=Evaluator('current',json.loads((extra/'r452_header.json').read_bytes()))
    try:
        for line in (extra/'currents.jsonl').open(encoding='utf-8'):
            item=json.loads(line)
            if item['dataset']!='r452': continue
            request_extra=current_request(item['plan_output'],item['plan_input'],item['current'],
                retained=item['retained'],crop_current_selection=True)
            value=guard_e(request_extra)
            expected=item['expected']
            # r459 이전 진단과 비교하므로 명령/경로/거절만 확인한다.
            record('recorded_traffic_'+item['id'],all(value.get(k)==expected.get(k) for k in
                   ('speed_mps','steering_rad','reason','path_points','path_source_s')),
                   speed=value['speed_mps'],reason=value['reason'])
    finally: guard_e.close()
    for seq in (1590,1986):
        row=next(r for r in rows if r['sequence']==seq)
        record(f'original_target_restored_{seq}',abs(row['speed']-.75)<1e-9 and row['region_same'] and row['geometry_same'])
        record(f'no_speed_retry_{seq}',not any('retry' in k for k in samples[seq]))
    original=request(data['calls'][-1])
    raw=original['current']['scan']['measurements']
    # 실제 마지막 스캔의 구조 마스크430..440와 양옆429/441을 사용한다.
    for name,index,value in [('near_flank',429,.14),('infinite_declared_flank',424,math.inf),
                             ('nan_declared_flank',446,math.nan),('visible_side_inside_band_angle',435,.10)]:
        r=copy.deepcopy(original)
        r['current']['scan']['measurements']['ranges_m'][index]=freeze(value)
        answer=e(r)
        proof=(answer.get('current_route_proof') or {}).get('current_structure_risk') or {}
        record(name,answer['speed_mps']==0 and proof.get('accepted') is not True,
               speed=answer['speed_mps'],reason=answer['reason'],proof=proof)
    # 같은 저장 입력에서 얻은 장면을 쓰는 순수 기하 반례. 원 입력은 바꾸지 않는다.
    info,scan,history=restore_call(original['current'],e.motion)
    scene=motion_scene(scan,history,info['reference_s'],timing_verified=info['timing_verified'],
        epoch=info['epoch'],rear_half_angle_deg=info['rear_half_angle_deg'])
    cfg=e.follow.config; context={}
    region=ps.motion_region(scene,0,.75,-.4578436017036438,-.4578436017036438,
                            original['current']['dt_s'],cfg,ps.lateral_config(cfg),context=context)
    q=context['queries'][context['masked']]
    sectors=parse_sectors(cfg.known_structure_sectors_json)
    proof=current_mask_shadow(scene,scan,q,context['padding'],sectors)
    record('bounded_shadow_original',proof['accepted'])
    record('no_declared_sector',not current_mask_shadow(scene,scan,q,context['padding'],())['accepted'])
    origin=np.asarray(scene.rays[435].ray_origin_xy_m)
    direction=np.asarray(scene.rays[435].direction_xy)
    far=origin[None]+direction[None]*.50
    proof=current_mask_shadow(scene,scan,far,context['padding'],sectors)
    record('wide_shadow_at_distance',not proof['accepted'],reason=proof['reason'])
    # 전면 가림 조건을 순수 함수의 성공으로 덮어쓰지 않는다.
    original_region=ps.motion_region
    def front(*args,**kw):
        value=original_region(*args,**kw)
        return {**value,'masked_front_samples':1}
    ps.motion_region=front
    try:
        answer=e(original)
        record('front_mask_retained',answer['speed_mps']==0 and
               'current_structure_risk' not in (answer.get('current_route_proof') or {}),reason=answer['reason'])
    finally: ps.motion_region=original_region
    def unknown(*args,**kw):
        value=original_region(*args,**kw)
        return {**value,'unobserved':value['unobserved']+1}
    ps.motion_region=unknown
    try:
        answer=e(original)
        record('general_unknown_not_mask_excused','current_structure_risk' not in (answer.get('current_route_proof') or {}),
               speed=answer['speed_mps'],reason=answer['reason'])
    finally: ps.motion_region=original_region
finally: e.close()
record('sources_and_inputs_unchanged',all(sha(A/p)==h for p,h in source_before.items()))
summary=dict(calls=len(rows),exact=sum(r['exact'] for r in rows),
    zero_to_positive=sum(r['old_speed']==0 and r['speed']>0 for r in rows),
    positive_to_zero=sum(r['old_speed']>0 and r['speed']==0 for r in rows),
    increased_positive=sum(0<r['old_speed']<r['speed'] for r in rows),
    decreased_positive=sum(0<r['speed']<r['old_speed'] for r in rows),
    checks=len(tests),passed=sum(r['passed'] for r in tests),
    median_ms=float(np.median([r['ms'] for r in rows])),p95_ms=float(np.percentile([r['ms'] for r in rows],95)),
    max_ms=max(r['ms'] for r in rows),failures=[r for r in tests if not r['passed']])
for name,value in [('rows',rows),('tests',tests),('summary',summary),('source_manifest',source_before)]:
    (O/f'{name}.json').write_text(json.dumps(value,indent=2),encoding='utf-8')
print(json.dumps(summary))
assert all(r['passed'] for r in tests)
