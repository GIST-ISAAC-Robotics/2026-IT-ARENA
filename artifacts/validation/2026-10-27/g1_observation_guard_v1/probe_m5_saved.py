"""10/26 M5 원시 광선을 현 G1 후보로 비교. 고정 자세이며 출발/폐루프 증명이 아니다."""
from collections import Counter
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
for part in ('src/arena_autonomy','src/arena_vehicle_interface','scripts','tests'):
    sys.path.insert(0,str(ROOT/part))
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.follow_trace import freeze
from arena_autonomy.scene_motion import motion_scene
from test_local_follow import history, scan

INPUT = ROOT/'artifacts/validation/2026-10-26/m5_grid_follow_v1/run2/data/report.json'


def thaw_number(value):
    if isinstance(value,dict):
        return {'+inf':math.inf,'-inf':-math.inf,'nan':math.nan}[value['nonfinite']]
    return float(value)


def compare(saved):
    h=history(end=2.)
    results=[]
    for entry in saved['scenarios']:
        ranges=list(map(thaw_number,entry['ranges']))
        assert len(ranges)==500
        # 원 저장물은 마스크 전 거리이며 옛 진단은 scene_from_scan 안에서 후방을 가렸다.
        # 현 입력 계약은 /scan 단계에서 이미 +inf인 후방 마스크를 요구한다. 원본은 보존한다.
        masked_indices=[i for i in range(500) if abs(-math.pi+i*math.tau/500)>math.radians(150)+1e-12]
        ranges=[math.inf if i in masked_indices else v for i,v in enumerate(ranges)]
        for enabled in (False,True):
            cfg=FollowConfig(partial_observation_policy='bounded_local_v4',
                boundary_evidence_mode='corner_distance_candidate',occupancy_timing_mode='per_ray_candidate',
                corridor_fallback_enabled=True,parallel_hold_trial=enabled,
                pass_release_trial=enabled,guard_curve_trial=enabled)
            ctl=LocalFollow(cfg)
            calls=[]
            for frame in range(4):
                first=.5+frame*.1
                raw=scan(first,ranges)
                for repeat in range(5):
                    now=first+.0998+.002+repeat*.02
                    scene=motion_scene(raw,h,now,timing_verified=True)
                    speed,steering,meta=ctl.command(scene,h,0.,'left',0.,.02,raw_scan=raw)
                    calls.append(dict(t_s=now,speed_mps=speed,steering_rad=steering,reason=meta.get('reason'),
                        selector_mode=meta.get('selector_mode'),selector_reason=meta.get('selector_reason'),
                        partial_reason=(meta.get('partial_observation') or {}).get('reason')))
            old=next(m for m in entry['modes'] if m['mode']=='g1')
            results.append(dict(group=entry['group'],ego_grid_index=entry['ego_grid_index'],
                others=entry['other_grid_indices'],selector_enabled=enabled,config=asdict(cfg),
                input_adapter='기존 후방60도 마스크를 raw /scan 표현(+inf)에 적용',masked_indices=masked_indices,
                historical_strict_positive_calls=old['positive_request_calls'],historical_strict_reason=old['final_reason'],
                positive_calls=sum(c['speed_mps']>0 for c in calls),reason_counts=dict(Counter(c['reason'] for c in calls)),
                calls=calls,last_detail=meta))
    return results


if __name__=='__main__':
    destination=Path(sys.argv[1])
    if destination.exists():raise FileExistsError(destination)
    saved=json.loads(INPUT.read_text(encoding='utf-8'))
    assert saved['sensor']['xyz_m']==[.06,0.,.095] and saved['sensor']['rate_hz']==10.
    rows=compare(saved)
    files=list((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))+[Path(__file__),INPUT]
    report=dict(scope='동일 16개 정지 합성 광선, 4 fresh scan/20 calls; 제어 출력 미적용, ROS/독립 보호/물리 주행 미실행',
        source_sha256={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
        input=str(INPUT.relative_to(ROOT)),rows=rows,driving_verified=False)
    destination.write_text(json.dumps(freeze(report),ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps([{k:r[k] for k in ('group','ego_grid_index','selector_enabled','positive_calls','reason_counts')} for r in rows]))
