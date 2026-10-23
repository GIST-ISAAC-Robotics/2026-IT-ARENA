"""거리 후보: 코너 첫 반환/거리 오차/취득 나이. 출력 기대와 가용성은 분리한다."""
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import zipfile

import numpy as np
from arena_autonomy.local_follow import FollowConfig,LocalFollow
from test_local_follow import cast_scene,history
from test_corner_raycast import elbow
from profile_follow_trace import SNAPSHOT_MODULES

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]


def main():
    output=BASE/'report.json'
    if output.exists():raise FileExistsError(output)
    paths=[f'src/{p}/{p}/{m}.py' for p,modules in SNAPSHOT_MODULES.items() for m in modules]
    paths+=['scripts/profile_follow_trace.py','tests/test_local_follow.py','tests/test_corner_raycast.py',
            str(Path(__file__).relative_to(ROOT))]
    hashes={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in paths}
    with zipfile.ZipFile(BASE/'source.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in paths:z.write(ROOT/p,p)
    rows=[]
    # 5 형상 × 2 진입 × 2 방향 × 4 오차 × 2 나이 = 160조건. closed-loop 아님.
    for kind in ('open','center_block','side_block','missing_inner','short_fake_inner'):
        for entry in (.05,.15):
            for sign in (1,-1):
                base=cast_scene(.5,(np.asarray(elbow(entry,kind))*[1,sign]).tolist())
                for error in ('none','bias_positive','bias_negative','uniform_seed0'):
                    rng=np.random.default_rng(0)
                    rays=[]
                    for r in base.rays:
                        if abs(math.atan2(r.direction_xy[1],r.direction_xy[0]))>math.radians(150):
                            rays.append(replace(r,status='masked',hit_xy_m=None));continue
                        if r.hit_xy_m is None:rays.append(r);continue
                        d=np.asarray(r.hit_xy_m)-r.ray_origin_xy_m
                        noise=0. if error=='none' else .03 if error=='bias_positive' else -.03 if error=='bias_negative' else rng.uniform(-.03,.03)
                        hit=np.asarray(r.ray_origin_xy_m)+d*(1+noise/np.linalg.norm(d))
                        rays.append(replace(r,hit_xy_m=tuple(hit)))
                    for age in (.10,.25):
                        ctl=LocalFollow(FollowConfig(boundary_evidence_mode='corner_distance_candidate',occupancy_timing_mode='per_ray_candidate'))
                        outputs=[]
                        for now in (.4,.5,.6,.7):
                            observed=replace(base,t_ref_s=now,source_first_s=now-age,source_last_s=now-age,
                                rays=tuple(replace(r,acquired_at_s=now-age) for r in rays))
                            outputs.append(ctl.command(observed,history(),0.,'left' if sign==1 else 'right'))
                        rows.append(dict(kind=kind,entry_m=entry,side='left' if sign==1 else 'right',error=error,age_s=age,
                            outputs=outputs,passed=(kind=='open' or all(r[0]==0 for r in outputs)),
                            availability_positive=outputs[-1][0]>0))
    report=dict(scope='static_first_return_noise_age_contract_not_closed_loop',rows=rows,
        passed=all(r['passed'] for r in rows),source_sha256=hashes,
        source_unchanged=all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items()),
        driving_authorized=False,closed_loop_pass=False)
    output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(passed=report['passed'],source_unchanged=report['source_unchanged'],
        counts={k:dict(rows=sum(r['kind']==k for r in rows),positive=sum(r['kind']==k and r['availability_positive'] for r in rows),
                       failures=sum(r['kind']==k and not r['passed'] for r in rows)) for k in ('open','center_block','side_block','missing_inner','short_fake_inner')})))
    return 0 if report['passed'] and report['source_unchanged'] else 1


if __name__=='__main__':raise SystemExit(main())
