"""급커브 정적 첫 반환에 고정 오차/시각을 주입하는 제한 진단. 실물 분포가 아니다."""
from dataclasses import replace
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import zipfile

import numpy as np
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.local_scene import scene_from_scan
from test_local_follow import cast_scene, history
from test_corner_raycast import elbow
from profile_follow_trace import SNAPSHOT_MODULES

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]


def main():
    output=BASE/'report.json'
    if output.exists(): raise FileExistsError(output)
    sources=[f'src/{p}/{p}/{m}.py' for p,mods in SNAPSHOT_MODULES.items() for m in mods]
    sources+=['tests/test_local_follow.py','tests/test_corner_raycast.py',str(Path(__file__).relative_to(ROOT))]
    hashes={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sources}
    with zipfile.ZipFile(BASE/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for p in sources:archive.write(ROOT/p,p)
    rows=[]
    # 2형상 × 좌우 × 4오차 × 2나이 = 32조건. 경계가 불충분하면 정지가 허용된다.
    # 중앙 완전 봉쇄에서 비영 출력은 실패다. 모든 입력점과 취득 나이는 보존한다.
    for kind in ('open','center_block'):
        for sign in (1,-1):
            world=(np.asarray(elbow(.30,kind))*[1,sign]).tolist()
            base=cast_scene(.5,world)
            distances=np.array([np.linalg.norm(np.asarray(r.hit_xy_m)-r.ray_origin_xy_m)
                if r.hit_xy_m else math.inf for r in base.rays])
            for error in ('none','bias_positive','bias_negative','uniform_seed0'):
                finite=np.isfinite(distances)
                adjusted=distances.copy()
                if error=='bias_positive':adjusted[finite]+=.03
                elif error=='bias_negative':adjusted[finite]-=.03
                elif error=='uniform_seed0':adjusted[finite]+=np.random.default_rng(0).uniform(-.03,.03,finite.sum())
                for age in (.10,.25):
                    side='left' if sign==1 else 'right'
                    ctl=LocalFollow(FollowConfig(boundary_evidence_mode='corner_anchor_candidate',occupancy_timing_mode='per_ray_candidate'))
                    outputs=[]
                    for t in (.4,.5,.6,.7):
                        scene=scene_from_scan(adjusted.tolist(),angle_min=-math.pi,angle_increment=math.tau/500,
                            range_min=.05,range_max=12.,acquired_at_s=[t-age]*500,t_ref_s=t)
                        scene=replace(scene,rays=tuple(replace(r,status='masked',hit_xy_m=None)
                            if abs(math.atan2(r.direction_xy[1],r.direction_xy[0]))>math.radians(150) else r for r in scene.rays))
                        outputs.append(ctl.command(scene,history(),0.,side))
                    rows.append(dict(kind=kind,side=side,error=error,age_s=age,outputs=outputs,
                        passed=kind=='open' or all(r[0]==0 for r in outputs)))
    result=dict(scope='static_first_return_range_and_age_injection_not_closed_loop',rows=rows,
        passed=all(row['passed'] for row in rows),source_sha256=hashes,
        source_unchanged=all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items()),
        driving_authorized=False)
    output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(passed=result['passed'],source_unchanged=result['source_unchanged'],
        rows=[dict(kind=r['kind'],side=r['side'],error=r['error'],age_s=r['age_s'],speed=r['outputs'][-1][0],
                   reason=r['outputs'][-1][2]['reason'],passed=r['passed']) for r in rows])))
    return 0 if result['passed'] and result['source_unchanged'] else 1


if __name__=='__main__':raise SystemExit(main())
