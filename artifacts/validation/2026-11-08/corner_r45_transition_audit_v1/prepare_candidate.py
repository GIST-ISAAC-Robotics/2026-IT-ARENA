"""기존 선택형 벽 갱신 분기에만 호길이 일치 비교를 추가한 분석 사본."""
from pathlib import Path
import hashlib
import json

A=Path(__file__).resolve().parent;ROOT=A.parents[3];OUT=A.parent/'retained_corner_agreement_probe_v1'
OUT.mkdir(exist_ok=False)
src=ROOT/'src/arena_autonomy/arena_autonomy/local_follow.py';raw=src.read_bytes()
(OUT/'before_local_follow.py').write_bytes(raw)
s=raw.decode('utf8')
needle='def boundary_support(points, path, side, config):'
helper='''def retained_boundary_agreement(candidate, reference, details, config):
    """현재 긴 벽의 유지 비교. 직각 경로는 공통 호길이 구간도 대조한다.

    x 창만으로는 같은 x를 가진 코너 출구의 서로 다른 길이를 함께 비교할 수 있다.
    선택형 교통·직각 경로에서만 기존 호길이 비교를 재사용한다. 허용 오차나
    관측 길이를 늘리지 않으며, 실제 점유/통과 검사는 호출자가 별도로 한다.
    """
    if path_agrees(candidate, reference, config.boundary_agreement_m):
        return 'shared_x'
    if (config.traffic_boundary_refresh_trial and config.piecewise_wall_trial
            and details.get('wall_model') == 'piecewise_corner'
            and path_agrees_arc(candidate, reference, config.boundary_agreement_m)):
        return 'shared_arc'
    return None


'''
assert s.count(needle)==1;s=s.replace(needle,helper+needle)
old='''                    if sufficient(opposite, other) and path_agrees(opposite,carried,cfg.boundary_agreement_m):
                        candidate,info,candidate_wall,eligible = opposite,other,other_side,True
                        detail['boundary_selection']='current_retained_wall_refresh' '''.rstrip()
new='''                    retained_match = (retained_boundary_agreement(opposite, carried, other, cfg)
                                      if sufficient(opposite, other) else None)
                    if retained_match is not None:
                        candidate,info,candidate_wall,eligible = opposite,other,other_side,True
                        detail['boundary_selection']='current_retained_wall_refresh'
                        detail['retained_boundary_agreement']=retained_match'''
assert old in s;s=s.replace(old,new)
(OUT/'candidate_local_follow.py').write_text(s,encoding='utf8',newline='\n')
(OUT/'design.json').write_text(json.dumps(dict(before_sha256=hashlib.sha256(raw).hexdigest(),
    candidate_sha256=hashlib.sha256(s.encode()).hexdigest(),source=src.relative_to(ROOT).as_posix()),indent=2))
print('detached current retained corner agreement prepared; production unchanged')
