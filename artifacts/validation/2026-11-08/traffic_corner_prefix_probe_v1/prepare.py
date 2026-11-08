"""Detached current observed corner prefix. No changes to production files."""
from pathlib import Path
import json

A=Path(__file__).resolve().parent; ROOT=A.parents[3]
OLD=A.parent/'current_corner_fallback_probe_v2'
assert not (A/'candidate_local_follow.py').exists()
original=(ROOT/'src/arena_autonomy/arena_autonomy/local_follow.py').read_bytes()
s=original.decode('utf8')
needle='\ndef boundary_support(points, path, side, config):'
helper='''
def traffic_corner_prefix_agreement(candidate, reference, details, config):
    """이미 확보한 코너와 일치하는 짧은 현재 관측만 갱신한다.

    최초 경로/만료 이력을 새로 승인하는 함수가 아니다. 호출자가 유효 이력을
    확인하며, 채택 경로는 현재 관측의 끝에서 끝난다. 점유·제동 예산은 이후 적용한다.
    """
    if (not config.traffic_boundary_refresh_trial or not config.piecewise_wall_trial
            or not config.short_boundary_prefix_enabled or candidate is None or reference is None
            or details.get('wall_model') != 'piecewise_corner'
            or not 0. < details.get('boundary_observed_ahead_m', 0.) < config.boundary_min_ahead_m
            or anchor_error(details, config) > config.boundary_anchor_error_m):
        return None
    return retained_boundary_agreement(candidate, reference, details, config)

'''
assert s.count(needle)==1;s=s.replace(needle,'\n'+helper+needle)
needle='                if (not eligible and cfg.boundary_bridge\n'
branch='''                if not eligible and carried is not None:
                    prefix_match = traffic_corner_prefix_agreement(preferred, carried, preferred_info, cfg)
                    if prefix_match is not None:
                        # 미관측 연장 없이 현재 보이는 짧은 경로로 교체한다. 긴 벽의 초기
                        # 확보 기준은 유지하고, 현재 길이의 제동/조향/원시 점유를 다시 검사한다.
                        candidate, info, candidate_wall, eligible = preferred, preferred_info, side, True
                        detail['boundary_selection'] = 'current_corner_prefix_agreement'
                        detail['current_corner_prefix_agreement'] = prefix_match
'''
assert s.count(needle)==1;s=s.replace(needle,branch+needle)
(A/'before_local_follow.py').write_bytes(original)
(A/'candidate_local_follow.py').write_bytes(s.encode('utf8'))
audit=A.parent/'corner_slowing_diagnosis_v1'
rows=json.loads((audit/'path_comparison.json').read_bytes())
row=min(rows,key=lambda r:abs(r['t']-35.62))
scan=min(json.loads((audit/'captures.json').read_bytes()),key=lambda r:abs(r['t']-35.62))
trace=A.parent/'corner_slowing_actor_v1/run1/follow_decisions.jsonl'
with trace.open() as f:
    header=json.loads(next(f))['data']
    call=next(json.loads(line)['data'] for line in f if '"kind":"call"' in line and abs(json.loads(line)['data']['input']['reference_s']-35.62)<1e-8)
fixture=dict(config=header['follow_config'],motion_config=header['motion_config'],input=call['input'],
    current=next(c for c in row['candidates'] if c['side']=='left' and c['kind']=='all'),
    reference=scan['carried_path'],prior=scan['prior'],source_trace=trace.relative_to(ROOT).as_posix())
(A/'fixture.json').write_text(json.dumps(fixture,indent=2)+'\n',encoding='utf8')
(A/'loader.py').write_bytes((OLD/'loader.py').read_bytes())
s=(OLD/'replay.py').read_text(encoding='utf8').replace('retained_corner_agreement_r46_v1','corner_slowing_actor_v1')
s=s.replace("'current_corner_agreement'", "'current_corner_prefix_agreement'")
(A/'replay.py').write_text(s,encoding='utf8',newline='\n')
s=(OLD/'run.sh').read_text(encoding='utf8').replace(OLD.name,A.name)
(A/'run.sh').write_text(s,encoding='utf8',newline='\n')
print('Prepared detached prefix candidate and actual 35.62s fixture.')
