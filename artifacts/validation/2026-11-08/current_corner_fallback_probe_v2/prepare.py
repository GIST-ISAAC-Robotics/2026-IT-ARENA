from pathlib import Path

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
previous=A.parent/'current_corner_agreement_probe_v1'
source=ROOT/'src/arena_autonomy/arena_autonomy/local_follow.py'
assert not (A/'candidate_local_follow.py').exists()
old=source.read_bytes();s=old.decode('utf8')
needle="                if not eligible and preferred is not None and anchor_error(preferred_info, cfg) <= cfg.boundary_anchor_error_m:\n"
assert s.count(needle)==1
fallback='''                if (not eligible and cfg.traffic_boundary_refresh_trial and carried is not None
                        and sufficient(preferred, preferred_info)
                        and retained_boundary_agreement(preferred, carried, preferred_info, cfg) == 'shared_arc'):
                    # 기존 경계의 현재 관측 갱신을 먼저 시도한다. 실패한 경우에만 선호 벽의
                    # 충분한 현재 직각 경로를 같은 유효 이력과 대조한다. 점유 검사는 이후 유지한다.
                    candidate, info, candidate_wall, eligible = preferred, preferred_info, side, True
                    detail['boundary_selection'] = 'current_corner_alternative_agreement'
                    detail['current_corner_agreement'] = 'shared_arc'
'''
s=s.replace(needle,fallback+needle)
(A/'before_local_follow.py').write_bytes(old)
(A/'candidate_local_follow.py').write_bytes(s.encode('utf8'))
for name in ('loader.py','fixture.json','test_candidate.py','replay.py','run.sh'):
    s=(previous/name).read_text(encoding='utf8')
    if name=='run.sh':s=s.replace(previous.name,A.name)
    if name=='replay.py':
        s=s.replace("if n.get('retained_boundary_agreement'):reasons[n['retained_boundary_agreement']]+=1", "if n.get('current_corner_agreement'):reasons[n['current_corner_agreement']]+=1")
        s=s.replace("agreement=n.get('retained_boundary_agreement')", "agreement=n.get('current_corner_agreement')")
    (A/name).write_text(s,encoding='utf8',newline='\n')
print('Prepared fallback after retained-current rejection; production unchanged.')
