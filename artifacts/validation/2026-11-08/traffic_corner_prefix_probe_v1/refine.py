"""Keep first failed candidate; bound the derived prefix by actual wall support."""
from pathlib import Path

A=Path(__file__).resolve().parent
p=A/'candidate_local_follow.py';s=p.read_text(encoding='utf8')
assert not (A/'candidate_before_refinement.py').exists()
(A/'candidate_before_refinement.py').write_bytes(p.read_bytes())
(A/'test_candidate_before_refinement.py').write_bytes((A/'test_candidate.py').read_bytes())
start=s.index('def traffic_corner_prefix_agreement(');end=s.index('\ndef boundary_support(',start)
s=s[:start]+'''def traffic_corner_prefix(candidate, reference, details, config):
    """짧은 현재 벽의 지지를 넘기지 않는 코너 경로를 유효 이력과 대조한다.

    벽의 호길이와 오프셋 경로 길이는 다르다. 이 제한 후보는 더 짧은 실제 벽의
    전방 관측 길이까지만 경로를 남긴다. 최초/만료 이력의 확보와 점유 허가는 별도다.
    """
    if (not config.traffic_boundary_refresh_trial or not config.piecewise_wall_trial
            or not config.short_boundary_prefix_enabled or candidate is None or reference is None
            or details.get('wall_model') != 'piecewise_corner'
            or not 0. < details.get('boundary_observed_ahead_m', 0.) < config.boundary_min_ahead_m
            or anchor_error(details, config) > config.boundary_anchor_error_m):
        return None, None
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(candidate, axis=0), axis=1))]
    nearest = int(np.argmin(np.linalg.norm(candidate, axis=1)))
    budget = details['boundary_observed_ahead_m']
    end = int(np.searchsorted(arc, arc[nearest]+budget, side='right'))
    prefix = candidate[:end].copy()
    match = retained_boundary_agreement(prefix, reference, details, config)
    return (prefix, match) if match is not None else (None, None)

''' + s[end:]
s=s.replace('prefix_match = traffic_corner_prefix_agreement(preferred, carried, preferred_info, cfg)',
    'prefix, prefix_match = traffic_corner_prefix(preferred, carried, preferred_info, cfg)')
needle="candidate, info, candidate_wall, eligible = preferred, preferred_info, side, True\n                        detail['boundary_selection'] = 'current_corner_prefix_agreement'"
assert s.count(needle)==1
s=s.replace(needle,"candidate, info, candidate_wall, eligible = prefix, preferred_info, side, True\n                        detail['current_corner_observed_wall_ahead_m'] = preferred_info['boundary_observed_ahead_m']\n                        detail['boundary_selection'] = 'current_corner_prefix_agreement'")
p.write_text(s,encoding='utf8',newline='\n')
p=A/'test_candidate.py';s=p.read_text(encoding='utf8')
s=s.replace("scene.source_first_s,'return','laser_frame',0,1000+i", "scene.source_last_s,'return','laser_frame',0,1000+i")
s=s.replace('m.traffic_corner_prefix_agreement(', 'm.traffic_corner_prefix(')
s=s.replace(') is not None', ')[1] is not None').replace(') is None', ')[1] is None')
s=s.replace('np.testing.assert_allclose(c.path,p,atol=1e-10)',
    "np.testing.assert_allclose(c.path,p[:len(c.path)],atol=1e-10)\n    assert len(c.path)<len(p)\n    assert out[2]['observed_path_ahead_m']<=f['current']['ahead']+.08")
p.write_text(s,encoding='utf8',newline='\n')
p=A/'run.sh';s=p.read_text().replace('tests_v1','tests_v2').replace('replay_v1','replay_v2');p.write_text(s,newline='\n')
print('Saved first failures; corrected ray timestamp and bounded current derived prefix.')
