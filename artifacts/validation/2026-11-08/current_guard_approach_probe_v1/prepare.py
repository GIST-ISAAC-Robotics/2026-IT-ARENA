"""Detached current observed approach in GUARD; no production writes."""
from pathlib import Path

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
P=A.parent/'traffic_corner_prefix_probe_v1'
assert not (A/'candidate_selector.py').exists()
original=(ROOT/'src/arena_autonomy/arena_autonomy/pass_release_trial.py').read_bytes()
s=original.decode('utf8')
needle='\ndef band_check(scene, frame, sign, cfg, sel, longitudinal=None):'
helper='''
def observed_guard_approach(frame, context, guarded_sides, rejected_sides):
    """옆차 쪽으로 추가 접근하는 모든 이동 표본의 현재 관측을 요구한다.

    과거 횡위치 초과의 대안 근거이며 미관측/장착 마스크는 통과시키지 않는다.
    context의 free는 원시 반환과 기존 거리/시간 여유를 적용한 depth_support다.
    """
    evidence = {'accepted': False, 'unknown_filled': False, 'sides': {}}
    if not rejected_sides or not context or frame is None:
        return evidence
    queries = context['queries']
    if not len(queries):
        return evidence
    _, lateral = frame.project(queries)
    for name in rejected_sides:
        sign = 1. if name == 'left' else -1.
        beyond = sign*lateral > guarded_sides[name]['current_m']+1e-9
        count = int(np.count_nonzero(beyond))
        free = int(np.count_nonzero(context['free'][beyond]))
        evidence['sides'][name] = {'samples': count, 'currently_observed': free}
        if count == 0 or free != count:
            return evidence
    evidence['accepted'] = True
    return evidence

'''
assert s.count(needle)==1;s=s.replace(needle,'\n'+helper+needle)
needle="        info.update(envelope_distance_m=distance, guarded_sides={})\n"
assert s.count(needle)==1;s=s.replace(needle,needle+'        rejected_sides = []\n')
needle="                return reject(f'toward_{name}')\n        excursion ="
replacement="""                rejected_sides.append(name)
        if rejected_sides and not (
                self.follow_config.traffic_recovery_trial
                and b_detail.get('boundary_from_memory') is False
                and b_detail.get('path_source_s') == scene.source_first_s):
            return reject(f'toward_{rejected_sides[0]}')
        excursion ="""
assert s.count(needle)==1;s=s.replace(needle,replacement)
needle="        small_tail = bool(region['masked_left'] or region['masked_right'])\n        info['accepted'] = True\n"
replacement="""        if rejected_sides:
            # 통과/해제 이력을 조작하지 않는다. 기존 전체 이동 검사에 더하여
            # 위협 방향의 추가 접근 표본 모두에 현재 자유 관측을 요구한다.
            if not context:
                _, queries = motion_queries(scene, speed, target, b_steer, previous, dt, self.follow_config, self.lat)
                pad = mx.ray_padding(scene, self.lat, self.follow_config.object_motion_bound_mps)
                context = {'queries': queries, 'free': depth_support(scene, queries, pad)}
            approach = observed_guard_approach(view['frame'], context, info['guarded_sides'], rejected_sides)
            info['current_approach'] = approach
            if not approach['accepted']:
                return reject(f'toward_{rejected_sides[0]}')
        small_tail = bool(region['masked_left'] or region['masked_right'])
        info['accepted'] = True
"""
assert s.count(needle)==1;s=s.replace(needle,replacement)
needle="        risk_detail = ({'selector_same_path_gap_risk_accepted': same['accepted']}"
assert s.count(needle)==1;s=s.replace(needle,"        if rejected_sides:\n            reason = 'guard_curve_current_approach_observed'\n"+needle)
(A/'before_selector.py').write_bytes(original)
(A/'candidate_selector.py').write_text(s,encoding='utf8',newline='\n')
replay=(P/'replay.py').read_text(encoding='utf8')
replay=replay.replace('from loader import load', 'from loader import load')
replay=replay.replace('m=load();LocalFollow._baseline_command=m.LocalFollow._baseline_command', 'load()')
replay=replay.replace("agreement=n.get('current_corner_prefix_agreement'),boundary=n.get('boundary_selection')",
    "agreement=n.get('current_corner_prefix_agreement'),boundary=n.get('boundary_selection'),selector=n.get('selector_reason'),guard=n.get('selector_guard_candidate')")
(A/'replay.py').write_text(replay,encoding='utf8',newline='\n')
s=(P/'run.sh').read_text(encoding='utf8').replace(P.name,A.name).replace('_v2','_v1')
(A/'run.sh').write_text(s,encoding='utf8',newline='\n')
print('Prepared prefix + strict current-approach candidate; production unchanged.')
