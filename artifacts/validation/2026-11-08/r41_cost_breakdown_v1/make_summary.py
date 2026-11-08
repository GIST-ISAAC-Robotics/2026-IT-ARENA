"""timers_result/profile_result/counterfactual에서 압축 요약표를 만든다(읽기 전용 입력)."""
import json
from pathlib import Path
HERE = Path(__file__).resolve().parent
t = json.loads((HERE/'timers_result.json').read_text(encoding='utf-8'))
p = json.loads((HERE/'profile_result.json').read_text(encoding='utf-8'))
cf = json.loads((HERE/'counterfactual_estimates.json').read_text(encoding='utf-8'))
TREE = [  # (깊이, 이름) 포함 시간 트리. 같은 깊이의 형제만 더할 수 있다.
    (0, 'restore_call'), (0, 'motion_scene'), (1, 'scene_from_scan'), (1, 'relative_poses@motion_scene'),
    (1, 'audit_front_observation@motion_scene'), (0, 'BypassHandoffFollow.command'),
    (1, 'ReadyInitialReference.update'), (1, 'LocalFollow.command'), (2, 'PassReleaseSelector._select'),
    (3, 'threat_view'), (4, 'ObservationGrid.__init__'), (4, 'cell_coordinates'), (4, 'ReturnGrid.__init__'),
    (4, 'side_return_masks'), (4, 'side_threats'), (3, 'motion_region@_select'), (3, 'PassReleaseSelector._guard_curve'),
    (2, 'LocalFollow._observed_command'), (3, 'LocalFollow._path_command'), (4, 'CorridorObserver.evaluate'),
    (4, 'LocalFollow._baseline_command'), (3, 'StructureAware.evaluate'), (2, 'ParallelLaneTrial.apply')]
EDGE = {'relative_poses@motion_scene': 'motion_scene > relative_poses',
        'audit_front_observation@motion_scene': 'motion_scene > audit_front_observation',
        'motion_region@_select': 'PassReleaseSelector._select > motion_region'}
rows = []
for depth, name in TREE:
    if name in EDGE:
        mean = t['edges_sum_s'][EDGE[name]]*1e3/t['calls']
        rows.append(dict(depth=depth, name=name, inclusive_mean_ms=round(mean, 3)))
        continue
    f = t['functions'][name]
    rows.append(dict(depth=depth, name=name, calls=f['total_calls'], decisions_with_call=f['decisions_with_call'],
                     inclusive_mean_ms=round(f['inclusive_per_decision_ms']['mean'], 3),
                     inclusive_p95_ms=round(f['inclusive_per_decision_ms']['p95'], 3),
                     self_mean_ms=round(f['self_per_decision_ms']['mean'], 3)))
summary = dict(
    trace=t['trace'], calls=t['calls'], resets=p['resets'],
    parity=dict(reference_vs_r42=t['parity']['reference_vs_r42_identical'] and p['parity']['reference_vs_r42_commands_identical'],
                cprofile_pass_vs_reference=p['parity']['profiled_vs_reference_identical'],
                timer_pass_vs_reference=t['parity']['timed_vs_reference_identical'],
                decision_stream_sha256=p['parity']['reference_stream_sha256']),
    uninstrumented_total_ms={k: {s: round(p['reference_costs']['all']['total_ms'][s], 2) for s in ('p50', 'p95', 'over_20ms')}
                             for k in ['pass_B_run']} | {'pass_C_run': {s: round(t['reference_costs']['total_ms'][s], 2)
                             for s in ('p50', 'p95', 'over_20ms')}},
    timer_pass_stages_ms={k: {s: round(v[s], 2) for s in ('mean', 'p50', 'p95')} for k, v in t['timer_pass_stages'].items()},
    cprofile_pass_stages_ms={k: round(p[k]['p50'], 2) for k in ('profiled_total_ms', 'profiled_restore_ms',
                                                                 'profiled_projection_ms', 'profiled_decision_ms')},
    inclusive_tree_mean_per_decision=rows,
    per_ray_side_measure_ms={k: round(t['side_measure'][k]['mean'], 3) for k in
                             ('rear_masked_loop_ms', 'blind_spec_loop_ms', 'parsed_prototype_loop_ms')}
                            | dict(prototype_identical_all=t['side_measure']['prototype_identical_all'],
                                   scans=t['side_measure']['scans_measured']),
    compact_duplicate=dict(checked=t['side_measure']['compact_duplicate_checked'],
                           identical_all=t['side_measure']['compact_duplicate_identical_all']),
    counterfactual_subtraction_estimates=cf,
    note='포함 시간은 같은 깊이 형제끼리만 더한다. 카운터팩추얼은 측정값 차감 추정이며 구현 측정이 아니다.')
(HERE/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=1), encoding='utf-8')
for r in rows:
    print('  '*r['depth']+r['name'], r['inclusive_mean_ms'], r.get('inclusive_p95_ms', ''), r.get('calls', ''))
