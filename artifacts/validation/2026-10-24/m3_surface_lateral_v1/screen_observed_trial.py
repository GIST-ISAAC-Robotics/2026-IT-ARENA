"""조건부 M3 시험 후보의 저장 입력 대조. 현재 경로 재생이며 제어 연결 없음."""
import json
from pathlib import Path
import sys
import time
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import screen_lateral as screen
from arena_autonomy.observed_lateral_trial import ObservedLateralTrial
from arena_autonomy.lateral_intent import LateralIntentConfig


result = {}
for name in ('e3_onroad_g1', 'e3_onroad_r0plus', 'solo_g1_20261016'):
    motion, follow, events = screen.load_inputs(screen.RECORDS[name])
    rows = []
    trial = ObservedLateralTrial(LateralIntentConfig(enabled=True, surface_policy='two_wheel_grass'))
    with screen.TruthBlocked():
        for event in events:
            if event['kind'] == 'reset':
                trial.reset()
                continue
            path = event['data']['output'][2].get('path_points')
            data, scan, history = screen.restore_call(screen.thaw(event['data']['input']), motion)
            scene = screen.motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                                        epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
            start = time.perf_counter()
            out = trial.step(scene, path, data['speed_mps'])
            row = {'t': scene.t_ref_s, 'mode': out['mode'], 'selected': out['selected'], 'checks': out['checks'],
                   'runtime_authorized': out['runtime_authorized'], 'ms': 1e3*(time.perf_counter()-start)}
            if out['selected'] not in (None, 'baseline'):
                pts = np.asarray(scene.occupied_endpoints_xy_m).reshape(-1, 2)
                margin = screen.per_ray_padding(scene, .025+follow.measurement_padding_m, follow.object_motion_bound_mps)
                age = data['reference_s']-scene.source_first_s
                row['downstream'] = screen.existing_checks(out['path'], pts, margin, data['speed_mps'],
                    data['previous_steering_rad'], age, follow, data['dt_s'],
                    path_age=max(age, event['data']['output'][2].get('boundary_age_s') or 0.))
            rows.append(row)
    modes, why = {}, {}
    for row in rows:
        modes[row['mode']] = modes.get(row['mode'], 0)+1
        for name_, check in row['checks'].items():
            for reason in check['reasons']:
                key = name_+':'+reason
                why[key] = why.get(key, 0)+1
    summary = {'calls': len(rows), 'modes': modes, 'candidate_reasons': why,
               'ms_p50_p95_max': np.percentile([r['ms'] for r in rows], [50, 95, 100]).tolist(),
               'downstream_positive': sum(r.get('downstream', {}).get('target_mps', 0) > 0 and
                                           not r.get('downstream', {}).get('gap_hold', True) for r in rows)}
    result[name] = {'summary': summary, 'rows': rows}
    print(name, json.dumps(summary), flush=True)
Path(sys.argv[1]).write_text(json.dumps(result, indent=1), encoding='utf-8')
