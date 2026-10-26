"""저장 센서 입력에서만 동일 시각 측방 대조. 운영 선택·실행·정답 입력 없음."""
import json
from pathlib import Path
import sys
import time
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import screen_lateral as screen
import arena_autonomy.lateral_intent as li
from arena_autonomy.lateral_sweep_review import review_sweep


results = {}
for name in ('e3_onroad_g1', 'e3_onroad_r0plus'):
    motion, _, events = screen.load_inputs(screen.RECORDS[name])
    cfg = li.LateralIntentConfig(enabled=True, surface_policy='two_wheel_grass')
    rows = []
    last_source = None
    with screen.TruthBlocked():
        for e in events:
            if e['kind'] != 'call':
                continue
            path = e['data']['output'][2].get('path_points')
            if path is None:
                continue
            data, scan, history = screen.restore_call(screen.thaw(e['data']['input']), motion)
            scene = screen.motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                                        epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
            if scene.source_first_s == last_source:
                continue
            last_source = scene.source_first_s
            frame = li.PathFrame(path)
            ctx = li.build_context(scene, frame, data['speed_mps'], cfg)
            if not any(t['present'] for t in ctx['threats'].values()):
                continue
            hold = li.offset_candidate(frame, ctx['state'], ctx['state']['d0'], cfg, 'hold')
            # 같은 입력에서 만든 현재 후보의 목표이며 검증된 선택이라는 뜻은 아니다.
            choice = li.LateralIntentLimiter(cfg).step(scene, path, data['speed_mps'])
            row = {'t': scene.t_ref_s, 'speed': data['speed_mps'], 'd0': ctx['state']['d0'], 'checks': {}}
            for key, check in choice['checks'].items():
                candidate = li.offset_candidate(frame, ctx['state'], check['d_target_m'], cfg, key)
                if not candidate['valid'] or not hold['valid']:
                    row['checks'][key] = {'geometry_invalid': True}
                    continue
                started = time.perf_counter()
                result = review_sweep(scene, ctx['grid'], candidate, hold, ctx['hidden_seeds_xy'],
                                      max(0., data['speed_mps']), cfg)
                row['checks'][key] = {**result, 'ms': 1e3*(time.perf_counter()-started),
                                      'old_reasons': check['reasons'], 'target': check['d_target_m']}
            rows.append(row)
    summary = {'fresh_threat_frames': len(rows)}
    for key in ('hold', 'center', 'yield'):
        valid = [r['checks'][key] for r in rows if key in r['checks'] and not r['checks'][key].get('geometry_invalid')]
        summary[key] = {'valid_geometry': len(valid),
                        'unknown_extra': sum(r['extra_unknown'] > 0 for r in valid),
                        'hidden_extra': sum(r['extra_hidden'] > 0 for r in valid),
                        'both_extra_zero': sum(r['extra_unknown'] == 0 and r['extra_hidden'] == 0 for r in valid),
                        'full_candidate_hidden': sum(r['candidate_hidden'] > 0 for r in valid),
                        'ms_p50_p95_max': np.percentile([r['ms'] for r in valid], [50, 95, 100]).tolist() if valid else []}
    results[name] = {'summary': summary, 'rows': rows}
    print(name, json.dumps(summary), flush=True)
Path(sys.argv[1]).write_text(json.dumps(results, indent=1), encoding='utf-8')
