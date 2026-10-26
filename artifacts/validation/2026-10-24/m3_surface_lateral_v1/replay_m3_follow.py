"""M3 순수 제어기를 저장 센서에 연결한 개루프 대조. 새 궤적/실차/폐루프 시험 아님."""
from dataclasses import replace
import json
from pathlib import Path
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import screen_lateral as screen
from arena_autonomy.m3_observed_follow import M3ObservedFollow
from arena_autonomy.local_follow import LocalFollow
from arena_autonomy.follow_trace import freeze


result = {'scope': 'open_loop_sensor_replay_not_driving', 'runtime_node_connected': False, 'records': {}}
names = sys.argv[2:] or ['e3_onroad_g1', 'e3_onroad_r0plus', 'e3_stop_restart_g1', 'solo_g1_20261016']
for name in names:
    motion, config, events = screen.load_inputs(screen.RECORDS[name])
    config = replace(config, occupancy_timing_mode='per_ray_candidate', recent_observation_enabled=False)
    candidate = M3ObservedFollow(config, allow_simulation_trial=True)
    baseline = LocalFollow(config)
    rows = []
    with screen.TruthBlocked():
        for event in events:
            if event['kind'] == 'reset':
                candidate.reset()
                baseline.reset()
                continue
            data, scan, history = screen.restore_call(screen.thaw(event['data']['input']), motion)
            scene = screen.motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                                        epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
            args = (scene, history, data['speed_mps'], data['side'], data['previous_steering_rad'], data['dt_s'])
            base = baseline.command(*args)
            started = time.perf_counter()
            out = candidate.command(*args)
            row = {'t': scene.t_ref_s, 'source': scene.source_first_s, 'output': freeze(out),
                   'base': freeze(base[:2]), 'base_reason': base[2]['reason'],
                   'ms': 1e3*(time.perf_counter()-started), 'equal_command': out[:2] == base[:2]}
            rows.append(row)
    reasons, modes = {}, {}
    for row in rows:
        detail = row['output'][2]
        reason = detail['reason']
        mode = detail.get('m3_lateral', {}).get('mode', 'BASE_GATE')
        reasons[reason] = reasons.get(reason, 0)+1
        modes[mode] = modes.get(mode, 0)+1
    summary = dict(calls=len(rows), reasons=reasons, lateral_modes=modes,
        candidate_positive=sum(r['output'][0] > 0 for r in rows), baseline_positive=sum(r['base'][0] > 0 for r in rows),
        equal_commands=sum(r['equal_command'] for r in rows),
        trial_positive=sum(r['output'][0]>0 and r['output'][2]['m3_selected'] for r in rows),
        ms_p50_p95_max=np.percentile([r['ms'] for r in rows], [50, 95, 100]).tolist(),
        over_20ms=sum(r['ms']>20 for r in rows))
    result['records'][name] = {'summary': summary, 'rows': rows}
    print(name, json.dumps(summary), flush=True)
Path(sys.argv[1]).write_text(json.dumps(result, indent=1), encoding='utf-8')
