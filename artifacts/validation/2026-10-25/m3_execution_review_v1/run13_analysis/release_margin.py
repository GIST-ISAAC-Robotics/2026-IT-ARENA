"""ON 반사실에서 해제 판단이 쓴 마지막 위협 반환 ds와 그 시점을 기록한다(읽기 전용, 소스 불변)."""
import json, sys
from dataclasses import replace
from pathlib import Path
from arena_autonomy.follow_trace import replay_call, freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.m3_execution_trial import M3ExecutionFollow, ExecutionConfig
out = {}
for path in sys.argv[1:-1]:
    events = [json.loads(x) for x in Path(path).read_text(encoding='utf-8').splitlines()]
    head = events[0]['data']
    f = M3ExecutionFollow(FollowConfig(**head['follow_config']),
                          replace(ExecutionConfig(**head['execution_config']), release_to_baseline=True), allow_simulation_trial=True)
    motion, last, rows = MotionConfig(**head['motion_config']), None, []
    for e in events[1:-1]:
        if e['kind'] == 'reset':
            f.reset(); continue
        o = freeze(replay_call(f, e['data']['input'], motion))
        lat = o[2].get('m3_lateral', {})
        present = [k for k, v in lat.get('threats', {}).items() if v.get('present')]
        if present:
            last = (e['data']['input']['reference_s'], lat.get('last_threat_rear_ds_m'))
        if lat.get('released_to_baseline'):
            rows.append({'release_s': e['data']['input']['reference_s'], 'last_present_s': last[0],
                         'last_threat_rear_ds_m': last[1], 'threshold_m': f.lateral_config.length_m/2})
    out[Path(path).parent.parent.name] = rows
Path(sys.argv[-1]).write_text(json.dumps(out, indent=1)+'\n', encoding='utf-8')
print(json.dumps(out))
