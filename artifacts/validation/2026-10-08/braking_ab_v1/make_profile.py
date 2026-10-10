"""왼쪽 가림 실행의 표본화 요청을 고정 A/B 입력으로 추출. 원본은 보존한다."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
BASE = Path(__file__).resolve().parent
RUN = ROOT/'artifacts/validation/2026-10-07/g1b_side_v1/side_left_s0'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()

report = json.loads((RUN/'report.json').read_text())
audit = json.loads((RUN.parent/'side_left_source_audit.json').read_text())
assert sha(RUN/'trace.json') == audit['local_raw_artifact_sha256']['trace.json']
trace = json.loads((RUN/'trace.json').read_text())
green = report['green_observed_sim_s']
rows = [[0., 0., 0.]]
for row in trace:
    elapsed = row['t']-green
    if 0 < elapsed < 3.:
        rows.append([round(elapsed, 9), row['drive_speed_mps'], row['autonomy']['steering_command_rad']])
rows.append([3.5, 0., 0.])
assert rows[-2][1:] == [0., 0.]
profile = {
    'schema_version': 1, 'interpolation': 'previous_sample', 'duration_s': 3.5,
    'brake_window_s': [1.75, 2.85],
    'source': {
        'run': RUN.relative_to(ROOT).as_posix(), 'report_sha256': sha(RUN/'report.json'),
        'trace_sha256': sha(RUN/'trace.json'), 'origin_sim_s': green,
        'scope': '10 Hz trace의 /drive 속도 표본과 마지막 autonomy 조향 상태를 유지형 입력으로 재구성. 원래 50 Hz 요청 전체의 재생은 아니다. 현재 차체 정답/센서로 수정하지 않는 동일 A/B 입력.',
        'source_trace_max_gap_s': max(b['t']-a['t'] for a, b in zip(trace, trace[1:])),
        'steering_limit': 'autonomy 상태는 일반적으로 0.2초 또는 상태 전이 때 발행. 속도와 조향의 원본 취득 시각은 같다고 가정하지 않는다.',
    },
    'samples_t_speed_steering': rows,
}
with (BASE/'command_profile.json').open('x', encoding='utf-8') as stream:
    json.dump(profile, stream, indent=2, ensure_ascii=False, allow_nan=False)
print(json.dumps({'rows': len(rows), 'profile_sha256': sha(BASE/'command_profile.json')}))
