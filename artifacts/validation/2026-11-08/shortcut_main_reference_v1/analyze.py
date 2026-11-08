"""r11 본선 저장 궤적의 분기 전후 시간과 영상 관측. 평가 전용, 제어기 입력 아님."""
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
TRACE = ROOT / 'artifacts/validation/2026-11-07/junction_handoff_v1/run1/trace.json'
SCENE = ROOT / 'src/arena_gazebo/worlds/it_arena_official_v2026_11_06/scene.json'
trace = json.loads(TRACE.read_text(encoding='utf8'))
scene = json.loads(SCENE.read_text(encoding='utf8'))
length = scene['track']['lap_length_m']
start_s = trace[0]['s']


def crossing(s):
    target = (s - start_s) % length
    for a, b in zip(trace, trace[1:]):
        if a['progress_m'] <= target < b['progress_m']:
            k = (target-a['progress_m']) / (b['progress_m']-a['progress_m'])
            return {'t_estimate_s': a['time']+k*(b['time']-a['time']),
                    'bracket_s': [a['time'], b['time']],
                    'main_s_m': s, 'target_progress_m': target}
    raise ValueError(f'no crossing: {s}')


results = []
for branch in scene['branches']:
    marker = 30 if 'P21' in branch['name'] else 20
    detections = [q for q in trace if marker in q['autonomy'].get('marker_ids', [])]
    # Status messages can repeat a last accepted camera result: deduplicate acquisition stamps.
    frames = sorted({q['autonomy']['vision']['last_result']['stamp'] for q in detections})
    enter, leave = crossing(branch['s0_m']), crossing(branch['s1_m'])
    duration = leave['t_estimate_s']-enter['t_estimate_s']
    results.append({
        'branch': branch['name'], 'marker_id': marker,
        'main_length_m': branch['s1_m']-branch['s0_m'],
        'branch_length_m': branch['length_m'], 'road_width_m': branch['width_m'],
        'entry': enter, 'exit': leave, 'main_duration_s': duration,
        'main_duration_bracket_s': [leave['bracket_s'][0]-enter['bracket_s'][1],
                                    leave['bracket_s'][1]-enter['bracket_s'][0]],
        'ideal_branch_average_for_equal_time_mps': branch['length_m']/duration,
        'status_detection_samples': len(detections),
        'unique_sampled_camera_frames': len(frames),
        'first_last_sampled_frame_stamp_s': [frames[0], frames[-1]],
        'first_status_detection_time_s': detections[0]['time'],
        'first_detection_to_entry_s': enter['t_estimate_s']-detections[0]['time'],
        'first_detection_to_entry_main_distance_m': enter['target_progress_m']-detections[0]['progress_m'],
    })
out = {'scope': 'saved single main-route simulation, evaluator only',
       'limitations': ['main s is nearest-centerline projection with discrete corner gates',
                      'no shortcut driving; equal-time speed excludes changed entry/exit or rejoining cost',
                      'sampled status contains only a subset of all camera frames; not detector recall',
                      'no repeat-run variance or hardware evidence'],
       'inputs': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in (TRACE, SCENE)},
       'branches': results}
(HERE/'comparison.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(json.dumps(out,ensure_ascii=False,indent=2))
