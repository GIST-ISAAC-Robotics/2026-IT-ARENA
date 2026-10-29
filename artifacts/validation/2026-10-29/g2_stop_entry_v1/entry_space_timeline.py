"""저장 판단 입력에서 G2 진입 후보의 여유를 '진입 시점'과 '다음 새 스캔 직전' 두 기준으로 비교한다.

실행 중 명령 검사는 같은 스캔을 재사용하는 동안 표적 반환의 취득 후 이동 여유
(0.5 m/s×나이)가 커진다. 진입 후보 비교는 진입 순간의 나이만 쓴다. 여기서는 측정된
스캔 간격(연속 source_first_s 차이)만큼 나이를 더한 여유로 같은 비교를 다시 계산한다.
고정 외벽 분류·기본/측정 여유·후보 목록·조향 한계는 그대로다. 오프라인 분석이며
생산 코드·주행에는 연결하지 않는다. 평가 정답은 사후 비교용이다.
"""
import json
from pathlib import Path
import sys

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
import arena_autonomy.bypass_entry_trial as trial
from arena_autonomy.bypass_entry_follow import BypassEntryFollow, BypassExecutionConfig
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene

CLOSED = BASE.parent/'g2_prepare_cap_v1/closed1'
TRACES = [('stopping_new', CLOSED/'stopping_combined__observed_entry_relevance_v1', 1.7, 1.9, 3.52),
          ('stopping_legacy', BASE/'legacy_stop1/stopping_combined__front_patch_v0', 1.7, 1.9, 3.52),
          ('right_combined_new', CLOSED/'right_combined__observed_entry_relevance_v1', 1.2, .9, 1.42),
          ('left_combined_new', CLOSED/'left_combined__observed_entry_relevance_v1', 1.2, .9, 1.42),
          ('right_far_new', CLOSED/'right_far_combined__observed_entry_relevance_v1', 2.4, 2.2, 2.82)]
ORIGINAL = trial.per_ray_padding


def compare(scene, now, speed, previous, dt, config, entry, extra_age):
    def padded(sc, base, bound):
        return ORIGINAL(sc, base, bound)+bound*extra_age
    trial.per_ray_padding = padded
    try:
        result = trial._compare_entries(scene, now, speed, previous, dt, config=config, entry=entry,
                                        stationary_boundaries=True)
    finally:
        trial.per_ray_padding = ORIGINAL
    rows = [r for r in result['candidates'] if r.get('eligible') and r['offset_m']]
    # 경로 끝까지 현재 반환 간섭이 없고 점 여유가 0 이상인 후보만 '실행 일관 후보'로 센다.
    full = [r for r in rows if r['preview_return_progress_m'] >= r['path_points'][-1][0]-r['path_points'][0][0]-1e-9]
    best = max(full, key=lambda r: r['preview_point_gap_m'], default=None)
    return dict(reason=result['reason'], eligible_nonzero=len(rows), full_preview=len(full),
        full_nonnegative_gap=sum(r['preview_point_gap_m'] >= 0 for r in full),
        best=None if best is None else [best['offset_m'], best['transition_m'], round(best['preview_point_gap_m'], 4)])


def main():
    results = {}
    for name, stem, target_x, lo, hi in TRACES:
        records = {round(r['t'], 3): r for r in json.loads(Path(str(stem)+'.records.json').read_text(encoding='utf-8'))}
        rows = []; last_source = None
        with open(str(stem)+'.jsonl', encoding='utf-8') as stream:
            header = json.loads(stream.readline())['data']
            motion = MotionConfig(**header['motion_config']); config = FollowConfig(**header['follow_config'])
            entry = BypassEntryFollow(config, BypassExecutionConfig(**header['execution_config']['bypass']),
                                      allow_simulation_trial=True).guide.entry
            for line in stream:
                event = json.loads(line)
                if event['kind'] != 'call':
                    continue
                data, scan, history = restore_call(event['data']['input'], motion)
                now = data['reference_s']
                scene = motion_scene(scan, history, now, timing_verified=True, epoch=data['epoch'],
                                     rear_half_angle_deg=data['rear_half_angle_deg'])
                interval = None if last_source is None else scene.source_first_s-last_source
                fresh = last_source is None or scene.source_first_s != last_source
                last_source = scene.source_first_s
                if not (lo <= now <= hi and fresh and interval) or data['speed_mps'] < 0:
                    continue
                rec = records.get(round(now, 3), {}).get('evaluation', {})
                center = rec.get('center_pose', [None])
                row = dict(t=round(now, 3), age_s=round(now-scene.source_first_s, 3), scan_interval_s=round(interval, 3),
                    measured_speed_mps=round(data['speed_mps'], 3),
                    truth_rear_axle_to_face_m=None if center[0] is None else round(target_x-.1-(center[0]-.0675), 3),
                    decision=event['data']['output'][2].get('handoff_state'),
                    at_entry_age=compare(scene, now, data['speed_mps'], data['previous_steering_rad'], data['dt_s'], config, entry, 0.),
                    before_next_scan=compare(scene, now, data['speed_mps'], data['previous_steering_rad'], data['dt_s'], config, entry, interval))
                rows.append(row)
                print(name, json.dumps(row), flush=True)
        results[name] = rows
    (BASE/'entry_space_timeline.json').write_text(json.dumps(dict(results=results,
        scope='Offline re-evaluation of stored G2 entry comparisons with padding grown by the measured scan interval. '
              'Not a controller change; truth only for post-hoc distance.'), indent=2)+'\n', encoding='utf-8')


if __name__ == '__main__':
    main()
