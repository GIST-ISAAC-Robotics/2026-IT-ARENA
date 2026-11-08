"""1단계: 저장 follow_decisions 입력만으로 전 랩 관측. 궤적·지도·마커·정답은 읽지 않는다."""
import hashlib
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(HERE))
from observe import observe  # noqa: E402
from arena_autonomy.follow_trace import restore_call  # noqa: E402
from arena_autonomy.scene_motion import motion_scene  # noqa: E402
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable  # noqa: E402

RUNS = {
    'r11': 'artifacts/validation/2026-11-07/junction_handoff_v1/run1',
    'r27': 'artifacts/validation/2026-11-08/traffic_faster_reacquire_live_v1/two_actor_course_lap/run1',
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main(name):
    source = ROOT / RUNS[name] / 'follow_decisions.jsonl'
    before = digest(source)
    rows, last, skipped, calls = [], None, {}, 0
    with source.open(encoding='utf-8') as stream:
        for line in stream:
            e = json.loads(line)
            if e['kind'] == 'header':
                mc = MotionConfig(**e['data']['motion_config'])
            if e['kind'] != 'call':
                continue
            calls += 1
            d, raw, h = restore_call(e['data']['input'], mc)
            stamp = (raw.header.stamp.sec, raw.header.stamp.nanosec)
            if stamp == last:
                continue
            last = stamp
            try:
                scene = motion_scene(raw, h, d['reference_s'], timing_verified=d['timing_verified'],
                                     epoch=d['epoch'], rear_half_angle_deg=d['rear_half_angle_deg'])
            except (ValueError, MotionUnavailable) as error:
                key = type(error).__name__ + ':' + str(error)
                skipped[key] = skipped.get(key, 0) + 1
                continue
            obs = observe(scene)
            rows.append(dict(t=d['reference_s'], scan_stamp=list(stamp),
                             returns=sum(r.status == 'return' for r in scene.rays),
                             unknown=sum(r.status == 'unknown' for r in scene.rays), **obs))
    out = dict(run=name, source=source.relative_to(ROOT).as_posix(), sensor_sha256=before,
               input_unchanged=before == digest(source), observer_sha256=digest(HERE / 'observe.py'),
               calls=calls, scans=len(rows), skipped=skipped, rows=rows)
    (HERE / f'observations_{name}.json').write_bytes(
        (json.dumps(out, ensure_ascii=False, separators=(',', ':')) + '\n').encode('utf-8'))
    n = lambda side, kind: sum(bool(r[side][kind]) for r in rows)
    print(json.dumps(dict(run=name, scans=len(rows), skipped=skipped, input_unchanged=out['input_unchanged'],
                          right_notch_scans=n('right', 'notches'), right_term_scans=n('right', 'terminations'),
                          left_notch_scans=n('left', 'notches'), left_term_scans=n('left', 'terminations'))))


if __name__ == '__main__':
    main(sys.argv[1])
