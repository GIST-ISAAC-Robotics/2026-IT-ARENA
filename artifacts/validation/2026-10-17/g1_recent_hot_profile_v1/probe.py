"""실제 기록의 보완 판단 6호출 cProfile. 계측 시간은 마감 판정에 쓰지 않는다."""
import cProfile
import json
from pathlib import Path
import pstats
import sys
import zipfile

from arena_autonomy import corridor_observer, observation_support, observation_lateral
from arena_autonomy.follow_trace import equal_output, freeze, restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.scene_motion import motion_scene
from audit_corridor_runtime import digest

ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent
CAPTURE = ROOT/'artifacts/validation/2026-10-17/g1_recent_solo_v1/run/follow_decisions.jsonl'
BASELINE = ROOT/'artifacts/validation/2026-10-17/g1_recent_full_profile_stream_v1/report.json'


def main():
    assert not (OUT/'report.json').exists()
    before = digest(CAPTURE)
    proof = json.loads(BASELINE.read_text())
    assert proof['passed'] and proof['input_unchanged'] and proof['input_sha256'] == before
    names = {Path(__file__).relative_to(ROOT).as_posix(): digest(Path(__file__))}
    for module in tuple(sys.modules.values()):
        location = getattr(module, '__file__', None)
        if location:
            path = Path(location).resolve()
            if path.is_relative_to(ROOT) and path.suffix == '.py':
                names[path.relative_to(ROOT).as_posix()] = digest(path)
    with zipfile.ZipFile(OUT/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for name in names:
            archive.write(ROOT/name, name)
    profiler = cProfile.Profile()
    matched = 0
    rows = []
    for line in CAPTURE.open():
        event = json.loads(line)
        if event['kind'] == 'header':
            motion = MotionConfig(**event['data']['motion_config'])
            follow = LocalFollow(FollowConfig(**event['data']['follow_config']))
        elif event['kind'] == 'reset':
            follow.reset()
        elif event['kind'] == 'call':
            if event['seq'] > 3156:
                break
            data, scan, history = restore_call(event['data']['input'], motion)
            scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
            measured = 3151 <= event['seq'] <= 3156
            if measured:
                profiler.enable()
            try:
                result = follow.command(scene, history, data['speed_mps'], data['side'],
                    data['previous_steering_rad'], data['dt_s'])
            finally:
                if measured:
                    profiler.disable()
            assert equal_output(freeze(result), event['data']['output']), event['seq']
            matched += 1
            if measured:
                rows.append(dict(seq=event['seq'], reason=result[2]['reason'],
                    recent=result[2].get('recent_observation_selected', False)))
    profiler.dump_stats(str(OUT/'calls.prof'))
    stats = pstats.Stats(profiler)
    top = []
    for (filename, line, function), (primitive, calls, own, cumulative, _) in sorted(
            stats.stats.items(), key=lambda item: item[1][3], reverse=True)[:35]:
        p = Path(filename)
        top.append(dict(file=p.relative_to(ROOT).as_posix() if p.is_relative_to(ROOT) else filename,
            line=line, function=function, primitive_calls=primitive, calls=calls,
            own_seconds=own, cumulative_seconds=cumulative))
    result = dict(scope='profiled_command_cost_attribution_not_latency_benchmark',
        baseline=BASELINE.relative_to(ROOT).as_posix(), baseline_sha256=digest(BASELINE),
        input_sha256=before, input_unchanged=digest(CAPTURE) == before,
        source_sha256=names, source_unchanged=all(digest(ROOT/n) == h for n, h in names.items()),
        matched=matched, rows=rows, top=top, profiler_changes_timing=True, online_deadline_verified=False)
    (OUT/'report.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({**{k: result[k] for k in ('matched', 'rows', 'input_unchanged', 'source_unchanged')}, 'top': top[:15]}))
    return 0 if len(rows) == 6 and result['input_unchanged'] and result['source_unchanged'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
