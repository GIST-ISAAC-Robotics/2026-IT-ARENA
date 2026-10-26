"""저장 입력 첫 피추월 구간의 함수별 비용 위치. 계측 오버헤드가 있어 실시간 보장용 수치가 아니다."""
import argparse
import cProfile
import hashlib
import json
from pathlib import Path
import pstats
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path[:0] = [str(ROOT/p) for p in ('src/arena_autonomy', 'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import freeze, replay_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.m3_execution_trial import ExecutionConfig, M3ExecutionFollow


def main(output):
    output.mkdir(parents=True, exist_ok=False)
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py')}
    hashes[str(Path(__file__).relative_to(ROOT))] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    summary = {}
    for name in ('run17_observed_release_closed_loop', 'run21_right_encounter'):
        path = ROOT/'artifacts/validation/2026-10-25/m3_execution_review_v1'/name/'run/follow_decisions.jsonl'
        raw = path.read_bytes(); events = [json.loads(line) for line in raw.decode('utf-8').splitlines()]
        header = events[0]['data']
        expected = json.loads((HERE.parent/'m3_rear_residual_v1/cachefix_runs/run7_premises_cachefix'/
                              (name+'__plain.json')).read_text())
        ctl = M3ExecutionFollow(FollowConfig(**header['follow_config']),
                               ExecutionConfig(**header['execution_config']), allow_simulation_trial=True)
        motion = MotionConfig(**header['motion_config'])
        profile = cProfile.Profile(); differences = []; calls = resets = 0
        for i, event in enumerate(events[1:-1]):
            if event['kind'] == 'reset':
                ctl.reset(); resets += 1
                if expected[i] != {'reset': True}: differences.append(i)
                continue
            if event['kind'] != 'call': raise ValueError('unexpected event')
            if event['data']['input']['reference_s'] > 6.:
                break
            result = profile.runcall(replay_call, ctl, event['data']['input'], motion)
            digest = hashlib.sha256(json.dumps(freeze(result), sort_keys=True).encode()).hexdigest()
            if digest != expected[i]['sha']: differences.append(i)
            calls += 1
        profile.dump_stats(str(output/(name+'.prof')))
        stats = pstats.Stats(profile)
        rows = []
        for (file, line, function), (primitive, total, own, cumulative, callers) in stats.stats.items():
            rows.append(dict(file=file, line=line, function=function, primitive=primitive, calls=total,
                             self_seconds=own, cumulative_seconds=cumulative))
        rows.sort(key=lambda x: x['cumulative_seconds'], reverse=True)
        (output/(name+'_functions.json')).write_text(json.dumps(rows[:100], indent=2)+'\n')
        summary[name] = dict(decision_calls=calls, resets=resets, differences=differences,
                            instrumented_total_seconds=stats.total_tt,
                            input_sha256=hashlib.sha256(raw).hexdigest(), input_unchanged=path.read_bytes() == raw,
                            top_functions=rows[:12])
        print(json.dumps({name: {k:v for k,v in summary[name].items() if k!='top_functions'}}), flush=True)
    unchanged = all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    passed = unchanged and all(v['decision_calls'] and not v['differences'] and v['input_unchanged'] for v in summary.values())
    (output/'summary.json').write_text(json.dumps(dict(passed=passed, source_sha256=hashes,
        source_unchanged=unchanged, results=summary,
        scope='instrumented saved-input first encounter; cost attribution only, no Gazebo/real-time guarantee'), indent=2)+'\n')
    return 0 if passed else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('--output', type=Path, required=True)
    sys.exit(main(parser.parse_args().output))
