"""r358의 실제 두 계산 프로세스에서 계획 함수만 제한 계측한다.

직렬 call 로그에는 폐기된 계획의 상태 갱신이 전부 없으므로 동일 판단 재생으로
쓰지 않는다. 이 진단은 실제 시간 인계 흐름을 사용하지만 계측 오버헤드를 포함한다.
"""
from pathlib import Path
import cProfile
import hashlib
import json
import pstats
import sys
import time

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface'),
               str(A.parent/'shared_ray_timed_r358_v1')]


def worker(stage, *args):
    import classification_worker
    from arena_autonomy import follow_trace
    original = follow_trace.replay_call
    records, profiles = [], []

    def measured(*a, **kw):
        index = len(records)
        profile = cProfile.Profile() if index in (2, 10, 30, 60, 90, 120) else None
        if profile:
            profile.enable()
        begin = time.perf_counter()
        try:
            result = original(*a, **kw)
        finally:
            elapsed = (time.perf_counter()-begin)*1000
            if profile:
                profile.disable()
        records.append(dict(index=index, elapsed_ms=elapsed, profiled=bool(profile),
                            speed=result[0], state=result[2].get('handoff_state')))
        if profile:
            stats = pstats.Stats(profile).stats
            chosen = set(sorted(stats, key=lambda k: stats[k][3], reverse=True)[:30])
            chosen.update(sorted(stats, key=lambda k: stats[k][2], reverse=True)[:30])
            profiles.append(dict(index=index, functions=[dict(
                file=k[0].replace(str(ROOT)+'/', ''), line=k[1], name=k[2],
                calls=stats[k][1], self_ms=stats[k][2]*1000, cumulative_ms=stats[k][3]*1000)
                for k in sorted(chosen, key=lambda k: stats[k][3], reverse=True)]))
        return result

    if stage == 'plan':
        follow_trace.replay_call = measured
    try:
        classification_worker.worker(stage, *args)
    finally:
        follow_trace.replay_call = original
        if stage == 'plan':
            (A/'runtime_result'/'plan_profile.json').write_text(json.dumps(
                dict(records=records, profiles=profiles,
                     profiling_overhead_included=True, closed_loop=False), indent=2)+'\n')


def install():
    import overlap
    overlap._worker = worker


def main():
    import timed_replay
    manifest = json.loads((A.parent/'shared_ray_timed_r358_v1/source_manifest.json').read_bytes())
    def changed():
        return [n for n, h in manifest.items()
                if hashlib.sha256((ROOT/n).read_bytes()).hexdigest() != h]
    assert not changed()
    out = A/'runtime_result'
    out.mkdir(exist_ok=False)
    timed_replay.install = install
    sys.argv = ['timed_replay', '--classification', 'shared', '--policy', 'deadline',
                '--mode', 'delay20', '--submission', 'latest', '--duration', '31',
                '--plan-ms', '0', '--current-ms', '0', '--out', str(out/'run'),
                '--trace', 'artifacts/validation/2026-11-13/final_geometry_crawl_drive_r353_v1/run1/follow_decisions.jsonl']
    try:
        timed_replay.main()
    finally:
        (out/'source_check.json').write_text(json.dumps(dict(
            checked=len(manifest), changed=changed(), devices_opened=False))+'\n')
    assert not changed()


if __name__ == '__main__':
    main()
