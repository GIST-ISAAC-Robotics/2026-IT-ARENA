"""r180 실제 현재 입력의 원 재검사/저속 전체 재검사를 같은 입력에서 비교한다."""
import copy
import json
from collections import Counter
from pathlib import Path
from arena_autonomy import current_path_recheck as old
from arena_autonomy.follow_trace import equal_output, freeze, restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
import candidate

A = Path(__file__).resolve().parent
TRACE = A.parent/'adjacent_low_runtime_r180_v1/two_actor_run1/run1/follow_decisions.jsonl'


def main():
    rows = []; counts = Counter(); mismatches = []
    with TRACE.open() as stream:
        header = json.loads(next(stream))['data']
        cfg = FollowConfig(**header['follow_config']); motion = MotionConfig(**header['motion_config'])
        for e in map(json.loads, stream):
            if e['kind'] != 'call' or 'current_rebase' not in e['data']: continue
            d = e['data']; now = d['current_rebase']['input']
            before = copy.deepcopy((d['output'], d['input'], now))
            _, scan, history = restore_call(now, motion)
            scene = motion_scene(scan, history, now['reference_s'], timing_verified=now['timing_verified'],
                                 epoch=now['epoch'], rear_half_angle_deg=now['rear_half_angle_deg'])
            base = old.rebase(d['output'], d['input'], now, scene, history, cfg)
            result = candidate.rebase(d['output'], d['input'], now, scene, history, cfg)
            if not equal_output(freeze(base), d['current_rebase']['output']):mismatches.append(e['seq'])
            assert equal_output(freeze((d['output'],d['input'],now)),freeze(before))
            if base['speed_mps'] > 0: assert equal_output(freeze(base),freeze(result))
            assert result['path_source_s'] == base['path_source_s']
            counts['calls'] += 1
            counts['stop_to_go'] += base['speed_mps'] <= 0 < result['speed_mps']
            counts['go_to_stop'] += result['speed_mps'] <= 0 < base['speed_mps']
            counts['changed'] += not equal_output(freeze(base),freeze(result))
            rows.append(dict(seq=e['seq'], old=base['speed_mps'], new=result['speed_mps'],
                             old_reason=base['reason'], new_reason=result['reason'],
                             retry=result.get('current_approach_low_retry')))
    with (A/'current_comparison.json').open('x') as f:
        json.dump(dict(counts=dict(counts), baseline_mismatches=mismatches, rows=rows),f,indent=2)
    print(json.dumps(dict(counts=dict(counts), baseline_mismatches=mismatches)))
    assert not mismatches and not counts['go_to_stop']


if __name__ == '__main__':main()
