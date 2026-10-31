"""평가 전용 상대차 정답으로 정지/재출발 장면 성립을 확인한다."""
import math


def observed_restart(actor_poses, transitions, green):
    restart = green + 9.
    result = dict(restart_expected_s=restart, stopped_poses_covered=False,
                  resumed_poses_covered=False, stopped_then_moved=False,
                  restarted_during_bypass=False, normal_return_after_restart=False,
                  actual_motion_s=None, entry_s=None, return_s=None, truth_only=True)
    before = [r for r in actor_poses if restart-.9 <= r[0] <= restart-.1]
    after = [r for r in actor_poses if restart+.05 <= r[0] <= restart+1.05]

    def covered(rows, start, end):
        return (len(rows) >= 3 and abs(rows[0][0]-start) <= .05
                and abs(rows[-1][0]-end) <= .05
                and all(all(math.isfinite(v) for v in r[:3]) for r in rows)
                and all(0 < b[0]-a[0] <= .05 for a, b in zip(rows, rows[1:])))

    result['stopped_poses_covered'] = covered(before, restart-.9, restart-.1)
    result['resumed_poses_covered'] = covered(after, restart+.05, restart+1.05)
    if not result['stopped_poses_covered'] or not result['resumed_poses_covered']:
        return result
    anchor = before[-1][1:3]
    stopped = max(math.dist(r[1:3], anchor) for r in before)
    travel = sum(math.dist(a[1:3], b[1:3]) for a, b in zip(after, after[1:]))
    movement = next((r[0] for r in actor_poses
                     if restart <= r[0] <= restart+1.05
                     and math.dist(r[1:3], anchor) > .0035), None)
    result.update(stopped_spread_m=stopped, resumed_travel_m=travel, actual_motion_s=movement)
    result['stopped_then_moved'] = stopped <= .01 and .25 <= travel <= .45 and movement is not None
    if movement is None:
        return result
    prior = [r for r in transitions if r['time_s'] <= movement]
    if prior and prior[-1]['state'] == 'BYPASS':
        result['entry_s'] = prior[-1]['time_s']
        result['restarted_during_bypass'] = True
    returned = next((r for r in transitions if r['time_s'] > movement
                     and r['state'] == 'FOLLOW' and r.get('reason') == 'handoff_return'), None)
    if returned:
        result['normal_return_after_restart'] = True
        result['return_s'] = returned['time_s']
    return result
