"""마커 건너뜀 재배치의 실행·입력·실제 진행을 분리해 요약한다(평가 전용)."""
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'scripts'))
from race_ops_eval import Course


def main():
    run = Path(sys.argv[1])
    report = json.loads((run/'report.json').read_text(encoding='utf-8'))
    course = Course()
    closes = [step for step in report['steps'] if step['step'] == 'resume_closed']
    ends = [step for step in report['steps'] if step['step'] == 'across_end']
    result = dict(completed=report.get('completed', False), error=report.get('error'),
                  exit_codes=report.get('exit_codes'), cleanup=report.get('auxiliary_cleanup'),
                  remaining_pids=report.get('remaining_test_pids'), source_unchanged=report.get('source_unchanged'),
                  stop_verified=(report.get('gazebo_stop_service') or {}).get('verified'),
                  note='정답은 사후 평가 전용. contact 센서는 이 실행에 없으며 접촉 0으로 해석하지 않는다.')
    if closes and ends:
        start, end = closes[-1], ends[-1]
        rows = [json.loads(line) for line in (run/'ego_poses.jsonl').read_text().splitlines()]
        rows = np.array([row for row in rows if start['truth'][0] <= row[0] <= end['truth'][0]])
        route = [course.nearest(x, y) for _, x, y, *_ in rows]
        ss, errors = np.array(route).T
        steps = np.diff(ss)
        length = course.s[-1]
        steps = np.where(steps < -length/2., steps+length, np.where(steps > length/2., steps-length, steps))
        intervals = np.diff(rows[:, 0])
        speeds = np.linalg.norm(np.diff(rows[:, 1:3], axis=0), axis=1)/intervals
        duration = longest = 0.
        for dt, speed in zip(intervals, speeds):
            duration = duration+dt if speed < .05 else 0.
            longest = max(longest, duration)
        history = json.loads((run/'ops_history.json').read_text(encoding='utf-8'))
        history = [r for r in history if start['wall_s'] <= r.get('wall', 0) <= end['wall_s']]
        result.update(resume=start, final=end, samples=len(rows), duration_s=float(rows[-1, 0]-rows[0, 0]),
                      start_s=float(ss[0]), end_s=float(ss[-1]), route_progress_m=float(steps.sum()),
                      distance_m=float(np.linalg.norm(np.diff(rows[:, 1:3], axis=0), axis=1).sum()),
                      max_centerline_error_m=float(errors.max()),
                      branch_samples=sum(course.in_branch(x, y) for _, x, y, *_ in rows),
                      longest_under_005_mps_s=longest, peak_mps=float(speeds.max()),
                      history_samples=len(history),
                      observed_sides=sorted({r['driving']['side'] for r in history}),
                      observed_marker_ids=sorted({i for r in history for i in (r['driving'].get('marker_ids') or [])}))
    target = run.parent/(run.name+'_analysis.json')
    target.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('resume','final')}, ensure_ascii=False))


if __name__ == '__main__':
    main()
