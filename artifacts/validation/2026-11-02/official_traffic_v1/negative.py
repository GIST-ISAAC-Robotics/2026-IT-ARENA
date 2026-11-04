"""경기 목표 평가의 부정 사례. 접촉·STOP·세 바퀴·미복귀가 통과로 숨지 않는지 사본에서 확인한다."""
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
WORLD = ROOT/'src/arena_gazebo/worlds/it_arena_official'


def report_change(change):
    def apply(run, surface):
        data = json.loads((run/'report.json').read_bytes())
        change(data)
        (run/'report.json').write_text(json.dumps(data), encoding='utf-8')
    return apply


def three_wheels(run, surface):
    data = json.loads(surface.read_bytes())
    data['interpretations']['any']['max_off_wheels'] = 3
    surface.write_text(json.dumps(data), encoding='utf-8')


def never_rejoined(run, surface):
    """끝 3초를 진행 방향 왼쪽 0.26 m로 민다."""
    rows = json.loads((run/'trace.json').read_bytes())
    end = rows[-1]['t']
    for index, row in enumerate(rows):
        if row['t'] >= end-3.:
            yaw = row['yaw']
            row['x'] -= .26*math.sin(yaw)
            row['y'] += .26*math.cos(yaw)
    (run/'trace.json').write_text(json.dumps(rows), encoding='utf-8')


def scene_not_established(run, surface):
    """상대가 정지를 유지하지 않은 실행으로 바꾼다."""
    path = run/'actor_poses.jsonl'
    rows = [json.loads(line) for line in path.read_bytes().splitlines()]
    green = json.loads((run/'report.json').read_bytes())['evaluation']['t_green']
    for row in rows:
        if row[0]-green > 10.:
            row[2] += .05*(row[0]-green-10.)
    path.write_text('\n'.join(json.dumps(row) for row in rows), encoding='utf-8')


def empty_interpretations(run, surface):
    data = json.loads(surface.read_bytes())
    data['interpretations'] = {}
    surface.write_text(json.dumps(data), encoding='utf-8')


CASES = dict(not_established=scene_not_established, empty_surface=empty_interpretations,
             invalid_run=report_change(lambda d: d.update(run_valid=False, harness_valid=True)),
             swept_gap=report_change(lambda d: d['evaluation']['gap'].update(min_swept_lower_bound_m=.05)),
             contact=report_change(lambda d: d['evaluation']['gap'].update(actor_contact_messages_with_ego=1)),
             stop=report_change(lambda d: d['stop_checks'].update(explicit_mcu_stop=False)),
             three_wheels=three_wheels, never_rejoined=never_rejoined)


def main():
    results = {}
    for name, change in CASES.items():
        with tempfile.TemporaryDirectory() as temporary:
            run = Path(temporary)/'p1'
            run.mkdir()
            for file in ('report.json', 'trace.json', 'actor_poses.jsonl', 'follow_decisions.jsonl'):
                shutil.copy(HERE/'p1'/file, run/file)
            surface = Path(temporary)/'p1_surface.json'
            shutil.copy(HERE/'p1_surface.json', surface)
            change(run, surface)
            subprocess.run([sys.executable, '-B', str(HERE/'goal_eval.py'), str(run), str(WORLD)], check=True, stdout=subprocess.DEVNULL)
            result = json.loads((Path(temporary)/'p1_goal.json').read_bytes())
            results[name] = dict(passed=result['passed'], failed=[k for k, v in result['gates'].items() if not v])
    (HERE/'goal_eval_negative.json').write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding='utf-8', newline='\n')
    print(json.dumps(results, ensure_ascii=False))
    raise SystemExit(1 if any(row['passed'] for row in results.values()) else 0)


if __name__ == '__main__':
    main()
