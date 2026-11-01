"""동일 진척 지점의 첫 도착 시각. 평가기 기록만 읽으며 미도달은 None이다."""
import argparse
import json
from pathlib import Path


def arrival(rows, goal):
    for a, b in zip(rows, rows[1:]):
        p, q = a['progress_m'], b['progress_m']
        if p < goal <= q:
            return a['time'] + (b['time'] - a['time']) * (goal - p) / (q - p)
    return None


def compare(folder):
    with (folder / 'trajectory.jsonl').open() as stream:
        rows = [row for row in map(json.loads, stream) if row['light'] == 'green']
    if not rows:
        return {'run': folder.name, 'error': 'no green trajectory'}
    # 속도를 올린 뒤 공통 구간의 진행 시간이 실제로 줄었는지 비교한다.
    # 초록 첫 표본에서 기준점까지 생기는 최대 한 수신 간격은 포함하지 않는다.
    base = rows[0]['time']
    corner = {}
    for goal in (27., 28., 28.5, 28.8, 29., 30.):
        row = next((r for r in rows if r['progress_m'] >= goal), None)
        if row is not None:
            detail = row['autonomy']
            corner[str(goal)] = {'at_m': row['progress_m'], 'sim_s': row['time'],
                'body_longitudinal_mps': row['dynamics']['truth_longitudinal_mps'],
                'road_wall': detail.get('path_wall'), 'decision': detail.get('reason'),
                'steering_rad': detail.get('steering_command_rad')}
    return {'run': folder.name, 'first_green_s': base, 'start_progress_m': rows[0]['progress_m'],
            'corner_first_samples': corner,
            'elapsed_sim_s': {str(goal): (None if (t := arrival(rows, goal)) is None
                                          else round(t - base, 3)) for goal in (10, 20, 30, 40, 46.633)}}


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('folders', type=Path, nargs='+')
    ap.add_argument('--out', type=Path)
    args = ap.parse_args()
    result = json.dumps([compare(folder) for folder in args.folders], indent=2)
    if args.out:
        with args.out.open('x') as f:
            f.write(result + '\n')
    print(result)
