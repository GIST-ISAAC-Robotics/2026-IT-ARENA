#!/usr/bin/env python3
"""운영 기록을 한 사건의 시간축 CSV/HTML로 내보낸다. 기록만 읽고 원본을 고치지 않는다.

--decisions는 기존 G1 판단 기록을 재사용한다. --truth는 평가용 정답 파일이며 표에 섞지 않고 별도 절로 둔다.
"""
import argparse
import json
from pathlib import Path

from arena_autonomy import race_ops_export as export


def center_of(rows, request):
    times = [row['sim_s'] for row in rows if row.get('request') == request and row.get('sim_s') is not None]
    if not times:
        raise ValueError(f'request not found in log: {request}')
    return times[0]


def truth_rows(path, center, before, after):
    """ego_poses.jsonl 형식 [t, x, y, yaw, z]. 0.5초 간격만 남긴다."""
    rows, last = [], None
    with open(path, encoding='utf-8') as stream:
        for line in stream:
            t, x, y = json.loads(line)[:3]
            if center-before <= t <= center+after and (last is None or t-last >= .5):
                rows.append(dict(sim_s=t, x_m=x, y_m=y))
                last = t
    return rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True, help='확장자 없는 출력 경로')
    parser.add_argument('--decisions', type=Path)
    parser.add_argument('--truth', type=Path)
    selected = parser.add_mutually_exclusive_group()
    selected.add_argument('--center-sim-s', type=float)
    selected.add_argument('--request', help='이 요청 번호의 첫 기록을 사건 중심으로 쓴다')
    parser.add_argument('--before-s', type=float, default=5.)
    parser.add_argument('--after-s', type=float, default=5.)
    args = parser.parse_args(argv)
    rows, invalid = export.read_jsonl(args.log)
    center = args.center_sim_s if args.request is None else center_of(rows, args.request)
    if center is not None:
        rows = export.window(rows, center, args.before_s, args.after_s)
    times = [row['sim_s'] for row in rows if row.get('sim_s') is not None]
    decisions, truth = [], []
    if args.decisions and times:
        decisions = export.decision_rows(args.decisions, min(times), max(times))
    if args.truth and center is not None:
        truth = truth_rows(args.truth, center, args.before_s, args.after_s)
    table = export.merge(rows, decisions=decisions, truth=truth)
    args.output.with_suffix('.csv').write_text(export.render_csv(table), encoding='utf-8', newline='\n')
    note = f'중심 sim {center} s, 깨진 기록 줄 {invalid}개' if center is not None else f'전체 기록, 깨진 기록 줄 {invalid}개'
    args.output.with_suffix('.html').write_text(export.render_html(table, '사건 시간축', note), encoding='utf-8', newline='\n')
    print(json.dumps(dict(rows=len(table['rows']), gaps=len(table['gaps']), decisions=len(decisions), truth=len(truth),
                          invalid=invalid), ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
