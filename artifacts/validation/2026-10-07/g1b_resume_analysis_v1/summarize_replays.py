#!/usr/bin/env python3
"""상세 재생 행(로컬·Git 제외)에서 추적 가능한 요약·핵심 행·해시를 만든다."""
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
KEEP = ('t', 'scan_s', 'reason', 'command_mps', 'measured_speed_mps', 'boundary_selection', 'boundary_candidate_ahead_m',
        'fresh_boundary_accepted', 'boundary_from_memory', 'boundary_age_s', 'path_wall', 'boundary_protrusions_removed',
        'left_unfiltered_ahead_m', 'right_unfiltered_ahead_m')


def main():
    summary = {'scope': 'open-loop replay summaries; detailed per-tick rows stay local (Git-excluded) with SHA-256 below',
               'replays': {}}
    for path in sorted(HERE.glob('replay_*.json')):
        if path.name == 'replay_summary.json':
            continue
        data = json.loads(path.read_text())
        rows = [row for row in data['rows'] if row.get('boundary_selection') is not None
                and (7.4 <= row['t'] <= 8.25 or 8.5 <= row['t'] <= 8.75 or 9.0 <= row['t'] <= 9.25)]
        summary['replays'][path.name] = {
            'sha256': hashlib.sha256(path.read_bytes()).hexdigest(), 'window_s': data['window_s'], 'gyro': data['gyro'],
            'cold_start_s': data['cold_start_s'], 'local_follow_sha256': data['local_follow_sha256'],
            'inputs_sha256': data['inputs_sha256'], 'summary': data['summary'],
            'fresh_rows': [{k: row.get(k) for k in KEEP} for row in rows]}
    (HERE/'replay_summary.json').write_text(json.dumps(summary, indent=1, ensure_ascii=False, default=float), encoding='utf-8')
    for name, item in summary['replays'].items():
        print(name, item['local_follow_sha256'][:12], json.dumps(item['summary']))


if __name__ == '__main__':
    main()
