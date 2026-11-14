"""기존 단1회 기록의 최초 지속 정지를 발췌한다. 제어 재실행/변경 없음."""
from pathlib import Path
import collections
import hashlib
import json
import math

A = Path(__file__).resolve().parent.parent
RUN = A / 'run1'
OUT = A / 'failure_inputs'


def read_lines(path):
    return [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, data):
    with path.open('x', encoding='utf-8') as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2)
        stream.write('\n')


def nearest(rows, timestamp, key):
    return min(rows, key=lambda row: abs(key(row) - timestamp))


def project_pose(ego, actor):
    dx, dy = actor[1] - ego[1], actor[2] - ego[2]
    c, s = math.cos(ego[3]), math.sin(ego[3])
    return dict(ego_pose=ego, actor_pose=actor,
                actor_in_ego_xy_m=[c * dx + s * dy, -s * dx + c * dy],
                basis='nearest evaluator ground truth only; x forward, y left; not controller input')


def main():
    OUT.mkdir(exist_ok=False)
    source = RUN / 'follow_decisions.jsonl'
    raw_lines = source.read_bytes().splitlines(keepends=True)
    rows = [json.loads(line) for line in raw_lines]
    calls = [row for row in rows if row['kind'] == 'call' and row['data']['compute']['stage'] == 'plan']
    first = next(i for i, row in enumerate(calls)
                 if row['data']['output'][2]['reason'] == 'moving_pass_current_path_unavailable'
                 and i > 0 and calls[i - 1]['data']['output'][0] > 0)
    chosen = calls[first - 2:first + 5]
    ids = {row['seq'] for row in chosen}
    headers = {'header', 'runtime_evaluator_header', 'runtime_factory_policy',
               'decision_submission_policy', 'current_path_recheck_policy', 'expiry_reference_policy'}
    selected = [line for line, row in zip(raw_lines, rows) if row['seq'] in ids or row['kind'] in headers]
    (OUT / 'selected_records.jsonl').write_bytes(b''.join(selected))
    ego = read_lines(RUN / 'ego_poses.jsonl')
    actor = read_lines(RUN / 'actor_poses.jsonl')
    trajectory = read_lines(RUN / 'trajectory.jsonl')
    summaries = []
    raw_points = []
    for row in chosen:
        data = row['data']
        inp = data['input']
        t = inp['reference_s']
        speed, steering, meta = data['output']
        moving = meta.get('moving_pass', {})
        entry = moving.get('entry', {})
        candidates = entry.get('candidates', [])
        compact_candidates = []
        for candidate in candidates:
            path = candidate.get('path_points', [])
            compact_candidates.append({
                **{key: candidate.get(key) for key in ('eligible', 'reason', 'offset_m', 'transition_m',
                    'steering_rad', 'target_speed_mps', 'command_clearance_m', 'follow_path_clearance_m',
                    'reference_remaining_m', 'preview_point_gap_m', 'maximum_occupancy_padding_m',
                    'occupied_points', 'command_occupancy_policy', 'motion_region')},
                'path_count': len(path), 'path_first': path[0] if path else None,
                'path_last': path[-1] if path else None,
            })
        measurement = inp['scan']['measurements']
        ranges = measurement['ranges_m']
        angles = measurement['angles_rad']
        finite = [(i, angle, distance) for i, (angle, distance) in enumerate(zip(angles, ranges))
                  if isinstance(angle, (int, float)) and isinstance(distance, (int, float))
                  and math.isfinite(angle) and math.isfinite(distance)]
        scan_summary = dict(representation=inp['scan']['representation'],
            source_frame_id=measurement.get('source_frame_id'), time_basis=measurement.get('time_basis'),
            raw_measurement_count=len(ranges), finite_count=len(finite),
            nonfinite_count=len(ranges) - len(finite),
            minimum_range_m=min((value[2] for value in finite), default=None),
            acquired_at_min_s=min(measurement['acquired_at_s']),
            acquired_at_max_s=max(measurement['acquired_at_s']))
        raw_points.append(dict(seq=row['seq'], reference_s=t, scan=inp['scan']))
        tr = nearest(trajectory, t, lambda value: value['t'])
        wall = entry.get('reference', {}).get('walls', {})
        wall_summary = {name: {key: value.get(key) for key in ('reason', 'slope', 'intercept_m',
            'observed_x_m', 'actual_return_count', 'residual_m', 'narrow_occluders_bridged')}
            for name, value in wall.items()}
        summaries.append(dict(seq=row['seq'], runtime_sequence=data['compute']['sequence'], reference_s=t,
            output_speed_mps=speed, output_steering_rad=steering, selector_reason=meta.get('reason'),
            behavior=meta.get('behavior'), handoff_state=meta.get('handoff_state'),
            measured_speed_mps=inp['speed_mps'], previous_steering_rad=inp['previous_steering_rad'],
            target=moving.get('target'), entry_reason=entry.get('reason'),
            entry_phase=entry.get('entry_phase'), entry_selected=entry.get('selected'),
            guide_mode=entry.get('guide_mode'), reference_reason=entry.get('reference', {}).get('reason'),
            current_walls=wall_summary, candidates=compact_candidates, raw_scan=scan_summary,
            truth=project_pose(nearest(ego, t, lambda value: value[0]), nearest(actor, t, lambda value: value[0])),
            trajectory={key: tr.get(key) for key in ('t', 'progress_m', 'ground_speed_mps',
                'drive_speed_mps', 'safe_speed_mps', 'safety_reason', 'safety_clearance_m',
                'wall_collision', 'sampled_gap_m', 'centerline_error_m', 'mcu_state')}))
    save(OUT / 'selected_raw_scans.json', raw_points)
    report = json.loads((RUN / 'report.json').read_bytes())
    green, end = report['case_window']['green_sim_s'], report['case_window']['end_sim_s']
    window = [row for row in calls if green <= row['data']['input']['reference_s'] <= end]
    summary = dict(source='run1/follow_decisions.jsonl', source_sha256=sha(source),
        first_persistent_zero_seq=chosen[2]['seq'], selected_call_sequences=sorted(ids),
        selection='last two positive plans and first five persistent zero plans; original JSONL bytes retained',
        records_sha256=sha(OUT / 'selected_records.jsonl'),
        replay_note='Stateful replay needs original full log prefix; seven isolated records do not capture internal selector state.',
        reason_counts=dict(collections.Counter(row['data']['output'][2]['reason'] for row in window)),
        first_zero_to_window_end_s=end-chosen[2]['data']['input']['reference_s'],
        summaries=summaries, code_changed=False, controller_reexecuted=False,
        inference_boundary='known_occupied_command_hold is observed. Which raw returns caused it is not yet replayed or attributed.',
        final_truth=project_pose(nearest(ego, end, lambda value: value[0]), nearest(actor, end, lambda value: value[0])))
    save(OUT / 'summary.json', summary)
    print(json.dumps(dict(path=str(OUT), first_zero_seq=summary['first_persistent_zero_seq'],
        selected_call_sequences=summary['selected_call_sequences'], reason_counts=summary['reason_counts']),
        ensure_ascii=False))


if __name__ == '__main__':
    main()
