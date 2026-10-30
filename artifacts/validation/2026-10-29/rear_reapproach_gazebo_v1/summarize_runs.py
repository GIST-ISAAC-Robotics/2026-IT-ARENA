"""기존 G1 판정을 보존하며 후방 재접근 등록 항목을 별도로 모은다."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'scripts'))
import g1_traffic_eval as ev
from traffic_encounter import encounter_metrics
import numpy as np


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def summarize(output):
    report = read(output/'report.json')
    runtime = read(output/'trial_overlay.json')
    transition = read(output/'rear_transition_audit.json')
    replay = read(output/'follow_trace_replay.json')
    wheels = read(output.parent/(output.name+'_wheel_centers.json'))
    side = report['additional_actors']['actors']['eval_e3_side']
    primary = report['evaluation']
    fixture = ROOT/'artifacts/validation/2026-10-27/official_departure_v2/fixture'
    route = np.genfromtxt(fixture/'centerline.csv', delimiter=',', names=True)
    scene = read(fixture/'scene.json')
    centerline = ev.Centerline(route['x_m'], route['y_m'], route['s_m'], scene['track']['lap_length_m'])
    ego = [json.loads(line) for line in (output/'ego_poses.jsonl').read_text().splitlines()]
    actor = read(output/'additional_actors.json')['poses']['eval_e3_side']
    first = report['case_window']['green_sim_s']
    encounter = encounter_metrics(ego, actor, centerline, first,
                                  first+report['case_window']['duration_s'], minimum_gap_m=.15)
    progress = primary['progress']['at_window_end_m']
    stall = primary['case_metrics']['longest_stationary_while_lead_moves_s']
    max_wheels = max(int(k) for k, n in wheels['wheel_count_histogram'].items() if n)
    contact = report['additional_actors']['combined_contacts']
    reg = dict(scene=transition['scenario_realized'], progress=progress >= 2.5,
               stall=stall <= 2.5, wheels=max_wheels <= 2,
               sampled_no_overlap=(side['min_sampled_gap_m'] > 0. and
                                   primary['gap']['sampled_overlap_samples'] == 0),
               no_contact_events=contact['valid'] and contact['ego_events'] == 0,
               no_wall_samples=report['wall_collision_samples'] == 0,
               stop=all(report['stop_checks'].values()), shutdown=report['shutdown_clean'],
               run_valid=report['run_valid'], input_integrity=all(report['integrity'].values()),
               runtime=runtime['runtime_contract']['passed'] and runtime['source_unchanged'],
               replay=replay['passed'] and replay['input_unchanged'],
               transition_pose_coverage=transition['all_pose_coverage_valid'],
               encounter_valid=encounter['valid'])
    result = dict(candidate=transition['candidate'], registered_gates=reg,
                  registered_passed=all(reg.values()), original_g1_passed=report['passed'],
                  original_g1_gates=report['gates'], original_side_gates=side['gates'],
                  progress_m=progress, longest_stationary_s=stall,
                  minimum_side_gap_m=side['min_sampled_gap_m'],
                  minimum_side_swept_gap_m=side['min_swept_lower_bound_m'],
                  maximum_lateral_change_m=encounter['maximum_ego_road_lateral_change_m'],
                  maximum_wheels_outside=max_wheels, contact_events=contact['ego_events'],
                  stop_displacement_m=report['stop']['displacement_m'],
                  normal_shutdown=report['shutdown_clean'],
                  replay_calls=replay['calls'], replay_matched=replay['matched'],
                  state_counts=transition['state_counts'],
                  selector_modes=transition['selector_modes'], lane_keep_reasons=transition['lane_keep_reasons'],
                  scope='registered bounded scenario, not whole lap, independent opponents or hardware proof')
    compact = {k: v for k, v in replay.items() if k not in ('outcomes', 'mismatches')}
    compact['mismatches'] = (replay.get('mismatches') or [])[:3]
    compact['detail_storage'] = 'Raw per-call outcomes and sensor records remain local.'
    target = output/'follow_trace_replay_summary.json'
    if not target.exists():
        with target.open('x', encoding='utf-8') as f:
            json.dump(compact, f, ensure_ascii=False, indent=2)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = {'runs': {p.name:summarize(p.resolve()) for p in args.runs},
              'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    with args.output.open('x', encoding='utf-8') as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
    print(json.dumps(result, ensure_ascii=False))
