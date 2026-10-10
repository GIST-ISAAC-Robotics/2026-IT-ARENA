#!/usr/bin/env python3
"""G1-b 실행 보고서 묶음 요약. 판정을 다시 계산하지 않고 각 report.json의 기록만 모은다."""
import argparse
import hashlib
import json
from pathlib import Path


def row(path):
    report = json.loads(path.read_text(encoding='utf-8'))
    evaluation = report.get('evaluation', {})
    gap, progress = evaluation.get('gap', {}), evaluation.get('progress', {})
    return {
        'run': path.parent.name, 'case': report.get('case'), 'seed': report.get('seed'), 'mode': report.get('mode'),
        'passed': report.get('passed'), 'run_valid': report.get('run_valid'), 'harness_valid': report.get('harness_valid'),
        'behavior_gates_passed': report.get('behavior_gates_passed'),
        'failed_gates': sorted(k for k, v in (report.get('gates') or {}).items() if not v),
        'failed_integrity': sorted(k for k, v in (report.get('integrity') or {}).items() if not v),
        'failed_stop_checks': sorted(k for k, v in (report.get('stop_checks') or {}).items() if not v),
        'error': report.get('error'), 'abort': report.get('abort'),
        'min_sampled_gap_m': gap.get('min_sampled_gap_m'), 'min_swept_lower_bound_m': gap.get('min_swept_lower_bound_m'),
        'contacts': gap.get('actor_contact_messages_with_ego'), 'progress_m': progress.get('at_window_end_m'),
        'gap_at_window_end_m': progress.get('sampled_gap_at_window_end_m'),
        'case_metrics': evaluation.get('case_metrics'), 'behavior_counts': evaluation.get('controller_mode', {}).get('behavior_counts'),
        'hold_reasons': evaluation.get('controller_mode', {}).get('hold_reasons'),
        'stop_displacement_m': (report.get('stop') or {}).get('displacement_m'),
        'deadline_diagnostics': (report.get('actuation_diagnostics') or {}).get('control_deadline_samples'),
        'shutdown_clean': report.get('shutdown_clean'), 'wall_elapsed_s': report.get('wall_elapsed_s'),
        'report_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('roots', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rows = [row(path) for root in args.roots for path in sorted(root.glob('*/report.json'))]
    g1 = [r for r in rows if r['mode'] == 'g1']
    summary = {'runs': rows, 'g1_passed': sum(bool(r['passed']) for r in g1), 'g1_runs': len(g1),
               'r0_valid_runs': sum(bool(r['passed']) for r in rows if r['mode'] == 'r0'),
               'r0_runs': sum(r['mode'] == 'r0' for r in rows),
               'scope': 'collection of per-run reports; per-run verdicts are not recomputed'}
    args.output.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    for r in rows:
        print(f"{r['case']:15s} s{r['seed']} {r['mode']} passed={r['passed']} valid={r['run_valid']} "
              f"gap={r['min_sampled_gap_m']} swept={r['min_swept_lower_bound_m']} prog={r['progress_m']} "
              f"fail={r['failed_gates']+r['failed_integrity']+r['failed_stop_checks']} err={r['error']} abort={r['abort']}")


if __name__ == '__main__':
    main()
