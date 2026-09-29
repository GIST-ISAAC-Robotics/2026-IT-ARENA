#!/usr/bin/env python3
"""A3 재생 판정과 CPU/RSS/온도 증거를 집계한다. 기준을 변경해 통과시키지 않는다."""
import argparse
import json
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/arena_vehicle_interface'))
from arena_vehicle_interface.bag_contract import quantiles
from summarize_controller_performance import summarize


def trial_summary(path):
    trial = json.loads((path/'trial.json').read_text())
    report = json.loads((path/'replay/report.json').read_text())
    result = summarize(path/'replay')
    result['run'] = path.name
    result['runtime'] = report['runtime']
    result['trial'] = trial
    result['timing_coverage'] = {
        name: dict(received=report['callback_counts'].get(name, 0), completed_at_snapshot=completed)
        for name, completed in report['completed_callback_counts'].items()}
    result['diagnostic_sequence_coverage'] = report.get('diagnostic_sequence_coverage')
    result['measurement_passed'] = report.get('measurement_passed')
    result['vision_stats'] = report.get('vision_stats', {})
    result['input_diagnostic_delivery_complete'] = all(
        value['callbacks'] == value['diagnostic_rows'] for value in report['input_callback_delivery'].values())
    result['timing_caveat'] = (
        'Sequence coverage is checked through each controller snapshot; quantiles include received rows after that cutoff too. '
        'No full-run upper bound is claimed for unobserved shutdown-tail callbacks. Sensor callback loss is separate.'
        if report.get('measurement_passed') else
        'Duration/interval quantiles summarize received rows only; missing diagnostics prevent full-run bounds. '
        'Sensor callback loss is separately measured by controller counters; control snapshot precedes final shutdown.')
    result['control_intervals_wall_ms'] = report['timing'].get('local_pursuit/control', {}).get('wall_interval_ms')
    result['source_age_ms'] = {name: values['source_age_ms'] for name, values in report['timing'].items()
                              if name.startswith('local_pursuit/')}
    start, end = result['reference_active_window_s']
    vision_age = []
    for status in report.get('status_records', {}).get('/autonomy/status', []):
        last = status.get('vision', {}).get('last_result')
        now = status.get('sim_time_s')
        if last and now is not None and start <= now <= end:
            vision_age.append((now-last['stamp'])*1000)
    result['vision_age_at_active_status_ms'] = quantiles(vision_age)
    result['vision_age_caveat'] = ('Sparse status samples of currently used observation age; '
        'not every result acceptance latency, camera driver latency or a full-run bound')
    active_began, active_ended = [], []
    with (path/'replay/timing.jsonl').open() as stream:
        for line in stream:
            row = json.loads(line)
            if row['node'] == 'local_pursuit' and start <= row['receive_ros_ns']/1e9 <= end:
                active_began.append(row['begin_monotonic_ns']/1e9)
                active_ended.append(row['end_monotonic_ns']/1e9)
    window = (min(active_began), max(active_ended)) if active_began else (float('inf'), float('-inf'))
    cpu, active_cpu, rss, previous = [], [], [], None
    with (path/'resources.jsonl').open() as stream:
        for line in stream:
            row = json.loads(line)
            by_pid = {p['pid']: p for p in row['processes']}
            rss.append(sum(p['rss_bytes'] for p in by_pid.values())/1024**2)
            if previous:
                old_time, old = previous
                elapsed = row['monotonic_s']-old_time
                ticks = sum(max(0, p['cpu_ticks']-old[pid]['cpu_ticks']) for pid, p in by_pid.items() if pid in old)
                value = ticks/trial['clock_ticks_per_second']/elapsed*100
                cpu.append(value)
                if window[0] <= old_time and row['monotonic_s'] <= window[1]:
                    active_cpu.append(value)
            previous = row['monotonic_s'], by_pid
    temps, gpu, ram, power = [], [], [], []
    for line in (path/'tegrastats.log').read_text().splitlines():
        temps.extend(float(v) for v in re.findall(r'@[ ]*([0-9.]+)C', line))
        gpu.extend(float(v) for v in re.findall(r'GR3D_FREQ\s+(\d+)%', line))
        ram.extend(float(v) for v in re.findall(r'RAM\s+(\d+)/\d+MB', line))
        power.extend(float(v) for v in re.findall(r'VDD_IN\s+(\d+)mW/', line))
    result['resources'] = dict(cpu_percent_one_core_100=quantiles(cpu),
        active_cpu_percent_one_core_100=quantiles(active_cpu), summed_rss_mib=quantiles(rss),
        max_reported_temperature_c=max(temps) if temps else None,
        gpu_percent=quantiles(gpu), system_ram_mb=quantiles(ram), input_power_mw=quantiles(power),
        caveat='RSS sums shared pages; CPU includes replay descendants including observer if present, excludes telemetry sampler/tegrastats; full trial includes audit/start/EOF; active CPU uses original moving window')
    delivery = path/'replay/delivery_audit.json'
    if delivery.exists():
        result['delivery_audit'] = json.loads(delivery.read_text())
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trials', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    results = [trial_summary(path) for path in args.trials]
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(results, stream, indent=2)
    print(json.dumps([dict(run=r['run'], passed=r['passed'], zero=r['active_window_zero_commands'],
                          commands=r['active_window_commands'], durations_ms=r['durations_ms'],
                          resources=r['resources']) for r in results]))


if __name__ == '__main__':
    main()
