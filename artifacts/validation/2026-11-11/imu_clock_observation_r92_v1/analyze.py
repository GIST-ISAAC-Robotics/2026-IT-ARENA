"""기존 정지 IMU 기록의 시계 비율/변동 분석. 취득 시각이나 보정값을 승인하지 않는다."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import statistics


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def percentile(values, p):
    a = sorted(values)
    if not a:
        return None
    i = (len(a) - 1) * p
    lo = math.floor(i)
    return a[lo] + (a[math.ceil(i)] - a[lo]) * (i - lo)


def spread(values):
    a = list(values)
    if not a:
        return dict(count=0)
    return dict(count=len(a), minimum=min(a), p50=percentile(a, .5),
                p95=percentile(a, .95), p99=percentile(a, .99), maximum=max(a))


def fit(x, y):
    if len(x) != len(y) or len(x) < 2:
        raise ValueError('paired clocks need at least two samples')
    mx, my = statistics.fmean(x), statistics.fmean(y)
    denom = math.fsum((v - mx) ** 2 for v in x)
    if denom <= 0:
        raise ValueError('clock has no elapsed span')
    slope = math.fsum((a - mx) * (b - my) for a, b in zip(x, y)) / denom
    return slope, my - slope * mx


def clock_relation(x, y, time_s, split_s=60.):
    """처음60초로 맞춘 비율을 이후 데이터에 그대로 적용한다. 모든 단위는µs."""
    train = [i for i, t in enumerate(time_s) if t < split_s]
    held = [i for i, t in enumerate(time_s) if t >= split_s]
    if len(train) < 2 or len(held) < 2:
        raise ValueError('insufficient train/held-out span')
    slope, intercept = fit([x[i] for i in train], [y[i] for i in train])
    residual = [(y[i] - slope * x[i] - intercept) / 1000 for i in held]
    chunks = []
    for start in range(0, math.ceil(max(time_s)), 30):
        ix = [i for i, t in enumerate(time_s) if start <= t < start + 30]
        if len(ix) > 1:
            block_slope, _ = fit([x[i] for i in ix], [y[i] for i in ix])
            chunks.append(dict(start_s=start, count=len(ix), slope=block_slope))
    return dict(training_s=split_s, train_samples=len(train), held_out_samples=len(held),
                slope=slope, fitted_offset_us=intercept,
                held_out_residual_ms=spread(residual),
                held_out_absolute_residual_ms=spread(abs(v) for v in residual),
                block_slopes=chunks)


def analyze(path):
    before = sha(path)
    stamps = []
    pattern = Counter()
    slot_patterns = Counter()
    slot_increments = Counter()
    previous_slot = None
    cycle = []
    read_duration = []
    triplet_read_offsets = []
    boot = None
    previous_seq = None
    raw_previous = None
    unwrapped = 0
    wraps = 0
    previous_host = None
    with path.open(encoding='utf8') as stream:
        for line in stream:
            row = json.loads(line)
            if row['phase'] != 'measure':
                continue
            if boot is None:
                boot = row['boot']
            if row['boot'] != boot:
                raise ValueError('boot changed; fit each epoch separately')
            if previous_seq is not None and (row['sequence'] - previous_seq) % 2**32 != 1:
                raise ValueError('wire sequence gap; do not bridge missing frames')
            previous_seq = row['sequence']
            if previous_host is not None and row['host_monotonic_ns'] < previous_host:
                raise ValueError('host clock reversed')
            previous_host = row['host_monotonic_ns']
            if row['type'] != 1:
                continue
            tag = row['sensor_tag']
            if row['read_end_us'] < row['read_begin_us']:
                raise ValueError('read interval reversed')
            read_duration.append(row['read_end_us'] - row['read_begin_us'])
            if tag == 4:
                if cycle:
                    pattern[','.join(str(v[0]) for v in cycle)] += 1
                    slot_patterns[','.join(f'{v[0]}:{v[2]}' for v in cycle)] += 1
                    if [v[0] for v in cycle] == [4, 1, 2]:
                        triplet_read_offsets.append((cycle[1][1] - cycle[0][1],
                                                     cycle[2][1] - cycle[0][1]))
                cycle = []
                slot = (row['tag_raw'] >> 1) & 3
                if previous_slot is not None:
                    slot_increments[str((slot - previous_slot) % 4)] += 1
                previous_slot = slot
                raw = row['sensor_timestamp_raw']
                if raw_previous is not None:
                    step = (raw - raw_previous) % 2**32
                    if not 0 < step < 2**31:
                        raise ValueError('timestamp reset/reversal without boot boundary')
                    unwrapped += step
                    wraps += int(raw < raw_previous)
                raw_previous = raw
                stamps.append((unwrapped, row['read_begin_us'], row['host_monotonic_ns']))
            cycle.append((tag, row['read_begin_us'], (row['tag_raw'] >> 1) & 3))
    if len(stamps) < 2:
        raise ValueError('no timestamp span')
    # 마지막 cycle은 다음 timestamp가 없어 완결성을 주장하지 않는다.
    s0, m0, h0 = stamps[0]
    ticks = [r[0] - s0 for r in stamps]
    mcu_us = [r[1] - m0 for r in stamps]
    host_us = [(r[2] - h0) / 1000 for r in stamps]
    time_s = [v / 1e6 for v in mcu_us]
    fitted = clock_relation(ticks, mcu_us, time_s)
    transport = clock_relation(mcu_us, host_us, time_s)
    return dict(input_sha256=before, input_unchanged=before == sha(path),
        timestamp_samples=len(stamps), measurement_span_s=time_s[-1], wraps_observed=wraps,
        completed_fifo_tag_patterns=dict(pattern), read_duration_us=spread(read_duration),
        completed_slot_patterns=dict(slot_patterns), timestamp_slot_increments=dict(slot_increments),
        gyro_read_after_timestamp_us=spread(v[0] for v in triplet_read_offsets),
        accel_read_after_timestamp_us=spread(v[1] for v in triplet_read_offsets),
        sensor_tick_to_mcu_read=fitted, mcu_read_to_host_receipt=transport,
        end_span_mismatch_ms=dict(nominal_25us=(ticks[-1]*25-mcu_us[-1])/1000,
                                 factory_candidate_25_div_0p955=(ticks[-1]*(25/.955)-mcu_us[-1])/1000),
        time_contract_verified=False, calibration_adopted=False,
        scope='static saved observation only; read/receipt clocks are not acquisition truth; affine intercept includes unknown transport/phase; no sensor movement or hardware change')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('recording', type=Path)
    p.add_argument('output', type=Path)
    args = p.parse_args()
    result = analyze(args.recording)
    with args.output.open('x', encoding='utf8') as dst:
        dst.write(json.dumps(result, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps({k: result[k] for k in ('timestamp_samples', 'measurement_span_s',
        'end_span_mismatch_ms', 'input_unchanged', 'time_contract_verified')}))


if __name__ == '__main__':
    main()
