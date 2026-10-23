#!/usr/bin/env python3
"""검증된 sensor_probe의 세 시계와 기록기 내부 완료 지연을 분리한다. 발행/교정 없음."""
import argparse
from dataclasses import dataclass, field
import json
import math
from pathlib import Path

from audit_sensor_probe import audit


@dataclass
class Moments:
    """정수 ns로 정확한 합/제곱합. 표본 수에 비례하는 메모리나 근사 분위수 없음."""
    count: int = 0
    total: int = 0
    squared: int = 0
    minimum: int | None = None
    maximum: int | None = None

    def add(self, ns):
        if type(ns) is not int or abs(ns) > 2**64:
            raise ValueError('bounded integer interval required')
        self.count += 1
        self.total += ns
        self.squared += ns * ns
        self.minimum = ns if self.minimum is None else min(self.minimum, ns)
        self.maximum = ns if self.maximum is None else max(self.maximum, ns)

    def result(self):
        if not self.count:
            return dict(count=0, minimum_ms=None, maximum_ms=None,
                        mean_ms=None, population_stddev_ms=None)
        variance_numerator = self.count * self.squared - self.total**2
        return dict(count=self.count, minimum_ms=self.minimum / 1e6,
                    maximum_ms=self.maximum / 1e6, mean_ms=self.total / self.count / 1e6,
                    population_stddev_ms=math.sqrt(variance_numerator / self.count**2) / 1e6)


@dataclass
class Cadence:
    count: int = 0
    first: int | None = None
    last: int | None = None
    zero_stamps: int = 0
    negative_stamps: int = 0
    equal_adjacent: int = 0
    regressions: int = 0
    intervals: Moments = field(default_factory=Moments)

    def add(self, ns):
        if type(ns) is not int or not -2**63 <= ns <= 2**63 - 1:
            raise ValueError('int64 timestamp required')
        self.zero_stamps += ns == 0
        self.negative_stamps += ns < 0
        if self.last is not None:
            delta = ns - self.last
            self.intervals.add(delta)
            self.equal_adjacent += delta == 0
            self.regressions += delta < 0
        else:
            self.first = ns
        self.last = ns
        self.count += 1

    def result(self, *, header=False):
        span = None if self.first is None else self.last - self.first
        usable = self.count >= 2 and span > 0 and not self.regressions
        if header:
            usable = usable and not (self.zero_stamps or self.negative_stamps or self.equal_adjacent)
        return dict(samples=self.count, first_ns=self.first, last_ns=self.last,
                    span_ms=None if span is None else span / 1e6,
                    span_message_rate_hz=(self.count - 1) * 1e9 / span if usable else None,
                    interval=self.intervals.result(), zero_stamps=self.zero_stamps,
                    negative_stamps=self.negative_stamps, equal_adjacent=self.equal_adjacent,
                    regressions=self.regressions)


class ProbeTiming:
    """토픽마다 고정 크기 집계. DDS 부가 시각도 취득 clock으로 신뢰하지 않는다."""
    def __init__(self):
        self.topics = {}

    def observe(self, row):
        name = row['topic']
        if type(name) is not str or not name or len(name) > 256:
            raise ValueError('bounded topic name required')
        if name not in self.topics:
            if len(self.topics) >= 8:
                raise ValueError('bounded sensor topic count required')
            self.topics[name] = dict(monotonic=Cadence(), ros=Cadence(), header=Cadence(),
                                     completion=Moments(), publisher_gid_present=0,
                                     publisher_gid_missing=0, first_gid=None,
                                     multiple_known_publishers=False)
        t = self.topics[name]
        t['monotonic'].add(row['receive_monotonic_ns'])
        t['ros'].add(row['receive_ros_ns'])
        t['header'].add(row['source_ns'])
        writer = row['writer_monotonic_ns']
        if type(writer) is not int or not 0 <= writer <= 2**63 - 1:
            raise ValueError('bounded writer timestamp required')
        completion = writer - row['receive_monotonic_ns']
        if completion < 0:
            raise ValueError('writer return precedes callback')
        t['completion'].add(completion)
        gid = row.get('publisher_gid')
        if gid is None or gid == '' or (type(gid) is str and gid and set(gid) == {'0'}):
            t['publisher_gid_missing'] += 1
        elif type(gid) is str and len(gid) <= 128 and len(gid) % 2 == 0 and all(c in '0123456789abcdef' for c in gid):
            t['publisher_gid_present'] += 1
            if t['first_gid'] is None:
                t['first_gid'] = gid
            elif t['first_gid'] != gid:
                t['multiple_known_publishers'] = True
        else:
            raise ValueError('invalid publisher identity metadata')

    def result(self):
        result = {}
        for name, t in sorted(self.topics.items()):
            mono, ros, header = (t[k].result(header=k == 'header') for k in ('monotonic', 'ros', 'header'))
            # Mixed publishers do not share a proved header clock or a sensor cadence.
            if t['multiple_known_publishers']:
                header['span_message_rate_hz'] = None
            span = t['monotonic'].last - t['monotonic'].first
            result[name] = dict(
                callback_monotonic=mono, recorder_ros=ros, original_header=header,
                callback_to_writer_receipt=t['completion'].result(),
                recorder_ros_span_over_monotonic_span=(t['ros'].last-t['ros'].first) / span if span > 0 else None,
                publisher_gid_present=t['publisher_gid_present'],
                publisher_gid_missing=t['publisher_gid_missing'],
                multiple_known_publishers=t['multiple_known_publishers'])
        return result


def analyze(root):
    timing = ProbeTiming()
    integrity = audit(root, on_receipt=timing.observe)
    return dict(schema='arena_sensor_probe_timing_v1', record_integrity=integrity,
                per_topic=timing.result(),
                interpretation=dict(
                    source_age_computed=False, acquisition_latency_verified=False,
                    sensor_rate_verified=False, upstream_loss_verified=False,
                    pure_queue_wait_measured=False, physical_disk_flush_measured=False,
                    automatic_calibration_applied=False, ready_to_drive=False,
                    notes=[
                        'Each interval uses one clock only; header and ROS receipt epochs are not subtracted.',
                        'Header span rate describes messages, not proved exposure/laser/IMU acquisition rate.',
                        'Writer receipt is sampled after queue, deserialization, write, digest and stats updates; not pure queue or fsync latency.',
                        'Equal headers do not prove duplicate data; publisher identity can be unavailable.',
                        'Aggregate rates hide bursts: inspect min/max/stddev and equal/regressing timestamps.',
                        'DDS sequence gaps and unobserved production are not certified by these received messages.',
                    ]))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    if args.output.exists():
        raise FileExistsError(args.output)
    result = analyze(args.recording)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(dict(schema=result['schema'], topics=len(result['per_topic']),
                          record_integrity_verified=True, ready_to_drive=False)))


if __name__ == '__main__':
    main()
