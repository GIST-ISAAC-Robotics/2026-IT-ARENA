#!/usr/bin/env python3
"""같은 기록 영상에서 OpenCV 작업 스레드 수만 비교한다. ROS 발행 없음."""
import argparse
import json
from pathlib import Path
import time

import cv2
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from audit_sensor_bag import reader_for
from arena_autonomy.core import StartSignal, image_rgb, marker_ids
from arena_vehicle_interface.bag_contract import load_manifest, quantiles


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--reference', type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    load_manifest(args.recording)
    reader = reader_for(args.recording)
    frames = []
    previous = -1.
    while reader.has_next():
        topic, payload, _ = reader.read_next()
        if topic != '/camera/color/image_raw':
            continue
        message = deserialize_message(payload, Image)
        stamp = message.header.stamp.sec+message.header.stamp.nanosec/1e9
        if stamp-previous < .095:
            continue
        previous = stamp
        frames.append((stamp, image_rgb(message)))
    original_threads = cv2.getNumThreads()
    runs = []
    try:
        for threads in (original_threads, 1, 1, original_threads):
            cv2.setNumThreads(threads)
            signal = StartSignal()
            outputs, total, signals, markers = [], [], [], []
            for stamp, rgb in frames:
                began = time.perf_counter_ns()
                signal.update(rgb)
                midway = time.perf_counter_ns()
                ids = sorted(set(marker_ids(rgb, .02)))
                end = time.perf_counter_ns()
                total.append((end-began)/1e6)
                signals.append((midway-began)/1e6)
                markers.append((end-midway)/1e6)
                outputs.append([stamp, ids, signal.started, signal.observed,
                                None if signal.red_roi is None else list(signal.red_roi),
                                signal.red_frames, signal.green_frames])
            runs.append(dict(threads=threads, total_ms=quantiles(total),
                signal_ms=quantiles(signals), marker_ms=quantiles(markers), outputs=outputs))
    finally:
        cv2.setNumThreads(original_threads)
    result = dict(opencv=cv2.__version__, original_threads=original_threads,
        frame_count=len(frames), frame_shape=list(frames[0][1].shape), runs=runs,
        outputs_identical=all(r['outputs'] == runs[0]['outputs'] for r in runs),
        scope='Offline same-frame comparison; does not prove executor deadlines')
    if args.reference:
        reference = json.loads(args.reference.read_text())
        result['reference_outputs_identical'] = all(r['outputs'] == reference['runs'][0]['outputs'] for r in runs)
        result['reference'] = str(args.reference)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps({**result, 'runs': [{k: v for k, v in r.items() if k != 'outputs'} for r in runs]}))
    return int(not result['outputs_identical'] or result.get('reference_outputs_identical') is False)


if __name__ == '__main__':
    raise SystemExit(main())
