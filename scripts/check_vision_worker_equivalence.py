#!/usr/bin/env python3
"""같은 영상·같은 순서에서 inline/별도 프로세스 판정 비교. 구동 명령 없음."""
import argparse
import json
from pathlib import Path
import time

from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from arena_autonomy.core import StartSignal, image_rgb, marker_ids
from arena_autonomy.vision_process import VisionProcess
from arena_vehicle_interface.bag_contract import load_manifest, quantiles
from audit_sensor_bag import reader_for


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    load_manifest(args.recording)
    direct, candidate = StartSignal(), StartSignal()
    reader = reader_for(args.recording)
    frames = []
    previous = float('-inf')
    worker = VisionProcess()
    try:
        while reader.has_next():
            topic, payload, _ = reader.read_next()
            if topic != '/camera/color/image_raw':
                continue
            image = deserialize_message(payload, Image)
            stamp = image.header.stamp.sec+image.header.stamp.nanosec*1e-9
            if stamp-previous < .095:
                continue
            previous = stamp
            request = dict(epoch=0, sequence=len(frames)+1, stamp=stamp, signal=vars(candidate),
                image={key: getattr(image, key) for key in ('height', 'width', 'step', 'encoding')})
            worker.submit(request, image.data)
            rgb = image_rgb(image)
            direct.update(rgb)
            ids = sorted(set(marker_ids(rgb, .02)))
            deadline = time.monotonic()+5
            result = None
            while result is None and time.monotonic() < deadline:
                result = worker.poll()
                time.sleep(.001)
            if result is None:
                raise RuntimeError('Worker result timeout')
            same = result['signal'] == vars(direct) and result['ids'] == ids
            candidate.__dict__.update(result['signal'])
            frames.append(dict(stamp=stamp, identical=same, expected_ids=ids,
                expected_signal=vars(direct).copy(), result=result))
    finally:
        cleanup = worker.close()
    report = dict(passed=bool(frames and all(r['identical'] for r in frames)
                              and cleanup == {'forced': False, 'exitcode': 0}),
        frames=frames, frame_count=len(frames), cleanup=cleanup,
        processing_ms=quantiles([r['result']['duration_ms'] for r in frames]),
        scope='Same sequential frames only; live latest-frame selection and timing differ')
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2)
    print(json.dumps({k: v for k, v in report.items() if k != 'frames'}))
    return int(not report['passed'])


if __name__ == '__main__':
    raise SystemExit(main())
