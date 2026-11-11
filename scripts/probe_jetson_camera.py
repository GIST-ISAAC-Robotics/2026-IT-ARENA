#!/usr/bin/env python3
"""구동 명령 없이 Argus → BGR 수신률과 프레임 간격을 기록한다.

Jetson 시스템 Python의 gi/GStreamer, numpy, cv2를 사용한다. 원시 CSV와
실내 프레임은 비공개 경로에 보관한다. Gst PTS는 센서 노출 시각이 아니다.
"""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import statistics
import time
import traceback
import zlib


def distribution(values):
    ordered = sorted(values)
    if not ordered:
        return None

    def percentile(p):
        position = (len(ordered) - 1) * p
        lower = int(position)
        upper = min(lower + 1, len(ordered) - 1)
        return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)

    return {"count": len(ordered), "min": ordered[0],
            "median": statistics.median(ordered), "p95": percentile(.95),
            "p99": percentile(.99), "max": ordered[-1]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--duration", type=float, default=300)
    parser.add_argument("--warmup", type=float, default=5)
    parser.add_argument("--sensor-id", type=int, default=0)
    parser.add_argument("--sensor-mode", type=int, default=4)
    parser.add_argument("--fps", type=int, default=60)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    args = parser.parse_args()
    if args.duration <= 0 or args.warmup < 0:
        parser.error("duration must be positive and warmup nonnegative")
    args.output.mkdir(parents=True, exist_ok=False)

    import gi
    gi.require_version("Gst", "1.0")
    gi.require_version("GstVideo", "1.0")
    from gi.repository import Gst, GstVideo
    import cv2
    import numpy as np

    Gst.init(None)
    pipeline_text = (
        f"nvarguscamerasrc name=camera sensor-id={args.sensor_id} "
        f"sensor-mode={args.sensor_mode} do-timestamp=true ! "
        f"video/x-raw(memory:NVMM),width={args.width},height={args.height},"
        f"format=NV12,framerate={args.fps}/1 ! nvvidconv ! "
        "video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! "
        "appsink name=receiver sync=false max-buffers=8 drop=false wait-on-eos=false"
    )
    pipeline = Gst.parse_launch(pipeline_text)
    sink = pipeline.get_by_name("receiver")
    source = pipeline.get_by_name("camera")
    bus = pipeline.get_bus()
    source_rows = []
    rows = []
    telemetry = []
    errors = []
    warnings = []
    qos = []
    representative = []
    first_frame = None
    first_pts = None
    first_time = None
    measured_start = None
    last_telemetry = 0
    caps_text = None
    last_frame = None
    last_pts = None
    state_null = False
    eos_received = False
    cpu_start = time.process_time()

    def source_probe(pad, info):
        buffer = info.get_buffer()
        if buffer is not None:
            source_rows.append((int(buffer.pts), time.monotonic_ns(),
                                int(buffer.offset), int(buffer.get_flags())))
        return Gst.PadProbeReturn.OK

    source.get_static_pad("src").add_probe(Gst.PadProbeType.BUFFER, source_probe)

    def messages():
        nonlocal eos_received
        while True:
            message = bus.pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.WARNING |
                                       Gst.MessageType.EOS | Gst.MessageType.QOS)
            if message is None:
                break
            if message.type == Gst.MessageType.ERROR:
                error, debug = message.parse_error()
                errors.append({"message": str(error), "debug": debug})
            elif message.type == Gst.MessageType.WARNING:
                warning, debug = message.parse_warning()
                warnings.append({"message": str(warning), "debug": debug})
            elif message.type == Gst.MessageType.EOS:
                eos_received = True
            else:
                qos.append(str(message.get_structure()))

    def temperatures():
        result = {}
        for zone in Path("/sys/class/thermal").glob("thermal_zone*"):
            try:
                result[(zone / "type").read_text().strip()] = (
                    int((zone / "temp").read_text()) / 1000)
            except (OSError, ValueError, TypeError):
                pass
        return result

    def save_frame(frame, name, pts):
        path = args.output / name
        if not cv2.imwrite(str(path), frame):
            raise RuntimeError("Failed to save representative frame")
        representative.append({"file": name, "pts_ns": pts,
                               "shape": list(frame.shape),
                               "bgr_mean": frame.mean(axis=(0, 1)).tolist(),
                               "pixel_std": float(frame.std()),
                               "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})

    start = time.monotonic()
    try:
        if pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("Pipeline failed to enter PLAYING")
        while True:
            sample = sink.emit("try-pull-sample", 5 * Gst.SECOND)
            received = time.monotonic_ns()
            messages()
            if errors:
                break
            if sample is None:
                errors.append({"message": "No frame within 5 seconds", "eos": eos_received})
                break
            buffer = sample.get_buffer()
            if first_time is None:
                first_time = received
            caps = sample.get_caps()
            if caps_text is None:
                caps_text = caps.to_string()
            elif caps.to_string() != caps_text:
                raise RuntimeError("Caps changed during capture")
            if received - first_time < args.warmup * 1e9:
                continue
            if measured_start is None:
                measured_start = received
            video = GstVideo.VideoInfo.new_from_caps(caps)
            mapped, data = buffer.map(Gst.MapFlags.READ)
            if not mapped:
                raise RuntimeError("Buffer mapping failed")
            try:
                stride = int(video.stride[0])
                frame = np.ndarray((video.height, video.width, 3), np.uint8,
                                   buffer=data.data, offset=int(video.offset[0]),
                                   strides=(stride, 3, 1))
                crc = zlib.crc32(data.data)
                if first_frame is None:
                    first_frame = frame.copy()
                    first_pts = int(buffer.pts)
                if received - measured_start >= args.duration * 1e9:
                    last_frame = frame.copy()
                    last_pts = int(buffer.pts)
            finally:
                buffer.unmap(data)
            rows.append((int(buffer.pts), received, int(buffer.duration),
                         int(buffer.offset), int(buffer.get_flags()), crc, stride))
            if time.monotonic() - last_telemetry >= 5:
                telemetry.append({"elapsed_s": (received - measured_start) / 1e9,
                                  "temperatures_c": temperatures(),
                                  "loadavg": list(os.getloadavg())})
                last_telemetry = time.monotonic()
                print(json.dumps({"frames": len(rows), "elapsed_s":
                                  (received - measured_start) / 1e9}), flush=True)
            if last_frame is not None:
                break
        pipeline.send_event(Gst.Event.new_eos())
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and not eos_received and not errors:
            messages()
            time.sleep(.02)
    except Exception as error:
        errors.append({"message": repr(error), "traceback": traceback.format_exc()})
    finally:
        pipeline.set_state(Gst.State.NULL)
        result, state, pending = pipeline.get_state(5 * Gst.SECOND)
        state_null = result != Gst.StateChangeReturn.FAILURE and state == Gst.State.NULL
        messages()

    if first_frame is not None:
        save_frame(first_frame, "first.png", first_pts)
    if last_frame is not None:
        save_frame(last_frame, "last.png", last_pts)
    with (args.output / "frames.csv").open("x", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(("pts_ns", "host_monotonic_ns", "duration_ns", "offset",
                         "flags", "crc32", "stride"))
        writer.writerows(rows)
    with (args.output / "source.csv").open("x", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(("pts_ns", "host_monotonic_ns", "offset", "flags"))
        writer.writerows(source_rows)
    period_ns = 1e9 / args.fps
    pts_deltas = [b[0] - a[0] for a, b in zip(rows, rows[1:])]
    host_deltas = [b[1] - a[1] for a, b in zip(rows, rows[1:])]
    pts_span = (rows[-1][0] - rows[0][0]) / 1e9 if len(rows) > 1 else 0
    host_span = (rows[-1][1] - rows[0][1]) / 1e9 if len(rows) > 1 else 0
    pts_fps = (len(rows) - 1) / pts_span if pts_span > 0 else None
    host_fps = (len(rows) - 1) / host_span if host_span > 0 else None
    source_window = [row for row in source_rows
                     if rows and rows[0][0] <= row[0] <= rows[-1][0]]
    source_pts = [row[0] for row in source_window]
    sink_pts = [row[0] for row in rows]
    source_times = {row[0]: row[1] for row in source_window}
    transit = [(row[1] - source_times[row[0]]) / 1e6
               for row in rows if row[0] in source_times]
    summary = {
        "schema": 1, "pipeline": pipeline_text, "gstreamer": Gst.version_string(),
        "requested": {"width": args.width, "height": args.height, "fps": args.fps,
                      "duration_s": args.duration, "warmup_s": args.warmup,
                      "sensor_id": args.sensor_id, "sensor_mode": args.sensor_mode},
        "caps": caps_text, "frames": len(rows), "source_frames_in_pts_window": len(source_window),
        "source_sink_pts_identical": source_pts == sink_pts,
        "pts_span_s": pts_span, "host_span_s": host_span,
        "pts_fps": pts_fps, "host_fps": host_fps,
        "pts_interval_ms": distribution([value / 1e6 for value in pts_deltas]),
        "host_interval_ms": distribution([value / 1e6 for value in host_deltas]),
        "source_to_sink_ms": distribution(transit),
        "pts_gaps_gt_1_5_period": sum(value > period_ns * 1.5 for value in pts_deltas),
        "pts_gaps_gt_2_period": sum(value > period_ns * 2 for value in pts_deltas),
        "host_gaps_gt_1_5_period": sum(value > period_ns * 1.5 for value in host_deltas),
        "host_gaps_gt_2_period": sum(value > period_ns * 2 for value in host_deltas),
        "nonpositive_pts_intervals": sum(value <= 0 for value in pts_deltas),
        "consecutive_identical_crc32": sum(a[5] == b[5] for a, b in zip(rows, rows[1:])),
        "discont_after_first": sum(bool(row[4] & int(Gst.BufferFlags.DISCONT)) for row in rows[1:]),
        "errors": errors, "warnings": warnings, "qos_messages": qos,
        "eos_received": eos_received, "state_null": state_null,
        "process_cpu_s": time.process_time() - cpu_start,
        "total_wall_s": time.monotonic() - start,
        "telemetry": telemetry, "representative": representative,
        "limits": ["Gst source PTS is not verified sensor exposure time or capture ID",
                   "No videorate or intentional frame duplication/drop in pipeline",
                   "CRC32 inequality excludes byte-identical consecutive buffers, not every sensor fault",
                   "No optical clock, rolling-shutter, calibration or ROS timestamp validation"],
    }
    summary["basic_pass"] = bool(
        host_span >= args.duration and pts_fps and pts_fps >= args.fps * .99 and
        host_fps >= args.fps * .99 and not errors and
        summary["nonpositive_pts_intervals"] == 0 and
        summary["consecutive_identical_crc32"] == 0 and
        summary["source_sink_pts_identical"] and eos_received and state_null)
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({key: summary[key] for key in (
        "basic_pass", "frames", "pts_fps", "host_fps", "errors", "eos_received", "state_null")}), flush=True)
    return 0 if summary["basic_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
