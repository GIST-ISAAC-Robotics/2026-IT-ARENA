#!/usr/bin/env python3
"""격리된 제조사 C1 노드의 LaserScan을 기록하고 정상 종료한다.

차량 제어 노드/토픽은 사용하지 않는다. ROS_LOCALHOST_ONLY와 별도
ROS_DOMAIN_ID를 외부에서 지정한다. 원시 실내 스캔은 Git 제외 경로에 둔다.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import time


def distribution(values):
    ordered = sorted(values)
    if not ordered:
        return None
    def percentile(fraction):
        position = (len(ordered)-1)*fraction
        lo, hi = math.floor(position), math.ceil(position)
        return ordered[lo] + (ordered[hi]-ordered[lo])*(position-lo)
    return dict(count=len(ordered), min=ordered[0], median=statistics.median(ordered),
                p95=percentile(.95), p99=percentile(.99), max=ordered[-1])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("driver", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--duration", type=float, default=300)
    parser.add_argument("--warmup", type=float, default=5)
    parser.add_argument("--compensate", choices=["true", "false"], default="true")
    args = parser.parse_args()
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1" or not os.environ.get("ROS_DOMAIN_ID"):
        parser.error("Set ROS_LOCALHOST_ONLY=1 and an isolated ROS_DOMAIN_ID")
    args.output.mkdir(parents=True, exist_ok=False)
    import numpy as np
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan

    command = [str(args.driver), "--ros-args", "-r", "__ns:=/c1_probe",
               "-p", "serial_port:=/dev/ttyUSB0", "-p", "serial_baudrate:=460800",
               "-p", "scan_mode:=Standard", "-p", "inverted:=false",
               "-p", "frame_id:=laser", "-p", "angle_compensate:="+args.compensate,
               "-p", "scan_frequency:=10.0"]
    rows, errors = [], []
    first_rx = None
    measure_start = None
    warmup_messages = 0
    body_hashes = []
    begin = time.monotonic()
    last_progress = begin
    rclpy.init()
    node = rclpy.create_node("c1_recording_probe")
    raw_file = (args.output/"scans.jsonl").open("x")

    def on_scan(message):
        nonlocal first_rx, measure_start, warmup_messages
        now = time.monotonic_ns()
        if first_rx is None:
            first_rx = now
        stamp = message.header.stamp.sec*10**9 + message.header.stamp.nanosec
        ranges = np.asarray(message.ranges, dtype=np.float32)
        finite = np.isfinite(ranges)
        valid = finite & (ranges >= message.range_min) & (ranges <= message.range_max)
        measured = (now-first_rx)/1e9 >= args.warmup
        row = dict(host_monotonic_ns=now, host_wall_ns=time.time_ns(), stamp_ns=stamp,
                   frame_id=message.header.frame_id, count=len(ranges),
                   angle_min=float(message.angle_min), angle_max=float(message.angle_max),
                   angle_increment=float(message.angle_increment),
                   scan_time=float(message.scan_time), time_increment=float(message.time_increment),
                   range_min=float(message.range_min), range_max=float(message.range_max),
                   intensities_count=len(message.intensities),
                   positive_inf=int(np.isposinf(ranges).sum()), nan=int(np.isnan(ranges).sum()),
                   negative_inf=int(np.isneginf(ranges).sum()),
                   finite_outside_range=int((finite & ~valid).sum()), valid=int(valid.sum()),
                   valid_range_min=float(ranges[valid].min()) if valid.any() else None,
                   valid_range_max=float(ranges[valid].max()) if valid.any() else None,
                   measured=measured)
        body_hash = hashlib.sha256(ranges.tobytes()+np.asarray(message.intensities,dtype=np.float32).tobytes()).hexdigest()
        raw_file.write(json.dumps(dict(row, ranges=ranges.tolist(),
                                       intensities=list(message.intensities),
                                       body_sha256=body_hash))+"\n")
        if measured:
            if measure_start is None:
                measure_start=now
            rows.append(row)
            body_hashes.append(body_hash)
        else:
            warmup_messages += 1

    node.create_subscription(LaserScan,"/c1_probe/scan",on_scan,
                             QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE))
    driver_log=(args.output/"driver.log").open("xb")
    process=subprocess.Popen(command,stdout=driver_log,stderr=subprocess.STDOUT)
    forced_kill=False
    try:
        while True:
            rclpy.spin_once(node,timeout_sec=.2)
            now=time.monotonic()
            if process.poll() is not None:
                errors.append("Driver exited before recording completed")
                break
            if first_rx is None and now-begin>25:
                errors.append("No first LaserScan within 25 seconds")
                break
            if rows and (time.monotonic_ns()-rows[-1]["host_monotonic_ns"])/1e9>2:
                errors.append("No LaserScan for more than 2 seconds")
                break
            if measure_start and (rows[-1]["host_monotonic_ns"]-measure_start)/1e9>=args.duration:
                break
            if now-last_progress>=10:
                print(json.dumps(dict(measured_scans=len(rows),warmup_scans=warmup_messages,
                    elapsed_s=(rows[-1]["host_monotonic_ns"]-measure_start)/1e9 if rows else None)),flush=True)
                last_progress=now
    except BaseException as error:
        errors.append(repr(error))
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            forced_kill=True
            process.kill()
            process.wait(timeout=5)
        driver_log.close()
        raw_file.close()
        node.destroy_node()
        rclpy.shutdown()

    span=(rows[-1]["host_monotonic_ns"]-rows[0]["host_monotonic_ns"])/1e9 if len(rows)>1 else 0
    intervals=[(b["host_monotonic_ns"]-a["host_monotonic_ns"])/1e6 for a,b in zip(rows,rows[1:])]
    stamps=[(b["stamp_ns"]-a["stamp_ns"])/1e6 for a,b in zip(rows,rows[1:])]
    log=(args.output/"driver.log").read_text(errors="replace")
    parameter_names=("frame_id","count","angle_min","angle_max","angle_increment",
                     "range_min","range_max","intensities_count")
    summary=dict(schema=1,command=command,requested_duration_s=args.duration,warmup_s=args.warmup,
                 measured_scans=len(rows),warmup_scans=warmup_messages,host_span_s=span,
                 scan_hz=(len(rows)-1)/span if span else None,
                 host_interval_ms=distribution(intervals),stamp_interval_ms=distribution(stamps),
                 scan_time_s=distribution([r["scan_time"] for r in rows]),
                 time_increment_s=distribution([r["time_increment"] for r in rows]),
                 header_to_rx_ms=distribution([(r["host_wall_ns"]-r["stamp_ns"])/1e6 for r in rows]),
                 fields={k:dict(Counter(str(r[k]) for r in rows)) for k in parameter_names},
                 valid_points=distribution([r["valid"] for r in rows]),
                 inf_points=distribution([r["positive_inf"] for r in rows]),
                 nan_total=sum(r["nan"] for r in rows),negative_inf_total=sum(r["negative_inf"] for r in rows),
                 finite_outside_range_total=sum(r["finite_outside_range"] for r in rows),
                 gaps_gt_150ms=sum(x>150 for x in intervals),gaps_gt_200ms=sum(x>200 for x in intervals),
                 stamp_nonpositive=sum(x<=0 for x in stamps),
                 consecutive_identical_body=sum(a==b for a,b in zip(body_hashes,body_hashes[1:])),
                 errors=errors,driver_returncode=process.returncode,forced_kill=forced_kill,
                 stop_motor_logged="Stop motor" in log,driver_error_lines=[line for line in log.splitlines() if "[ERROR]" in line],
                 driver_warning_lines=[line for line in log.splitlines() if "[WARN]" in line],
                 limits=["Header-to-rx is not verified acquisition latency",
                         "Uniform ROS angles/time_increment do not establish original acquisition order/time",
                         "No controlled target/physical yaw calibration or vehicle motion test"])
    summary["basic_pass"]=bool(span>=args.duration and summary["scan_hz"]>=9.5 and
        not errors and not forced_kill and process.returncode==0 and summary["stop_motor_logged"] and
        not summary["driver_error_lines"] and summary["stamp_nonpositive"]==0)
    (args.output/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    print(json.dumps({k:summary[k] for k in ("basic_pass","measured_scans","host_span_s","scan_hz","errors","driver_returncode","stop_motor_logged")}),flush=True)
    return 0 if summary["basic_pass"] else 1


if __name__=="__main__":
    raise SystemExit(main())
