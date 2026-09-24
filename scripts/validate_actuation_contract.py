#!/usr/bin/env python3
"""보드 없는 A1 계약 시험. 장난감 종방향 모형이며 Gazebo/실차 성능 시험이 아니다."""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import math
import multiprocessing as mp
from pathlib import Path
import statistics
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/arena_vehicle_interface"))
from arena_vehicle_interface.actuation_contract import ActuationReceiver, CommandProducer, State
from arena_vehicle_interface.actuation_wire import Decoder, encode
from arena_vehicle_interface.mcu_speed_loop import SpeedPI, MotorIntent

CASES = ["normal", "host_loss", "duplicate_replay", "delayed_packet", "encoder_loss",
         "estop_release", "mcu_reboot", "host_reboot", "clear_without_arm", "explicit_rearm"]


def round_trip(message):
    result = Decoder().feed(encode(message))
    assert len(result) == 1
    return result[0]


class Bench:
    def __init__(self):
        self.receiver = ActuationReceiver()
        self.pi = SpeedPI()
        self.velocity = self.position = 0.
        self.intent = MotorIntent(0., True)
        self.last_us = 0
        self.last_feedback_us = -10_000
        self.feedback_seq = 0

    def step(self, now_us, *, feedback=True):
        dt = (now_us - self.last_us) * 1e-6
        if dt > 0:
            if self.intent.brake_requested:
                self.velocity = max(0., self.velocity - 2. * dt)
            else:
                self.velocity = max(0., self.velocity + (3. * self.intent.effort - .3 * self.velocity) * dt)
            self.position += self.velocity * dt
        self.last_us = now_us
        if feedback and now_us - self.last_feedback_us >= 10_000:
            self.feedback_seq += 1
            c = self.receiver.config
            ticks = round(self.position / (math.tau * c.wheel_radius_m) * c.ticks_per_rev)
            self.receiver.update_encoder(now_us, sequence=self.feedback_seq, capture_us=now_us,
                                         left_ticks=ticks, right_ticks=ticks)
            self.last_feedback_us = now_us
        status = self.receiver.tick(now_us)
        self.intent = self.pi.update(self.receiver.target_speed, self.receiver.left_mps,
                                     self.receiver.right_mps, dt, enabled=self.receiver.armed)
        return {"time_us": now_us, **status, "model_speed_mps": self.velocity,
                "model_position_m": self.position, "normalized_effort": self.intent.effort,
                "brake_requested": self.intent.brake_requested}

    def reboot(self):
        self.receiver = ActuationReceiver()
        self.pi = SpeedPI()
        self.intent = MotorIntent(0., True)
        # 물리 속도는 0으로 지우지 않는다. 재부팅 뒤에도 관성으로 움직일 수 있다.
        self.feedback_seq = 0

    def send(self, packet, now_us):
        return self.receiver.receive(round_trip(packet), now_us)

    def offer(self, now_us):
        return round_trip(self.receiver.offer(now_us))


def deterministic_case(name):
    b, h = Bench(), CommandProducer()
    trace, events, retained = [], [], None
    for now in range(0, 6_000_001, 5_000):
        if now == 3_000_000:
            if name in ("estop_release", "clear_without_arm", "explicit_rearm"):
                b.receiver.set_estop(True, now)
            elif name == "mcu_reboot":
                b.reboot()
            elif name == "host_reboot":
                h = CommandProducer()
        if now == 3_200_000 and name in ("estop_release", "clear_without_arm", "explicit_rearm"):
            b.receiver.set_estop(False, now)
        row = b.step(now, feedback=not (name == "encoder_loss" and 3_000_000 <= now < 3_300_000))
        if now == 300_000:
            ok, reason = b.send(h.control(b.offer(now), "ARM"), now)
            events.append({"time_us": now, "kind": "ARM", "ok": ok, "reason": reason})
        if now == 4_500_000 and name in ("clear_without_arm", "explicit_rearm"):
            ok, reason = b.send(h.control(b.offer(now), "CLEAR"), now)
            events.append({"time_us": now, "kind": "CLEAR", "ok": ok, "reason": reason})
        if now == 4_520_000 and name == "explicit_rearm":
            ok, reason = b.send(h.control(b.offer(now), "ARM"), now)
            events.append({"time_us": now, "kind": "ARM", "ok": ok, "reason": reason})
        if now >= 300_000 and now % 20_000 == 0:
            after_fault = now >= 3_000_000 and name != "normal"
            if after_fault and name == "host_loss":
                pass
            elif after_fault and name == "duplicate_replay":
                ok, reason = b.send(retained, now)
                events.append({"time_us": now, "kind": "REPLAY", "ok": ok, "reason": reason})
            elif after_fault and name == "delayed_packet":
                if now == 3_000_000:
                    retained = h.drive(b.offer(now), speed_mps=1.2, steering_rad=.1,
                                       source_us=now + 9_000_000, host_now_us=now + 9_000_000)
                elif now == 3_160_000:
                    ok, reason = b.send(retained, now)
                    events.append({"time_us": now, "kind": "DELAYED", "ok": ok, "reason": reason})
            else:
                try:
                    speed = 0. if name == "normal" and now >= 4_000_000 else 1.2
                    retained = h.drive(b.offer(now), speed_mps=speed, steering_rad=.1,
                                       source_us=now + 9_000_000, host_now_us=now + 9_000_000)
                    ok, reason = b.send(retained, now)
                    if not ok:
                        events.append({"time_us": now, "kind": "DRIVE", "ok": ok, "reason": reason})
                except ValueError:
                    pass  # 호스트 자동 ARM 없음. 수신기 상태는 trace로 별도 판정한다.
        # 명령 적용 직후의 상태와 다음 물리 갱신 전 속도를 함께 보존한다.
        row.update(b.receiver.status())
        trace.append(row)
    moving = [r for r in trace if 2_500_000 <= r["time_us"] < 3_000_000]
    fault_trace = [r for r in trace if 3_250_000 <= r["time_us"] < 4_500_000]
    post_trace = [r for r in trace if r["time_us"] >= 4_600_000]
    checks = {"started_before_injection": max(r["model_speed_mps"] for r in moving) > .8,
              "no_drive_before_arm": all(r["target_speed_mps"] == 0 for r in trace if r["time_us"] < 300_000),
              "positive_target_requires_armed": all(r["target_speed_mps"] == 0 or
                   r["state"] == State.ACTIVE.value for r in trace),
              "operator_controls_accepted": all(e["ok"] for e in events if e["kind"] in ("ARM", "CLEAR"))}
    if name == "normal":
        error = statistics.mean(abs(r["model_speed_mps"] - 1.2) for r in moving)
        checks.update(tracking_mean_error_below_015=error < .15,
                      no_fault=all(r["state"] not in (State.FAULT.value, State.ESTOP.value) for r in trace),
                      model_stopped=trace[-1]["model_speed_mps"] < .01)
    else:
        checks.update(zero_target_after_fault=all(r["target_speed_mps"] == 0 for r in fault_trace),
                      model_stopped_before_recovery=next(r for r in trace if r["time_us"] == 4_450_000)["model_speed_mps"] < .01)
        if name == "explicit_rearm":
            checks["resumes_only_after_explicit_arm"] = max(r["model_speed_mps"] for r in post_trace) > .8
        else:
            checks["no_automatic_restart"] = all(r["target_speed_mps"] == 0 for r in post_trace)
        if name in ("duplicate_replay", "delayed_packet"):
            checks["old_packets_rejected"] = all(not e["ok"] for e in events if e["kind"] in ("REPLAY", "DELAYED"))
    return {"case": name, "passed": all(checks.values()), "checks": checks,
            "peak_model_speed_mps": max(r["model_speed_mps"] for r in trace),
            "final_state": trace[-1]["state"], "events": events, "trace": trace}


def worker(connection):
    """실제 시간으로 도는 별도 가상 MCU 프로세스. 네트워크/실제 포트를 열지 않는다."""
    b, start = Bench(), time.monotonic_ns()
    try:
        while True:
            now = (time.monotonic_ns() - start) // 1000
            row = b.step(now)
            if connection.poll():
                raw = connection.recv_bytes(65536)
                messages = Decoder().feed(raw)
                if len(messages) != 1:
                    raise ValueError("bench transport frame")
                msg = messages[0]
                if msg.get("kind") == "BENCH_EXIT":
                    b.receiver.receive({"v": 1, "kind": "STOP"}, now)
                    connection.send_bytes(encode({"exit": True, "state": b.receiver.state.value}))
                    break
                if msg.get("kind") == "BENCH_POLL":
                    response = {"offer": b.offer(now), "model_speed_mps": row["model_speed_mps"]}
                else:
                    ok, reason = b.send(msg, now)
                    response = {"ok": ok, "reason": reason, "state": b.receiver.state.value}
                connection.send_bytes(encode(response))
            time.sleep(.002)
    finally:
        connection.close()


def process_case():
    context = mp.get_context("spawn")
    parent, child = context.Pipe()
    proc = context.Process(target=worker, args=(child,), name="arena-virtual-mcu")
    proc.start()
    child.close()
    observations = []

    def exchange(message):
        parent.send_bytes(encode(message))
        if not parent.poll(2.):
            raise TimeoutError("virtual MCU reply missing")
        result = Decoder().feed(parent.recv_bytes(65536))
        if len(result) != 1:
            raise ValueError("bad MCU response")
        return result[0]

    result = {"case": "independent_process_sender_silence", "passed": False}
    try:
        h = CommandProducer()
        # 시작 시퀀스는 시험기만 수행한다. 실제 호스트 자동 ARM 정책이 아니다.
        ready_limit = time.monotonic() + 3.
        while True:
            status = exchange({"kind": "BENCH_POLL"})["offer"]
            if status["lease_issued_us"] >= 350_000 and status["feedback_valid"]:
                break
            if time.monotonic() > ready_limit:
                raise TimeoutError("encoder readiness")
            time.sleep(.025)
        arm = exchange(h.control(status, "ARM"))
        result["arm"] = arm
        end = time.monotonic() + 2.
        while time.monotonic() < end:
            state = exchange({"kind": "BENCH_POLL"})
            observations.append(state)
            now = time.monotonic_ns() // 1000
            packet = h.drive(state["offer"], speed_mps=1.2, steering_rad=.1,
                             source_us=now, host_now_us=now)
            ack = exchange(packet)
            if not ack["ok"]:
                raise RuntimeError(f"drive rejected: {ack}")
            time.sleep(.015)
        before = exchange({"kind": "BENCH_POLL"})
        # 제어 송신측이 아무것도 보내지 않는 동안 MCU 프로세스는 계속 돌린다.
        time.sleep(.3)
        after = exchange({"kind": "BENCH_POLL"})
        replay = exchange(packet)
        time.sleep(.9)
        final = exchange({"kind": "BENCH_POLL"})
        checks = {"moving_before_silence": before["model_speed_mps"] > .8,
                  "receiver_latched_on_own_clock": after["offer"]["state"] == State.FAULT.value,
                  "command_timeout_reason": after["offer"]["reason"] == "command_expired",
                  "zero_target": after["offer"]["target_speed_mps"] == 0,
                  "old_packet_rejected": not replay["ok"],
                  "model_stopped": final["model_speed_mps"] < .01,
                  "no_auto_restart": final["offer"]["state"] == State.FAULT.value}
        result.update(checks=checks, passed=all(checks.values()), before=before, after=after,
                      replay=replay, final=final, observations=observations)
    except Exception as error:
        result["error"] = repr(error)
        result["observations"] = observations
    finally:
        try:
            result["exit_ack"] = exchange({"kind": "BENCH_EXIT"})
        except (EOFError, OSError, TimeoutError, ValueError):
            pass
        proc.join(3.)
        result["forced_termination"] = proc.is_alive()
        if proc.is_alive():
            proc.terminate()
            proc.join(3.)
        result["exitcode"] = proc.exitcode
        result["residual_process"] = proc.is_alive()
        parent.close()
    result["passed"] = (result["passed"] and result["exitcode"] == 0 and
                        not result["forced_termination"] and not result["residual_process"])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--skip-process", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    report = {"created_utc": datetime.now(timezone.utc).isoformat(), "scope": "PC reference protocol and toy motor only",
              "config": asdict(ActuationReceiver().config), "results": [], "source_sha256": {}}
    paths = [Path(__file__), ROOT / "tests/test_actuation_contract.py"]
    paths += [ROOT / "src/arena_vehicle_interface/arena_vehicle_interface" / p for p in
              ("actuation_contract.py", "actuation_wire.py", "mcu_speed_loop.py")]
    for path in paths:
        report["source_sha256"][str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
    for name in CASES:
        item = deterministic_case(name)
        (args.output / f"{name}.json").write_text(json.dumps(item, ensure_ascii=False, indent=2), encoding="utf-8")
        report["results"].append({key: value for key, value in item.items() if key not in ("events", "trace")})
    if not args.skip_process:
        try:
            item = process_case()
        except (OSError, RuntimeError) as error:
            item = {"case": "independent_process_sender_silence", "passed": False,
                    "error": repr(error), "phase": "process_initialization"}
        (args.output / "independent_process.json").write_text(json.dumps(item, ensure_ascii=False, indent=2), encoding="utf-8")
        report["results"].append({key: value for key, value in item.items() if key != "observations"})
    report["passed"] = all(item["passed"] for item in report["results"])
    (args.output / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"passed": report["passed"], "cases": len(report["results"]),
                      "failed": [i["case"] for i in report["results"] if not i["passed"]]}, ensure_ascii=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
