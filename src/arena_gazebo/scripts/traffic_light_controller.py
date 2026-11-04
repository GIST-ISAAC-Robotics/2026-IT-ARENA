#!/usr/bin/env python3
"""공식 2×5 신호의 광원 재질을 바꾸는 시뮬레이터 전용 제어기.

공식 SDF는 상시 발광 정적 재질이고 공식 traffic_light.py는 UDP 예제이므로 Gazebo 광원을
직접 전환합니다. 차량은 RGB만 보며 이 상태 토픽은 평가기에만 전달합니다.
"""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import selectors
import subprocess

from ament_index_python.packages import get_package_prefix
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from arena_vehicle_interface.node_lifecycle import run_node


GREEN_COLUMN = 4
LAMPS = tuple(dict(name=f"lamp_{'green' if column == GREEN_COLUMN else f'red_{column+1}'}_{row}", column=column)
              for column in range(GREEN_COLUMN+1) for row in ("top", "bottom"))
READY_IMAGES = 20
MANUAL_STAGES = {"red": "red4", "green": "green", "off": "off"}


def sequence_stages(red_hold_s):
    """시험 점등 뒤 네 빨강을 쌓고 빨강 소등과 함께 초록을 켭니다. 공식 무작위 유지시간은 고정값입니다."""
    return (("green_probe", 1.), ("off", 1.), ("red1", 1.), ("red2", 1.), ("red3", 1.),
            ("red4", float(red_hold_s)), ("green", None))


def lamp_lit(stage, column):
    red = int(stage[-1]) if stage.startswith("red") else 0
    return column < red or (column == GREEN_COLUMN and stage in ("green", "green_probe"))


def stage_label(stage):
    return "red" if stage.startswith("red") else stage


def lamp_request(name, column, lit):
    color = ("0 1 0" if column == GREEN_COLUMN else "1 0 0") if lit else ".004 .004 .004"
    red, green, blue = color.split()
    rgba = f"r:{red} g:{green} b:{blue} a:1"
    glow = rgba if lit else "r:0 g:0 b:0 a:1"
    return (f'name:"{name}_vis" parent_name:"{name}" material {{ambient {{{rgba}}} diffuse {{{rgba}}} '
            f'emissive {{{glow}}} specular {{r:0 g:0 b:0 a:1}}}}')


class LampClient:
    """지속 연결 한 개로 광원을 끄는 쪽부터 적용해 빨강·초록이 함께 켜진 표본을 피합니다."""

    def __init__(self, executable, service):
        self.process = subprocess.Popen([executable, service], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        text=True, bufsize=1)

    def apply(self, stage):
        records = []
        for lamp in sorted(LAMPS, key=lambda item: lamp_lit(stage, item["column"])):
            lit = lamp_lit(stage, lamp["column"])
            self.process.stdin.write(lamp_request(lamp["name"], lamp["column"], lit)+"\n")
            self.process.stdin.flush()
            with selectors.DefaultSelector() as poll:
                poll.register(self.process.stdout, selectors.EVENT_READ)
                if not poll.select(5):
                    raise TimeoutError("광원 재질 응답 없음")
            line = self.process.stdout.readline()
            if not line:
                raise RuntimeError("광원 재질 클라이언트가 회신 없이 종료됨")
            receipt = json.loads(line)
            records.append(dict(lamp=lamp["name"], lit=lit, **receipt))
            if not all(receipt.get(key) is True for key in ("parsed", "executed", "service_result", "applied")):
                raise RuntimeError(f"{lamp['name']} 재질 적용 실패: {receipt}")
        return records

    def close(self):
        self.process.stdin.close()
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            self.process.wait(timeout=3)


class TrafficLight(Node):
    def __init__(self):
        super().__init__("traffic_light_controller")
        self.red_hold = float(self.declare_parameter("red_duration_s", 8.0).value)
        if self.red_hold < 1:
            raise ValueError("마지막 빨강 유지 시간은 1초 이상이어야 합니다.")
        self.require_armed = bool(self.declare_parameter("require_armed", False).value)
        world = str(self.declare_parameter("world_name", "it_arena_track").value)
        audit = str(self.declare_parameter("audit_path", "").value)
        self.audit = Path(audit).open("x", encoding="utf-8") if audit else None
        executable = Path(get_package_prefix("arena_gazebo"))/"lib/arena_gazebo/visual_material_client"
        self.client = LampClient(str(executable), f"/world/{world}/visual_config/blocking")
        self.stages = sequence_stages(self.red_hold)
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.future = None
        self.pending = None
        self.applied = "off"
        self.error = ""
        self.sequence = True
        self.index = -1
        self.manual = None
        self.phase_started = None
        self.images = 0
        self.last_rgb = None
        self.armed = False
        self.publisher = self.create_publisher(String, "/sim/traffic_light/state", 10)
        self.create_subscription(Image, "/camera/color/image_raw", self.image_ready, QoSProfile(depth=1))
        if self.require_armed:
            self.create_subscription(String, "/actuation/status", self.actuation_status, 10)
        self.create_service(Trigger, "/sim/traffic_light/reset", self.reset)
        for state in MANUAL_STAGES:
            self.create_service(Trigger, f"/sim/traffic_light/set_{state}", self.command_callback(state))
        self.create_timer(.05, self.tick)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def image_ready(self, message):
        stamp = float(message.header.stamp.sec) + message.header.stamp.nanosec * 1e-9
        if self.last_rgb is None or stamp > self.last_rgb:
            self.images += 1
            self.last_rgb = stamp

    def actuation_status(self, message):
        self.armed = json.loads(message.data).get("state") == "ARMED"

    def reset(self, request, response):
        self.sequence, self.index, self.manual, self.phase_started, self.error = True, -1, None, None, ""
        response.success, response.message = True, "공식 점등 순서를 다시 요청했습니다. 적용 상태는 /sim/traffic_light/state에서 확인합니다."
        return response

    def command_callback(self, state):
        def callback(request, response):
            self.sequence, self.manual, self.phase_started, self.error = False, MANUAL_STAGES[state], None, ""
            response.success, response.message = True, f"{state} 적용을 요청했습니다. 적용 상태는 /sim/traffic_light/state에서 확인합니다."
            return response
        return callback

    def write_audit(self, row):
        if self.audit is not None:
            self.audit.write(json.dumps(row, ensure_ascii=False)+"\n")
            self.audit.flush()

    def submit(self, stage, now):
        self.pending = stage
        self.write_audit(dict(requested_stage=stage, requested_sim_s=now))
        self.future = self.pool.submit(self.client.apply, stage)

    def next_stage(self, now):
        if not self.sequence:
            return self.manual if self.manual is not None and self.manual != self.applied else None
        if self.index < 0:
            ready = self.images >= READY_IMAGES and (self.armed or not self.require_armed)
            return self.stages[0][0] if ready else None
        duration = self.stages[self.index][1]
        if self.phase_started is None or duration is None or now-self.phase_started < duration:
            return None
        return self.stages[self.index+1][0]

    def tick(self):
        now = self.now_s()
        if self.phase_started is not None and now < self.phase_started:
            self.reset(None, Trigger.Response())
        if self.future is not None and self.future.done():
            try:
                receipts = self.future.result()
            except Exception as error:
                self.error, self.sequence = f"{type(error).__name__}: {error}", False
                self.write_audit(dict(failed_stage=self.pending, failed_sim_s=now, error=self.error))
                self.get_logger().error(self.error)
            else:
                self.applied, self.phase_started = self.pending, now
                self.write_audit(dict(stage=self.applied, applied_sim_s=now, receipts=receipts))
                self.get_logger().info(f"실제 신호 광원: {self.applied}")
            self.future = None
        if self.future is None and not self.error:
            stage = self.next_stage(now)
            if stage is not None:
                self.index += int(self.sequence)
                self.submit(stage, now)
        self.publisher.publish(String(data=json.dumps({
            "requested": self.pending, "applied": stage_label(self.applied), "stage": self.applied,
            "sequence": self.sequence, "error": self.error, "sim_time_s": now,
            "scope": "evaluation-only material receipt; not controller input"}, ensure_ascii=False)))

    def destroy_node(self):
        self.pool.shutdown(wait=True, cancel_futures=True)
        self.client.close()
        if self.audit is not None:
            self.audit.close()
        return super().destroy_node()


def main():
    run_node(TrafficLight)


if __name__ == "__main__":
    main()
