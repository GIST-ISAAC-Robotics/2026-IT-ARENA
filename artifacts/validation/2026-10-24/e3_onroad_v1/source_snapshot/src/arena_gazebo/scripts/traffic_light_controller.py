#!/usr/bin/env python3
"""실험 신호등의 실제 렌즈 재질을 바꾸는 시뮬레이터 전용 제어기."""
from concurrent.futures import ThreadPoolExecutor
import json
import subprocess

from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from arena_vehicle_interface.node_lifecycle import run_node


COLORS = {"red": (1.0, .015, .005), "yellow": (1.0, .65, .005), "green": (.005, 1.0, .04)}


def set_lamps(state):
    """셸 문자열 대신 인수 배열을 사용하고, 적용 완료 응답을 확인합니다."""
    # 먼저 꺼질 렌즈를 처리하여 빨강·초록이 동시에 켜지는 중간 상태를 피합니다.
    order = [name for name in COLORS if name != state] + [state]
    for name in order:
        on = name == state
        color = COLORS[name]
        diffuse = color if on else tuple(component * .08 for component in color)
        emissive = color if on else (0.0, 0.0, 0.0)
        def rgba(values):
            return f"r: {values[0]} g: {values[1]} b: {values[2]} a: 1"
        request = (f'name: "lamp_{name}_vis" parent_name: "lamp_{name}" '
                   f'material {{ ambient {{ {rgba(diffuse)} }} diffuse {{ {rgba(diffuse)} }} '
                   f'emissive {{ {rgba(emissive)} }} specular {{ r: 0 g: 0 b: 0 a: 1 }} }}')
        result = subprocess.run([
            "gz", "service", "-s", "/world/it_arena_track/visual_config/blocking",
            "--reqtype", "gz.msgs.Visual", "--reptype", "gz.msgs.Boolean",
            "--timeout", "3000", "--req", request], capture_output=True, text=True, timeout=5)
        if result.returncode != 0 or "data: true" not in result.stdout:
            raise RuntimeError(f"{name} 렌즈 적용 실패: {result.stdout} {result.stderr}")
    return state


class TrafficLight(Node):
    def __init__(self):
        super().__init__("traffic_light_controller")
        self.red_duration = float(self.declare_parameter("red_duration_s", 8.0).value)
        self.yellow_duration = float(self.declare_parameter("yellow_duration_s", 2.0).value)
        if self.red_duration < 1 or self.yellow_duration < 0:
            raise ValueError("빨강은 1초 이상, 노랑은 0초 이상이어야 합니다.")
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.future = None
        self.desired = "red"
        self.applied = "unknown"
        self.error = ""
        self.sequence = True
        self.phase_started = None
        self.last_rgb = None
        self.publisher = self.create_publisher(String, "/sim/traffic_light/state", 10)
        self.create_subscription(Image, "/camera/color/image_raw", self.image_ready, QoSProfile(depth=1))
        self.create_service(Trigger, "/sim/traffic_light/reset", self.reset)
        for state in COLORS:
            self.create_service(Trigger, f"/sim/traffic_light/set_{state}", self.command_callback(state))
        self.create_timer(.1, self.tick)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def image_ready(self, message):
        self.last_rgb = float(message.header.stamp.sec) + message.header.stamp.nanosec * 1e-9

    def reset(self, request, response):
        self.desired, self.sequence, self.phase_started, self.error = "red", True, None, ""
        response.success, response.message = True, "빨강→노랑→초록 출발 순서를 다시 요청했습니다. 적용 상태는 /sim/traffic_light/state에서 확인합니다."
        return response

    def command_callback(self, state):
        def callback(request, response):
            self.desired, self.sequence, self.phase_started, self.error = state, False, None, ""
            response.success, response.message = True, f"{state} 적용을 요청했습니다. 적용 상태는 /sim/traffic_light/state에서 확인합니다."
            return response
        return callback

    def tick(self):
        now = self.now_s()
        if self.phase_started is not None and now < self.phase_started:
            self.reset(None, Trigger.Response())
        if self.future is not None and self.future.done():
            try:
                self.applied = self.future.result()
                self.phase_started = None if self.applied == "red" else now
                self.get_logger().info(f"실제 신호등 렌즈: {self.applied}")
            except Exception as error:
                self.error, self.sequence = str(error), False
                self.get_logger().error(self.error)
            self.future = None
        if self.future is None and self.desired != self.applied and not self.error:
            self.future = self.pool.submit(set_lamps, self.desired)
        rgb_ready = self.last_rgb is not None and 0 <= now - self.last_rgb < 1.0
        if self.sequence and self.future is None and rgb_ready:
            if self.phase_started is None:
                self.phase_started = now
            elapsed = now - self.phase_started
            if self.applied == "red" and elapsed >= self.red_duration:
                self.desired = "yellow"
            elif self.applied == "yellow" and elapsed >= self.yellow_duration:
                self.desired = "green"
        self.publisher.publish(String(data=json.dumps({"requested": self.desired, "applied": self.applied,
                                                      "sequence": self.sequence, "error": self.error,
                                                      "sim_time_s": now}, ensure_ascii=False)))

    def destroy_node(self):
        self.pool.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()


def main():
    run_node(TrafficLight)


if __name__ == "__main__":
    main()
