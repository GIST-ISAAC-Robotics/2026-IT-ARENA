"""CAD 확정 전 JR60P20 단독 1랩 진입점. ROS Jazzy/작업공간을 source한 WSL에서 실행."""
import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
# 2026-11-04 사용자 선택 임시 차량: 센서 +60 mm·앞 지주 +20 mm·뒤 지주 바깥 Ø20 mm 배선 예약 외곽.
SENSOR = ROOT / "src/arena_description/config/b0183_c1_jr60p20_sim.yaml"
# 이전 기준(얇은 2 mm 합성 배선). 11/4 JR50 기록 재현용으로만 선택한다.
PREVIOUS_SENSOR = ROOT / "src/arena_description/config/b0183_c1_jr50_sim.yaml"
# 50 mm ID49 판 아래끝 115 mm가 75 mm 스캔면과 배선 예약 표시 상단 105 mm보다 높다.
MARKER_Z = "0.140"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="새 결과 디렉터리")
    parser.add_argument("--print-args", action="store_true", help="실행 없이 최종 인자 출력")
    parser.add_argument("--previous-jr50", action="store_true", help="이전 JR50 프리셋 재현(신규 개발 기준 아님)")
    parser.add_argument("--straight-budget", action="store_true",
                        help="기본 OFF 단독 직선 속도 후보(현재 양쪽 벽 관측 길이로만 증속) 선택")
    parser.add_argument('--corner-approach', action='store_true', help='단독 증속 프로필의 직각 접근 감속 후보')
    parser.add_argument('--oblique-gap', action='store_true', help='기본 OFF 사선 좁은 공백 저속 시험')
    parser.add_argument('--piecewise-wall', action='store_true', help='관측 직각 벽의 두 직선+원호 시험 후보')
    parser.add_argument('--standing-wall-reacquire', action='store_true',
                        help='저속에서 현재 긴 선호 벽을 재획득하는 기존 후보를 명시 선택')
    parser.add_argument("--track", choices=("official", "official_v2026_11_06"), default="official",
                        help="새 코스는 official_v2026_11_06 명시. 생략 시 이전 코스 재현")
    parser.add_argument("--local-sensor-config", type=Path,
                        help="센서 프로필 명시 선택. B0390 신규 개발 구성은 b0390_c1_jr60p20_sim.yaml")
    opts = parser.parse_args()
    if opts.previous_jr50 and opts.local_sensor_config is not None:
        parser.error("--previous-jr50과 --local-sensor-config는 함께 선택할 수 없습니다.")
    sensor = PREVIOUS_SENSOR if opts.previous_jr50 else SENSOR
    if opts.local_sensor_config is not None:
        sensor = opts.local_sensor_config
    args = [
        "scripts/validate_basic_autonomy.py", "--autonomy-mode", "local_pursuit",
        "--g1-follow", "--g1-boundary-evidence", "corner_distance_candidate",
        "--g1-occupancy-timing", "per_ray_candidate", "--record-follow-decisions",
        "--g1-corridor-fallback", "--g1-race-path", "--partial-observation-policy", "bounded_local_v4",
        "--speed-profile", "g1_open_10kmh_solo_gap05_arc_preview_bridge", "--disable-speed-bump",
        "--lidar-rate-hz", "10", "--lidar-acquisition", "sequential", "--lidar-compensation", "both",
        "--actuation-mode", "virtual_mcu", "--control-rate-hz", "50", "--render-backend", "wsl_nvidia",
        "--shutdown-mode", "service", "--grid-slot", "3", "--laps", "1", "--max-sim-seconds", "110",
        "--wall-timeout-s", "1200", "--stop-wall-timeout-s", "90", "--video", "--red-duration-s", "1.6",
        "--local-sensor-config", str(sensor.resolve(strict=True)), "--output", str(opts.output.resolve()),
        "--pilot-marker-z", MARKER_Z,
    ]
    if opts.straight_budget:
        args.append("--g1-solo-straight-budget")
    if opts.corner_approach:
        if not opts.straight_budget: parser.error('--corner-approach requires --straight-budget')
        args.append('--g1-solo-corner-approach')
    if opts.oblique_gap:
        args.append('--g1-oblique-gap')
    if opts.piecewise_wall:
        args.append('--g1-piecewise-wall')
    if opts.standing_wall_reacquire:
        args.append('--g1-standing-wall-reacquire')
    if opts.track != "official":
        args.extend(["--track", opts.track])
    if opts.print_args:
        print(json.dumps(args))
        return 0
    # 기존 결과 디렉터리 거절·로그·STOP·프로세스 종료는 기존 검증기가 맡는다.
    return subprocess.call([sys.executable, "-B", *args], cwd=ROOT)


if __name__ == "__main__":
    raise SystemExit(main())
