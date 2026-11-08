"""새 공식 v2026.11.06 코스 단독 1랩 이름 있는 프리셋. ROS Jazzy/작업공간을 source한 WSL에서 실행.

인자 조합은 scripts/run_current_solo.py에 위임한다. 그 파일은 과거 시험의 소스 해시 계보에
포함되어 있으므로 고치지 않고, 이 파일은 선택 옵션 묶음만 이름으로 고정한다.
"""
import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "scripts/run_current_solo.py"
# 2026-11-08 r25 단독 거리 한 랩(46.67 m·29.228초)의 등록 인자. 소스·기록기·물성 동일성의 근거는 아니다.
R25_REGISTERED = ROOT / "artifacts/validation/2026-11-08/solo_gap_envelope_connected_v3/registered.json"
PRESETS = {
    "new-course-solo": [
        "--track", "official_v2026_11_06",
        "--local-sensor-config", "src/arena_description/config/b0390_c1_jr60p20_sim.yaml",
        "--straight-budget", "--corner-approach", "--oblique-gap",
        "--piecewise-wall", "--standing-wall-reacquire",
    ],
}


def preset_args(name, output):
    """run_current_solo.py --print-args로 최종 검증기 인자를 받는다."""
    command = [sys.executable, "-B", str(BASE), str(output), "--print-args", *PRESETS[name]]
    return json.loads(subprocess.run(command, cwd=ROOT, check=True, capture_output=True, text=True).stdout)


def normalized(args):
    """경로 구분자·로컬 절대 센서 경로·결과 경로만 정상화해 옵션→값 표로 만든다."""
    head, rest = args[0].replace("\\", "/"), args[1:]
    table = {}
    i = 0
    while i < len(rest):
        key = rest[i]
        if not key.startswith("--"):
            raise ValueError(f"옵션 위치에 값이 있습니다: {key}")
        if key in table:
            raise ValueError(f"중복 옵션: {key}")
        value = None
        if i + 1 < len(rest) and not rest[i + 1].startswith("--"):
            value = rest[i + 1]
            i += 1
        if key in ("--output", "--local-sensor-config") and value is None:
            raise ValueError(f"값이 없는 경로 옵션: {key}")
        if key == "--output":
            value = "<output>"
        elif key == "--local-sensor-config":
            path = value.replace("\\", "/")
            marker = "/src/arena_description/"
            value = "src/arena_description/" + path.split(marker, 1)[1] if marker in path else path
        table[key] = value
        i += 1
    return head, table


def compare_with_r25(args):
    reference = json.loads(R25_REGISTERED.read_text(encoding="utf-8"))["args"]
    expected, actual = normalized(reference), normalized(args)
    if expected[0] != actual[0]:
        return [f"진입 스크립트: r25={expected[0]} 현재={actual[0]}"]
    keys = sorted(set(expected[1]) | set(actual[1]))
    return [f"{k}: r25={expected[1].get(k, '<없음>')} 현재={actual[1].get(k, '<없음>')}"
            for k in keys if expected[1].get(k, "<없음>") != actual[1].get(k, "<없음>")]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="새 결과 디렉터리(기존 경로 거절)")
    parser.add_argument("--preset", choices=tuple(PRESETS), required=True, help="이름 있는 실행 조합")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--print-args", action="store_true", help="실행 없이 최종 인자 출력")
    mode.add_argument("--check-r25", action="store_true",
                      help="실행 없이 r25 등록 인자와 옵션·값을 대조(경로만 정상화)")
    opts = parser.parse_args()
    # 호출 위치 기준으로 한 번만 절대화해 검사·출력·실행에 같은 경로를 쓴다(하위 실행은 cwd=ROOT).
    output = opts.output.resolve()
    if output.exists():
        parser.error(f"이미 있는 결과 경로입니다: {output}")
    args = preset_args(opts.preset, output)
    if opts.check_r25:
        diff = compare_with_r25(args)
        print(json.dumps({"preset": opts.preset, "reference": str(R25_REGISTERED.relative_to(ROOT)).replace("\\", "/"),
                          "options": len(normalized(args)[1]), "differences": diff}, ensure_ascii=False, indent=2))
        return 1 if diff else 0
    if opts.print_args:
        print(json.dumps(args))
        return 0
    # 출력·대조한 최종 인자를 그대로 실행한다. 기존 결과 거절·STOP·종료는 기존 검증기가 맡는다.
    return subprocess.call([sys.executable, "-B", *args], cwd=ROOT)


if __name__ == "__main__":
    raise SystemExit(main())
