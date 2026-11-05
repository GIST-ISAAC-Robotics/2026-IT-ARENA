#!/usr/bin/env python3
"""고립 직각 코너 변형(대표 가정). gen_world.py의 단면·시설·검사를 그대로 쓰고 꼭짓점만 바꾼다.

출발 직선 끝의 첫 코너가 헤어핀이 아닌 고립 왼쪽 90°가 되게 한다. 모든 직선은 2.5 m 이상.
사용: python3 gen_world_iso.py <공식 월드 디렉터리> <출력 디렉터리>
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gen_world  # noqa: E402

gen_world.VERTICES = [
    ("V1", 6.5, 0.0),      # 고립 왼쪽 90° (출발 직선 끝)
    ("V2", 6.5, 2.5),      # 고립 왼쪽 90°
    ("V3", 4.0, 2.5),      # 고립 오른쪽 90°
    ("V4", 4.0, 5.0),      # 고립 왼쪽 90°
    ("V7", -4.4665, 5.0),  # 고립 왼쪽 90°
    ("V0", -4.4665, 0.0),  # 출발 직선 진입 왼쪽 90°
]
gen_world.CORNER_ROLE = {"V1": "isolated_left_first", "V2": "isolated_left", "V3": "isolated_right",
                         "V4": "isolated_left", "V7": "isolated_left", "V0": "isolated_left_before_start"}
gen_world.__file__ = __file__  # provenance의 생성기 해시는 이 변형 파일 기준(공용 함수는 gen_world.py)

if __name__ == "__main__":
    raise SystemExit(gen_world.main())
