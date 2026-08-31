**어떤 파일에서**

`track/design_final.json:features.start_line`, `track/track_gen.py:build_all_from_design()/write_dxf()`, `track/output_final/scene.json`, `venue_layout.dxf`, `preview.png`

**무엇이 이상한가요**

설계에 저장된 출발/결승선 s와 실제 신호등·ID 0·도면의 기준점이 약 0.386738 m 다릅니다. 생성기는 설계 start_line을 읽지만 출력에 사용하지 않습니다.

**환경**

- 재현 환경: Windows 호스트의 WSL Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Sim 8.11.0
- repository: `MOSW626/istech-it-arena`
- commit/release: `d61c5db9252cedfbc163cd044a47671df91e1660` / `v2026.08.31`
- Python 3로 JSON·CSV를 확인; 화면은 Gazebo Sim에서 원본 좌표·치수를 유지한 호환성 변환 월드로 관찰

## 재현 방법

1. `design_final.json.features.start_line.s`를 읽습니다.
2. 같은 파일의 `features.traffic_light.s`와 `features.aruco` 중 ID 0의 `s`를 비교합니다.
3. 생성기의 `build_all_from_design()` 결과에서 `meta.startfinish_s`를 확인합니다.
4. `write_dxf()`의 출발/결승선이 위 값이 아니라 traffic-light pose를 사용함을 확인합니다.

릴리스 ZIP을 푼 디렉터리에서 읽기 전용으로 재현할 수 있습니다.

```bash
python3 -B - <<'PY'
import json
from pathlib import Path
d = json.loads(Path("design_final.json").read_text())
f = d["features"]
print("design_start_line_s:", f["start_line"]["s"])
print("traffic_light_s:", f["traffic_light"]["s"])
print("id0_s:", next(m["s"] for m in f["aruco"] if m["id"] == 0))
PY
```

## 기대동작

- 설계 start/finish 기준 s가 도면·시각 표시·scene에 일관되게 반영되어야 합니다.
- 신호등과 ID 0에 의도적인 오프셋이 있다면, 출발선과 별도의 위치라는 설명과 오프셋 필드가 필요합니다.
- JSON 소비자가 출발/결승선의 위치를 명시적으로 읽을 수 있어야 합니다.

## 실제결과

- 설계 `features.start_line.s=46.246139726...`; 보간 XY는 약 `(9.963404, 6.079770)` m입니다.
- 신호등과 ID 0은 `s=0`, XY 약 `(9.9660, 6.4665)` m에 있습니다.
- 두 기준의 폐회로 거리 차이는 약 `0.386738` m입니다.
- 생성기는 설계 start_line을 `meta.startfinish_s`에 저장하지만 DXF의 start/finish 선은 신호등 pose를 사용합니다.
- `scene.json`에는 명시적인 start_line pose가 없어 소비자가 어느 값이 경기 기준인지 결정하기 어렵습니다.

## 영향

- 랩 타이밍, 출발 표시, grid-to-start 거리와 경로 진행도 기준이 팀별로 달라질 수 있습니다.
- 편집기에서 start line을 변경해도 배포 도면·scene에 반영되지 않을 수 있습니다.
- 이것은 후속 신호등의 실물 높이·모양과 무관한 설계/출력 데이터의 일관성 문제입니다.

## 근거자료

- [공식 설계 JSON](https://github.com/MOSW626/istech-it-arena/blob/d61c5db9252cedfbc163cd044a47671df91e1660/track/design_final.json)
- [설계 start line을 읽는 코드](https://github.com/MOSW626/istech-it-arena/blob/d61c5db9252cedfbc163cd044a47671df91e1660/track/track_gen.py#L1136-L1139)
- [신호등 pose로 DXF 선을 그리는 코드](https://github.com/MOSW626/istech-it-arena/blob/d61c5db9252cedfbc163cd044a47671df91e1660/track/track_gen.py#L1848-L1856)
- [좌표·거리 수치 감사](https://github.com/GIST-ISAAC-Robotics/2026-IT-ARENA/blob/main/docs/track/OFFICIAL_SOURCE_AUDIT.md)

![원본 전체 배치의 참고 화면](https://raw.githubusercontent.com/GIST-ISAAC-Robotics/2026-IT-ARENA/main/artifacts/screenshots/2026-08-31/official_update/upstream_full_overview.png)

이미지는 전체 배치의 맥락을 보여 줍니다. 두 s의 차이와 약 38.7 cm 오차의 정량 근거는 JSON 값과 동일 중심선 보간 계산입니다.

## 수정제안

1. 조직위가 의도한 출발/결승선 기준을 설계 start_line 또는 s=0 중 하나로 정합니다.
2. 모든 출력에서 그 기준을 사용합니다. 신호등을 의도적으로 이격한다면 별도 오프셋으로 표현합니다.
3. `scene.json`에 `start_finish_s_m`과 pose를 출력합니다.
4. 설계 start_line 변경 후 scene/DXF/preview가 함께 바뀌는 회귀 검사를 추가합니다.

## 완료 체크리스트

- [ ] 출발/결승선 기준 s와 신호등·ID 0 오프셋의 의도 확정
- [ ] 설계·scene·DXF·preview의 의미 일치
- [ ] 명시적 `start_finish_s_m`과 pose 제공
- [ ] start_line을 바꾸는 회귀 검사 통과
- [ ] 변경 내역과 기존 기준에서의 거리 차이 문서화
