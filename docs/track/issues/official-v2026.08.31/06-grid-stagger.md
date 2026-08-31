**어떤 파일에서**

`track/track_gen.py:build_grid_zone_slots()/write_scene_json()`, `track/output_final/scene.json:starting_grid`

**무엇이 이상한가요**

실제 슬롯은 열 사이를 0.20 m 엇갈리게 생성하지만 `longitudinal_stagger_m` 메타데이터에는 적용 전 상수 0.30 m가 기록됩니다.

**환경**

- 재현 환경: Windows 호스트의 WSL Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Sim 8.11.0
- repository: `MOSW626/istech-it-arena`
- commit/release: `d61c5db9252cedfbc163cd044a47671df91e1660` / `v2026.08.31`
- Python 3의 JSON 읽기만으로 재현 가능하며 Gazebo 실행이 필요하지 않음

## 재현 방법

릴리스 ZIP을 푼 디렉터리에서 실행합니다.

```bash
python3 -B - <<'PY'
import json
from pathlib import Path
g = json.loads(Path("output_final/scene.json").read_text())["starting_grid"]
print("metadata_stagger_m:", g["longitudinal_stagger_m"])
for row in sorted({s["row"] for s in g["slots"]}):
    pair = {s["col"]: s for s in g["slots"] if s["row"] == row}
    print("row", row, "actual_delta_s_m:", round(pair[0]["s_m"] - pair[1]["s_m"], 6))
PY
```

## 기대동작

- `longitudinal_stagger_m`가 실제 생성된 슬롯의 열 간 종방향 차이와 일치해야 합니다.
- 요청값과 적용값을 둘 다 제공하려면 서로 구분되는 필드명을 사용해야 합니다.

## 실제결과

- `starting_grid.longitudinal_stagger_m=0.30` m입니다.
- 같은 row의 두 열을 비교하면 실제 `slots[*].s_m` 차이는 세 row 모두 약 `0.20` m입니다.
- 생성기는 `min(GRID_STAGGER, row_spacing * 0.4)`로 배치를 제한하지만, scene에는 제한 전 상수 `GRID_STAGGER`를 기록합니다.
- 현재 grid 구간 1.5 m를 3행으로 나누므로 row spacing≈0.5 m, 적용 stagger≈0.2 m가 됩니다.

## 영향

- 슬롯 pose 대신 메타데이터로 그리드를 재구성하는 프로그램은 배포 월드와 다른 배치를 만듭니다.
- 현장 슬롯 간격, 시뮬레이터 이식, grid-to-start 검증 결과가 서로 달라질 수 있습니다.
- 슬롯 pose 자체의 겹침 실패가 아니라 메타데이터의 의미 오류입니다.

## 근거자료

- [실제 stagger 제한 계산](https://github.com/MOSW626/istech-it-arena/blob/d61c5db9252cedfbc163cd044a47671df91e1660/track/track_gen.py#L501-L520)
- [scene에 상수를 기록하는 코드](https://github.com/MOSW626/istech-it-arena/blob/d61c5db9252cedfbc163cd044a47671df91e1660/track/track_gen.py#L2012-L2017)
- [공식 scene의 메타데이터·슬롯 pose](https://github.com/MOSW626/istech-it-arena/blob/d61c5db9252cedfbc163cd044a47671df91e1660/track/output_final/scene.json#L4151-L4222)
- [수치 감사](https://github.com/GIST-ISAAC-Robotics/2026-IT-ARENA/blob/main/docs/track/OFFICIAL_SOURCE_AUDIT.md)

![공식 슬롯들이 포함된 전체 배치 참고 화면](https://raw.githubusercontent.com/GIST-ISAAC-Robotics/2026-IT-ARENA/main/artifacts/screenshots/2026-08-31/official_update/upstream_full_overview.png)

스크린샷은 배치 참고용이며 0.20/0.30 m 차이의 정량 근거는 위 JSON 읽기 명령입니다.

## 수정제안

1. 슬롯 생성 시 실제 적용한 stagger를 결과 메타데이터에 함께 반환합니다.
2. `longitudinal_stagger_m`에 실제값 0.20 m를 기록하거나, `requested_longitudinal_stagger_m`와 `applied_longitudinal_stagger_m`를 분리합니다.
3. 생성된 슬롯 s에서 다시 계산한 값과 메타데이터를 대조하는 회귀 검사를 추가합니다.
4. grid 구간 길이나 차량 수를 바꿔 clamp가 달라지는 경우도 검사합니다.

## 완료 체크리스트

- [ ] 메타데이터의 stagger가 실제 슬롯 pose와 일치
- [ ] 요청값/적용값을 둘 다 유지한다면 이름과 의미 구분
- [ ] 세 row에 대해 같은 열 간 차이를 확인하는 회귀 검사 통과
- [ ] grid 구간·차량 수 변경 시에도 메타데이터 일치
- [ ] README/scene 스키마 설명 갱신
