# r549 갈림길 반대벽 검토 입력

**두 합성 입력만 준비했다. Windows에서는 공통 제어기 계산을 실행하지 않았다.** NumPy2.5.4의 `np.cross([1,0],[0,1])`가 2차원 입력 ValueError를 내므로 지시대로 호환 패치/환경 변경을 하지 않았다. 아직 실제 반례 성립을 주장하지 않는다.

Y갈림길의 두 외벽은 x=−0.15m에서 각각 좌/우45°로 벌어지며, x=0.70m부터 중앙 섬이 두 분기를 나눈다. 선호벽 정책은 left다. 모든 기하는 시험 가정이며 실제 경기장 갈림길을 복제한 것이 아니다.

1. `preferred_continuous_scan.json`: 좌/우 외벽 모두 연속. 유한 반환403·UNKNOWN97개.
2. `preferred_fragmented_scan.json`: 같은 형상에서 좌벽의 x=0.05…0.70m 구간만 실제 선분이 없는 입력. 유한 반환366·UNKNOWN134개. 오른벽은 그대로여서 다른 분기로 이어지는 현재 경로를 만들 수 있는지 검사한다.

두 입력의500개 원 광선 index·방향·취득시각은 같다. 센서 원점(.06,0)m에서 가장 가까운 실제 선분 교차를 계산했으며, 다른 벽/섬의 더 먼 반환이 있으면 그대로 사용한다. 교차가 없는 null range는 실행 스크립트에서 inf→UNKNOWN으로 복원한다. 빈 공간으로 채우지 않는다. `evaluation_geometry.json`은 입력 생성/평가용이며 제어기 호출에 전달하지 않는다.

`follow_config.json`은 기존 r548 fixture와 새 명령 상한0.28475436568260193rad다. `run_native_review.py`는 원 입력을 scene으로 바꾼 뒤 각 벽의 기존 `local_path`와 변경된 helper를 호출한다. 선호벽 경로 생성 여부, 선택된 벽, fallback 여부, 명령 조향, 후보 경로 끝 방향, 기존 양측 `path_agrees/path_agrees_arc`, 점유 여유를 기록한다. **동적 주행이나 실제 분기 진입/완료를 판정하는 시험은 아니다.**

root의 지원 환경에서 다음 명령 한 번으로 확인할 수 있다.

```text
python3 -B artifacts/validation/2026-11-14/current_short_path_r549_v1/junction_review/run_native_review.py
```

실행 결과는 새 `native_result.json`에 한 번만 기록한다. 원 소스 SHA와 준비 시점 SHA도 비교한다. `prepare_inputs.py`는 표준 라이브러리의 ray 교차 생성만 수행했고 공통 파일·WSL·Gazebo·Jetson 접근은 없었다. 새 정지 관문이나 제어기 수정은 만들지 않았다.
