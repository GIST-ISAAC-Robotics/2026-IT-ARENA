# C1급 10 Hz 순차 취득 검증

- 구현·해석·회의용 요약: [센서 시험 문서](../../../../docs/sensors/C1_SEQUENTIAL_SPEED_EXPERIMENT.md)
- 본 비교: [14회 표·그래프](matched_matrix_v1/RESULTS.md)
- 경계 조건 추가 확인: [직선 15 km/h 순간·순차 각 1회](boundary_repeat_v1/RESULTS.md). 본 비교와 합쳐 순간 2/2 통과·순차 0/2 통과.
- 초기 1000 Hz 원시 광선 스모크: [9월 5일 5 km/h 순차 실행](../../2026-09-05/rotating_lidar/straight5_sequential_01/report.json)

## 조건과 원시 기록

10 Hz·회전당 500점, 기본 500 Hz 원시 광선 프레임에서 순차 선택합니다.
`snapshot_matched`도 동일한 원시 계산·전달 경로를 사용합니다. 2 kg 차량,
단일 모터·임시 물성·100 Hz 벽 조향은 같고, ToF 보호 및 일반 데모의 거리
비례 감속은 독립 시험에서 해제했습니다. 공식 코스나 차량 치수는 바꾸지 않았습니다.

각 실행은 다음을 포함합니다.

- `report.json`: 최초 측정·소스 해시·센서/정지/종료 판정
- `trace.jsonl`: 지면 속도·위치·차체 상태·노면/벽 여유의 평가용 관측
- `commands.jsonl`: 실제 조향·속도 요청과 이유·사용 스캔 시각
- `scans.json`, `scan_examples.json`, `acquisition.json`: 수신 메타데이터·일부 거리 배열·시간 근사 오차
- `simulation.log`, `trial_world.sdf`: 실행 로그와 독립 시험장
- `phase_audit.json`: 주행과 제동을 분리한 후속 재집계

평가 위치 정답은 관측기에만 사용했습니다. 제어기 구독은 `/scan`·`/clock`이며
고빈도 원시 거리나 정답 경로를 제어에 전달하지 않았습니다. 이탈/벽 겹침은
보수적 20×17 cm 평면 평가 상자의 표본 검사이며 접촉 센서로 측정한 충돌 횟수가 아닙니다.

## 집계 오류와 정정

최초 집계는 `elapsed < 8` 조건으로 주행 표본을 골라 조기 중단 후 제동도 일부
포함했습니다. `phase == running`을 추가하고 회귀 검사를 보완했습니다.
최초 보고서와 행렬 `summary.json`은 덮어쓰지 않습니다. 최종 표는 원시 trace를
재집계한 `phase_audit.json` 기준이며 평가 코드 해시도 별도로 기록합니다.

본 비교 14회는 모두 동일한 수정 전 검사기 해시
`14e0958d804a786a28ab72b9e604f8f6c5f5d733c3b7bdbaa47c26f096c9956c`로 실행했습니다.
해당 파일은 `matched_matrix_v1/validator_before_phase_audit.py`에 보존했습니다.
집계 정정으로 최초 14회의 통과·실패 판정은 바뀌지 않았습니다.
추가 반복은 집계 수정 후의 검사기를 사용하며 센서·조향·차량 로직은 같습니다.

## 자동 검사

ROS 환경에서 `python3 -m pytest -q tests src/arena_vehicle_interface/test`:
165개 통과(집계 회귀 수정 후 재확인). 5개 ROS 패키지 빌드는 이 작업 시작 시
완료했습니다. 시험의 센서 수신·실제 정지·각 자식 정상 종료는 각 보고서에서
별도로 확인하며 자동 검사 통과만으로 주행 통과를 대신하지 않습니다.

## 재생성

```bash
python3 scripts/run_rotating_lidar_comparison.py --output artifacts/validation/새로운_폴더
python3 scripts/summarize_rotating_lidar.py artifacts/validation/2026-09-06/rotating_lidar/matched_matrix_v1
```

실차 잡음·반사율·가림·통신·고정 회전 위상·실차 물성은 미검증입니다.
이 시험으로 C1의 절대 최대속도나 공식 코스·다중 차량 안전을 인증하지 않습니다.
