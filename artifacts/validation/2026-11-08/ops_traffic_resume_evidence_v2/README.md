# 교통 속 STOP/START(ops_traffic_resume_v1 run1) 사후 평가 보정

2026-11-08. 보존된 [v1 실행](../ops_traffic_resume_v1/PLAN.md) `run1` 원시 자료만 다시 읽었다. Gazebo·ROS·빌드를 재실행하지 않았고, v1의 run·assessment·등록 소스는 바꾸지 않았다. **공식 판정은 그대로 실패다.** 검사기 `report.json`의 `passed=false`와 `traffic resume did not progress 1.2m within test time`, v1 `assessment.json`의 `passed=false`를 유지한다. 이 문서는 측정기 결함만 바로잡는다.

재현: `wsl.exe bash -lc "cd '/mnt/c/.../IT ARENA local' && /usr/bin/python3 -B artifacts/validation/2026-11-08/ops_traffic_resume_evidence_v2/analyze.py"`. shapely가 필요하다. 결과는 `evidence.json`, 출력은 `analyze.log`, 종료 코드는 `analyze.exit`(`EXIT=0`)에 있고, 실행 시간은 15.2초였다. 입력 파일의 sha256은 `evidence.json` `inputs`에 있다.

## 바로잡은 v1 측정기 결함

1. **actor 시각.** Gazebo Pose_V JSON에는 최상위 `header`가 없다(7,352줄 중 0줄). 시각은 각 `pose[i].header.stamp`에 있다. proto3 JSON은 0인 `sec`를 생략하므로 생략값을 0으로 읽었다. v1은 모든 시각을 0으로 읽어 actor 표본을 1개만 남겼고, 그 결과 `actor_pose_coverage`와 `positive_sample_gap`이 거짓이 되었다. 보정 결과:
   - 7,352줄을 파싱 오류 0으로 읽었다. 모델 `eval_g1b_actor`, `frame_id` `it_arena_track`, `child_frame_id` `eval_g1b_actor`가 모든 줄에서 같았다.
   - 시각은 0.01–73.52 s이고 간격은 모두 10 ms였다. 비증가 시각은 0이다.
   - 위치는 등록 자세 (9.8, 5.12)에서 0 m 벗어났다(정지 actor).
   - ego 평가 창 전체를 덮는다. actor 표본의 최대 나이는 9 ms였다.
2. **평가 창.** v1은 `final_stop_hold`가 없어서 창을 첫 hold 끝 11.357 s에서 잘랐다(ego 1,603표본). 보정 창은 visual START 8.129 s부터 `failure_stop_hold` 끝 73.477 s까지이며, ego 32,003표본을 **하나도 빼지 않고** 대조했다.

## 시간선(시뮬레이션 시각, s)

| 구분 | 값 |
|---|---|
| visual START / 실제 출발(원점에서 0.01 m 초과) | 8.129 / 8.557 |
| **첫 STOP** 요청 → 정지 확인 → 0.5 s hold | 10.682 → 10.856 → 10.856–11.357 (요청 후 0.675 s, 최고 0.039 m/s, 변위 4.1 mm) |
| RESET 없는 START 요청 → **허가** | 11.357 → 11.769 (`permitted`, 0.412 s) |
| **실패 복구용 마지막 STOP** 요청 → 정지 → hold | 72.973 → 72.977 → 72.977–73.477 (요청 후 0.504 s, 최고 0.0006 m/s) |

마지막 STOP은 계획된 `final_stop_hold`가 아니다. 진행 시간 초과 뒤 실행기가 넣은 복구 정지다.

## 허가 뒤 실제 종방향 전진

`run.py`와 같은 기준으로 계산했다. `before_resume`의 truth 자세(11.357 s, yaw 1.2465 rad)를 원점으로 두고, 그 초기 방향으로 투영했다.

| 지표 | 값 |
|---|---:|
| 최대 전진(허가~마지막 STOP 요청) | **0.2196 m** (12.652 s) |
| 마지막 STOP 요청 시 / hold 끝 | 0.2120 m / 0.2119 m |
| 평면 이동 거리 | 0.258 m |
| hold 끝 횡오프셋 | 0.085 m |
| 1.2 m 조건 | **미달(실패 유지)** |

## 최장 정체(0.05 m/s 미만)

ego 원시 표본의 간격은 1 ms 1,321개, 2 ms 31,905개, 3 ms 2,785개였다(최대 3 ms, 비증가 0). 속도는 두 방법으로 계산했다. **두 방법 모두 필터나 평활은 쓰지 않았다.**

- **원시 쌍:** 연속 표본의 변위를 시간 차로 나눈 값이다.
- **0.05 s 변위:** 각 표본에서 0.05 s 이상 앞선 가장 가까운 표본까지의 평균 속도다.

누적 방식은 r27 `assess.py`와 같다. 속도를 구간 끝 표본에 두고, 기준 미만이 연속되는 동안 시간을 더한다.

| 범위 | 원시 쌍 | 0.05 s 변위 |
|---|---:|---:|
| 허가 뒤~마지막 STOP 요청 | **60.353 s** (12.620–72.973) | 60.327 s (12.646–72.973) |
| 출발~첫 STOP 요청 | 0 s | 0 s |
| 출발~마지막 STOP 요청(운영 정지 포함) | 60.353 s | 60.327 s |

보조로 운영 기록 `ops_log.jsonl`의 상태를 평가 전용으로 집계했다. 허가 뒤 60.55 s가 `HANDOFF_FOLLOW|handoff_follow|safety clear`, 0.44 s가 `RUNNING`이었다. 즉 보호층 정지가 아니라 정지 actor 뒤에서 추종 상태를 유지한 것으로 기록되어 있다. 원인 판정은 아니며, 추월·조향 판단은 이 범위가 아니다.

## 간격·접촉·노면(보정 창 전체)

- **간격:** ego 32,003표본 각각에 대해, 그 시각 이전의 최신 actor 표본(나이 ≤ 0.05 s, 실제 최대 9 ms)과 대조했다. 외형은 둘 다 0.20×0.15 m 사각형이다.
  - 미대응 표본 0, 겹침 0이다.
  - 최소 표본 간격은 **0.4726 m**(12.652 s)였다.
  - 연속 표본 사이 ego 최대 이동은 1.85 mm이므로, 이를 뺀 보수 하한은 0.4707 m다(actor는 정지).
  - 연속 휩쓸기 증명이 아닌 이산 표본 판정이다.
- **접촉:** `contacts.jsonl`은 0바이트다. 같은 실행의 그래프 조사에서 `gz.msgs.Contacts` 발행자와 수신기 생존(`reader_alive_at_graph_audit`)을 확인했다. 이 토픽은 접촉 시에만 발행하므로, 접촉 메시지가 기록되지 않았다고 해석한다.
- **노면:** v1과 같은 식을 썼다. `evaluate_pose`를 창의 모든 ego 표본에 조향 0, ±0.45 rad 세 이산 표본으로 적용했다. 정확히 같은 자세는 결과를 재사용했는데, 계산식은 그대로이므로 동등하다(고유 자세 13,272개).
  - 이탈 바퀴 최대는 any/center/full 해석 모두 0이다.
  - 벽 바퀴 표본 0, 차체가 도로 밖으로 나간 표본 0이다.
  - 실제 조향 관절 기록은 없다.

## 보정 후 원 게이트 상태(참고, 판정 번복 아님)

| 게이트 | v1 | 원시 보정 |
|---|---|---|
| `actor_pose_coverage` | 거짓(측정기 결함) | 성립: 7,352표본, 10 ms, 미대응 0 |
| `positive_sample_gap` | 거짓(간격 없음) | 성립: 0.4726 m |
| `no_contact`, `wheels` | 참(잘린 창) | 전체 창에서도 성립 |
| `dense_stop` | 거짓(hold 2개 요구) | 첫 hold와 복구 hold는 각각 기준을 만족한다. 다만 복구 hold는 계획된 최종 hold가 아니므로 성립으로 바꾸지 않는다 |
| `progress`(1.2 m), `completed` | 거짓 | **거짓 유지**: 최대 0.2196 m, 60.35 s 정체 |

## 한계

- 정지 actor 한 대를 둔 한 장면의 시뮬레이션 사후 평가다. 이동차 추종 재개, 실차 STOP, 공유 Wi-Fi는 증명하지 않는다.
- 간격과 노면은 표본 단위 판정이며, 조향은 세 이산 표본이다.
- ego 튜플의 다섯째 값은 이번 계산에 쓰지 않았다.
- 센서 오차 비교는 [별도 후속](../newcourse_sensor_stress_audit_v1/README.md)이며 여기서 구현하지 않았다.
