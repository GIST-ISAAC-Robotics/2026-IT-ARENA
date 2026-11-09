# 편집표 — feature_showcase.mp4

자동 생성(`tools/compose_showcase.py`). 전체 1744프레임·87.20 s·20 fps·1280×720, 모든 장면 1.0배속(프레임 1장=시뮬레이션 0.05 s). 사건 시각은 원시 기록에서 계산했으며 판정 결과는 각 원 보고서를 따른다. 상대차 자세·정답 자세는 영상/사후 설명에만 쓰였다.

## ① 단독 주행 · 거리 한 랩 — 영상 0.00–29.30 s

- 원자료: `artifacts/validation/2026-11-08/solo_gap_envelope_connected_v3/run1` (원본 3인칭 녹화)
- 원 시뮬레이션 시각: 8.25–37.50 s, 586프레임
- 보고서: `artifacts/validation/2026-11-08/solo_gap_envelope_connected_v3/README.md`

| 영상 시각 | 원 시뮬 시각 | 사건(기록 기준) |
|---:|---:|---|
| 0.04 s | 8.295 s | RGB 출발 기록(live_summary race_start_sim_s) |
| 2.17 s | 10.420 s | 자세 차분 속도 최대 부근(약 2.87 m/s; 원 보고 최고 2.862 m/s) |
| 5.80 s | 14.051 s | 진행 10 m 통과(trajectory progress_m) |
| 12.11 s | 20.359 s | 진행 20 m 통과(trajectory progress_m) |
| 19.61 s | 27.859 s | 진행 30 m 통과(trajectory progress_m) |
| 24.95 s | 33.205 s | 진행 40 m 통과(trajectory progress_m) |
| 26.88 s | 35.133 s | 후반 짧은 저속 표본 시작(42.75 m 부근) |
| 구간 밖 | 37.523 s | 목표 도달 표본(영상 마지막 프레임 37.50 s 뒤 0.023 s) |

## ② 저속 0.30 m/s 앞차 추월 — 영상 29.30–40.05 s

- 원자료: `artifacts/validation/2026-11-07/traffic_piecewise_v1/straight_moving_lead/run1` (기록 자세 Gazebo 재렌더)
- 원 시뮬레이션 시각: 7.70–18.40 s, 215프레임
- 보고서: `artifacts/validation/2026-11-07/traffic_piecewise_v1/README.md`

| 영상 시각 | 원 시뮬 시각 | 사건(기록 기준) |
|---:|---:|---|
| 31.21 s | 9.607 s | 제어 상태 FOLLOW→MOVING_PASS |
| 34.35 s | 12.750 s | 종방향 겹침 시작: 자차 앞끝이 상대 뒤끝에 도달, 중심 횡간격 0.36 m |
| 34.91 s | 13.310 s | 두 차 중심 나란히(자차가 앞으로), 중심 횡간격 0.36 m |
| 35.48 s | 13.880 s | 종방향 겹침 끝: 자차 뒤끝이 상대 앞끝을 지남(자차 완전히 앞섬), 중심 횡간격 0.36 m |
| 38.99 s | 17.391 s | 제어 상태 MOVING_PASS→FOLLOW |

## ③ 중속 0.50 m/s 비정형 앞차 추월 — 영상 40.05–49.10 s

- 원자료: `artifacts/validation/2026-11-08/medium_recessed_pass_v1/straight_moving_lead/run1` (기록 자세 Gazebo 재렌더)
- 원 시뮬레이션 시각: 8.40–17.40 s, 181프레임
- 보고서: `artifacts/validation/2026-11-08/medium_recessed_pass_v1/README.md`

| 영상 시각 | 원 시뮬 시각 | 사건(기록 기준) |
|---:|---:|---|
| 41.36 s | 9.707 s | 제어 상태 FOLLOW→MOVING_PASS |
| 44.52 s | 12.870 s | 종방향 겹침 시작: 자차 앞끝이 상대 뒤끝에 도달, 중심 횡간격 0.36 m |
| 45.03 s | 13.380 s | 두 차 중심 나란히(자차가 앞으로), 중심 횡간격 0.36 m |
| 45.52 s | 13.870 s | 종방향 겹침 끝: 자차 뒤끝이 상대 앞끝을 지남(자차 완전히 앞섬), 중심 횡간격 0.36 m |
| 48.45 s | 16.800 s | 제어 상태 MOVING_PASS→FOLLOW |

## ④ 앞차 가속 → 양보 후 뒤로 복귀 — 영상 49.10–57.55 s

- 원자료: `artifacts/validation/2026-11-07/return_guide_front_v1/straight_moving_lead/run2` (기록 자세 Gazebo 재렌더)
- 원 시뮬레이션 시각: 9.00–17.40 s, 169프레임
- 보고서: `artifacts/validation/2026-11-07/return_guide_front_v1/README.md`

| 영상 시각 | 원 시뮬 시각 | 사건(기록 기준) |
|---:|---:|---|
| 50.21 s | 10.111 s | 제어 상태 FOLLOW→MOVING_PASS |
| 51.55 s | 11.450 s | 상대 가속(1초 이동속도 0.40 m/s 초과) |
| 56.28 s | 16.181 s | 제어 상태 MOVING_PASS→FOLLOW |
| 57.31 s | 17.210 s | 상대 감속(1초 이동속도 0.40 m/s 미만) |

## ⑤ 정지차 우회 후 본선 복귀 — 영상 57.55–68.10 s

- 원자료: `artifacts/validation/2026-11-08/rear_face_motion_connected_v1/persistent_block/run1` (기록 자세 Gazebo 재렌더)
- 원 시뮬레이션 시각: 8.70–19.20 s, 211프레임
- 보고서: `artifacts/validation/2026-11-08/rear_face_motion_connected_v1/README.md`

| 영상 시각 | 원 시뮬 시각 | 사건(기록 기준) |
|---:|---:|---|
| 57.94 s | 9.091 s | 제어 상태 FOLLOW→BYPASS |
| 61.16 s | 12.310 s | 종방향 겹침 시작: 자차 앞끝이 상대 뒤끝에 도달, 중심 횡간격 0.30 m |
| 62.14 s | 13.290 s | 두 차 중심 나란히(자차가 앞으로), 중심 횡간격 0.28 m |
| 62.68 s | 13.830 s | 종방향 겹침 끝: 자차 뒤끝이 상대 앞끝을 지남(자차 완전히 앞섬), 중심 횡간격 0.28 m |
| 65.73 s | 16.885 s | 제어 상태 BYPASS→FOLLOW |

## ⑥ 앞차 정지·재출발 → 뒤로 합류 — 영상 68.10–81.35 s

- 원자료: `artifacts/validation/2026-11-08/rear_face_restart_v1/moving_stop_restart/run1` (기록 자세 Gazebo 재렌더)
- 원 시뮬레이션 시각: 10.20–23.40 s, 265프레임
- 보고서: `artifacts/validation/2026-11-08/rear_face_restart_v1/README.md`

| 영상 시각 | 원 시뮬 시각 | 사건(기록 기준) |
|---:|---:|---|
| 68.76 s | 10.860 s | 상대 정지(1초 이동속도 0.05 m/s 미만) |
| 69.83 s | 11.931 s | 제어 상태 FOLLOW→BYPASS |
| 72.84 s | 14.940 s | 종방향 겹침 시작: 자차 앞끝이 상대 뒤끝에 도달, 중심 횡간격 0.30 m |
| 72.97 s | 15.070 s | 상대 재출발(1초 이동속도 0.05 m/s 초과) |
| 73.65 s | 15.750 s | 상대 가속(1초 이동속도 0.40 m/s 초과) |
| 73.89 s | 15.990 s | 종방향 겹침 끝: 상대가 다시 자차 앞으로 벗어남, 중심 횡간격 0.28 m |
| 79.64 s | 21.745 s | 제어 상태 BYPASS→FOLLOW |

## ⑦ 빠른 뒤차에게 추월당함 — 영상 81.35–87.20 s

- 원자료: `artifacts/validation/2026-11-08/rear_overtake_measured_v1/left_rear_overtakes/run1` (기록 자세 Gazebo 재렌더)
- 원 시뮬레이션 시각: 8.60–14.40 s, 117프레임
- 보고서: `artifacts/validation/2026-11-08/rear_overtake_measured_v1/README.md`

| 영상 시각 | 원 시뮬 시각 | 사건(기록 기준) |
|---:|---:|---|
| 84.82 s | 12.070 s | 종방향 겹침 시작: 뒤 상대 앞끝이 자차 뒤끝에 도달, 중심 횡간격 0.28 m |
| 84.95 s | 12.200 s | 두 차 중심 나란히(상대가 앞으로), 중심 횡간격 0.28 m |
| 85.09 s | 12.340 s | 종방향 겹침 끝: 상대가 다시 자차 앞으로 벗어남, 중심 횡간격 0.28 m |

