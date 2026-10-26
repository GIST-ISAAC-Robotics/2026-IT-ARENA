# M3 선택형 연결 — 첫 제한 주행 사전 등록

등록: 2026-10-25, 새 M3 Gazebo 주행 전. 사용자의 10/24 간격 축소·두 바퀴 잔디 활용·후속 개발 지시에 따른 시험이며, 실물 위험 수용이나 기본 활성화는 아니다.

## 첫 실행과 중단선

1. 기본 false `g1_m3_execution_trial`을 명시한 `--mode m3`. 비교 입력은 기존 `e3_onroad_encounter_v1.json`, `slow_lead`, seed 0 그대로. G1 경계는 `corner_distance_candidate`, `per_ray_candidate`, 회랑 보완 ON, 최근 관측 보완 OFF다. 이 조합은 저장 입력 대조에 사용한 기본 G1과 같다.
2. C1 순차 10 Hz·both 보정·B0183+LSM6DSOX·단일 모터축 28카운트/15:1/65 mm·가상 MCU·현재 물성·속도 상한을 유지한다. 측방 후보의 상한은 0.35 m/s, 기본 G1은 기존 1 m/s다. 임시 신호는 시험 시작 자극이며 공식 출발 구현의 증명이 아니다.
3. 도로 안 왼쪽 옆차 1회만 먼저 실행한다. 결과와 원인을 확인하기 전 오른쪽·앞차 정지재개를 연속 실행하지 않는다. 실패를 같은 설정으로 반복하지 않는다. 처리 지연으로 MCU 래치/보호 정지가 나면 그 증거부터 분리한다.
4. 대회 노드는 `/drive`만 발행하고, `/drive/safe`는 독립 보호층만 발행한다. 정답 자세·actor 일정은 평가 전용이다. 노드에서 `use_sim_time=true`, 판단 기록, per-ray 설정을 요구한다. 이는 시뮬 시험용 제약이며 실물 사용 승인 장치로 해석하지 않는다.

## 판정과 결과 표현

- 기존 평가기의 15 cm·차체 도로 내 기준 결과를 삭제하거나 덮어쓰지 않는다. 이전 실패 비교용 원시 판정으로 보존한다.
- M3 별도 판정: 실행 무결성·actor 시험기·명시 STOP·밀집 정지·정상 종료·잔존 0·판단 기록 재생 일치가 모두 필요하다.
- 접촉/벽 충돌 0. 옆차 sampled/swept 간격을 가정 직선 예산 0.097 m와 대조한다. 기존 종방향 앞차 간격/재개 조건은 유지한다. 0.097 m는 실제 안전 거리 확정값이 아니다.
- 노면은 `surface_contact_eval`의 `any` 해석으로 3바퀴 이상 이탈 0·기타 지면 0·벽 접촉 0을 요구한다. 두 바퀴 이하 잔디 시간과 3초 초과 여부를 별도로 보고한다. `center/full` 해석도 함께 보존한다. 실제 심판의 접지 정의는 확정하지 않는다.
- 옆차가 뒤→옆→앞으로 실제 통과했는지, 통과 후 자차가 다시 전진했는지 별도 기록한다. 기존 slow_lead의 총 진행/최종 앞차 거리/정지 지속 기준을 함께 본다. 후보 활성 양수 명령만으로 진행성 통과라 하지 않는다.
- 활성 판단 비용은 저장 재생에서 54–63 ms p95로 남아 있다. 이번 주행은 PC 실시간 성능·Jetson·실차 안전·전체 코스 성공을 입증하지 않는다.

## 고정 명령

ROS Jazzy와 현재 `install/local_setup.bash`를 읽은 WSL bash에서, 설치본/소스 일치 확인 후 1회 실행한다. 출력 폴더는 실행 전에 없어야 한다.

```bash
python3 -B scripts/validate_g1_traffic.py --case slow_lead --seed 0 --mode m3 --record-follow-decisions --record-braking-diagnostics --boundary-evidence corner_distance_candidate --occupancy-timing per_ray_candidate --corridor-fallback --actor-overlay config/tests/e3_onroad_encounter_v1.json --output artifacts/validation/2026-10-25/m3_execution_review_v1/run9_closed_loop_left/run
```

종료 뒤 `scripts/replay_follow_trace.py`와 `scripts/surface_contact_eval.py`를 별도 결과 경로로 실행한다. 후보 구동 외곽은 이산 표본이며 연속 안전·후방 차량의 미래 진입·횡슬립·실물 제동은 미검증이다.
