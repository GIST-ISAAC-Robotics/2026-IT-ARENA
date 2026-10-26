# 관측 기반 복귀 — run17 제한 폐루프 1회

2026-10-25 GPT 검토. run14의 부재 기반 복귀 주행안은 반려·미실행으로 보존한다. run16은 관련 139회귀, 저장 run13/run9 OFF 637/636 전체 출력 동일, ON은 각각 7.02/6.92 s에 복귀했다. 저장 입력은 새 궤적 증거가 아니다.

## 질문과 범위

- run13과 같은 왼쪽 도로 내 옆차·seed0·slow_lead·명령 재선택 ON·회랑 ON·최근 관측 OFF이다. `--m3-release-to-baseline`만 추가한다. 옵션의 현 구현은 과거 위협 가능 영역의 운동 전파와 양의 자유 관측, 복귀 회랑/정지 거리 검사, 서로 다른 스캔의 연속 확인을 요구한다.
- 목적은 옆차 통과 뒤 불필요한 M3 상태 유지와 정지/출발 반복을 줄이는지 확인하는 것이다. 97 mm 측방 간격 개선은 별도다. 원래 평행 간격 약 70 mm가 그대로면 실패로 기록한다.
- 마지막 반환 위치만 쓰던 run14와 다르게, 가림·무응답·점 개수 미달·빈 기록·세대/시각 단절을 부재로 치환하지 않는다. 시간 연결을 잃은 활성 위협은 명시 reset 전까지 HOLD한다.
- 남은 한계: 2 cm 표본, 현재 회랑 좌표/최대 차량 크기/이동 상계의 가정, 1초 기록 만료, 미관측 영역의 새로운 차량 유입, 기본 G1 복귀 뒤 후방 및 측방 전체 검사가 없는 점. 현재 영역이 관측됐다는 사실을 장래의 비어 있음으로 확대하지 않는다. 이 시험의 고정 장면에서만 비교하며 기본 채택·실물 안전 검증이 아니다.

## 실행

설치본 해시·소스 동결 확인 후 아래 한 번만 실행한다. 결과와 소스 ZIP을 보존한다. 실패 시 동일 조건 재시도 없이 분석한다.

```bash
python3 -B scripts/validate_g1_traffic.py --case slow_lead --seed 0 --mode m3 --m3-command-fallback --m3-release-to-baseline --record-follow-decisions --record-braking-diagnostics --boundary-evidence corner_distance_candidate --occupancy-timing per_ray_candidate --corridor-fallback --actor-overlay config/tests/e3_onroad_encounter_v1.json --output artifacts/validation/2026-10-25/m3_execution_review_v1/run17_observed_release_closed_loop/run
```

## 판정

- 기존 보고서의 진행 2.5 m·최종 앞차 간격 1.2 m·측방 0.15 m·차체 노면 판정을 보존한다. 별도 97 mm·두 바퀴 잔디 평가로 기존 실패를 소급 통과시키지 않는다.
- 입력/시험기 유효·모드 표식·독립 보호 경계·접촉·명시 STOP와 밀집 정지·정상 종료·잔존 0·소스/입력 불변·판단 재생을 확인한다. 정상 종료와 주행 성공을 분리한다.
- 바퀴 노면 `any/center/full`을 기록하고, `any`의 3바퀴 이상 이탈·기타 지면·벽 접촉은 실패다. 2바퀴 잔디 시간은 따로 기록한다.
- 복귀 시각/횟수·재활성·복귀 전후 옆차/앞차 간격·목표 속도/조향·정지 누적을 비교한다. 정답 자세는 사후 평가에만 쓴다.
- 느린 시뮬레이션에서 MCU 갱신 기한을 지킨 결과를 실시간 처리 여유라고 하지 않는다. run16 관문 계산을 수행한 9/10호출의 p95 약 92/74 ms는 실제 연산 비용의 별도 한계다.
