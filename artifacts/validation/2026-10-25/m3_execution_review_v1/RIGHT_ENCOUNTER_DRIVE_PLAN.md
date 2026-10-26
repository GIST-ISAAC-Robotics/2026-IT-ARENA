# M3 오른쪽 도로 내 피추월 — run21 제한 1회

2026-10-25 GPT 설계·검토. run17 왼쪽은 진행 개선·약 70 mm 간격 실패, run19 정지·재개 결합은 정지/재개 통과·최종 추종 간격 실패로 보존한다. 오른쪽은 순차 취득 시각과 선호 벽의 가림이 다르므로 별도 한 번을 확인한다. 더 넓은 간격으로 실패를 숨기지 않는다.

## 조건

- `e3_m3_right_grid_v1.json`: `g1_traffic_v1.json`에서 자차를 왼쪽 0번 슬롯으로만 옮긴 시험 사본. 공식 슬롯 0은 슬롯 3보다 0.20 m 앞이므로 엄밀한 동일 위치 거울 시험은 아니다. 나머지 속도·기간·센서·물성·판정은 같다.
- `e3_onroad_right_encounter_v1.json`: 옆차 횡위치만 +0.13 → −0.13 m. 초기 종방향 간격 −0.60 m, 속도 0.45 m/s와 앞차 slow_lead는 같다.
- 정적 사전 검사에서 자차 횡위치 +0.090023 m, 명목 평행 간격 70.023 mm, 자차 도로 이탈 0, 상대 경로 541표본의 최소 도로 끝 여유 18.977 mm와 actor끼리 간격 ≥0.80 m를 확인했다. [사전 검사](run20_followup_audit/right_preflight.json). 이것은 실제 자차 궤적 검증이 아니다.
- 제어 소스는 run17/run19와 같고 모든 실험 옵션을 명시한다. 선호 벽은 기존 왼쪽으로 유지한다. C1은 기존 −180° 시작 순차 취득·10 Hz·후방 60° 마스크다. 실제 C1 순서는 아직 미확정이다.
- `/drive` → 독립 보호층 → `/drive/safe`, STOP, 단일 모터축 28카운트·15:1·65 mm를 보존한다. 원래 비교 150 mm·별도 가정 97 mm는 바꾸지 않는다.

## 목적과 종료

촉발 시각, 옆차 통과, 자차가 만든 간격 손실, 관측 기반 복귀 여부, 진행·실제 앞 차량 간격을 구분한다. 원래 주행 판정·STOP·밀집 정지·소스 불변·정상 종료·잔존 0·판단 재생·노면을 기록한다. 한 번만 실행하고 같은 실패를 자동 반복하지 않는다. 실패는 실패로 보존하며 양쪽·전체 랩·실차 완료로 확대하지 않는다.

```bash
python3 -B scripts/validate_g1_traffic.py --case slow_lead --seed 0 --mode m3 --m3-command-fallback --m3-release-to-baseline --record-follow-decisions --record-braking-diagnostics --boundary-evidence corner_distance_candidate --occupancy-timing per_ray_candidate --corridor-fallback --config config/tests/e3_m3_right_grid_v1.json --actor-overlay config/tests/e3_onroad_right_encounter_v1.json --output artifacts/validation/2026-10-25/m3_execution_review_v1/run21_right_encounter/run
```
