# M3 선택형 후보의 첫 U자 코너 추종 — run24 1회

2026-10-25 GPT 검토. 좌우 직선 피추월은 통과 후 복귀와 진행만 개선했고 간격 실패다. 정지·재개 결합에는 후방 가림으로 인한 복귀 보류가 남는다. 이제 다른 필수 장면인 코너에서 고정 구조를 옆차로 잘못 보거나 시험 상태에 갇히는지를 한 번 확인한다. 결과를 보고 같은 사례를 자동 반복하지 않는다.

- 기존 `g1_traffic_extensions_v1.json`의 `first_curve`, seed0, 슬롯3, 앞차 0.35 m/s·35초. 추가 옆차는 없다. 코너 형상과 앞차가 측방 촉발을 만드는지 확인하는 곡선 추종 시험이지 코너 병렬주행 성공의 증거가 아니다.
- 제어 소스는 run17/19/21과 같다. M3·명령 재선택·관측 기반 복귀·corner-distance/per-ray·회랑을 명시하고 최근 관측은 OFF다. 모든 옵션의 기본값은 유지한다.
- 원래 진행 ≥11.7 m, 출구 s≥9.8 m·방향 변화 ≥150°, 최종 간격 ≤1.2 m, 정체 ≤2.5초, 최소 간격 0.15 m 및 접촉·노면·STOP·정상 종료 기준을 보존한다.
- 실패 시 최초 촉발/정지의 반환이 벽인지 앞차인지 평가 정답으로만 분류한다. 제어기에 지도·정답 자세를 넣지 않는다. 센서·물성·구동·보호 경계 변경은 없다.
- 기존 등록 경로 검사와 새 설치본 동일성을 확인한 뒤 한 번 실행하고, 판단 재생·바퀴 노면 평가를 회수한다. 정상 종료를 주행 성공으로 바꾸지 않는다.

```bash
python3 -B scripts/validate_g1_traffic.py --case first_curve --seed 0 --mode m3 --m3-command-fallback --m3-release-to-baseline --record-follow-decisions --record-braking-diagnostics --boundary-evidence corner_distance_candidate --occupancy-timing per_ray_candidate --corridor-fallback --config config/tests/g1_traffic_extensions_v1.json --output artifacts/validation/2026-10-25/m3_execution_review_v1/run24_first_curve/run
```
