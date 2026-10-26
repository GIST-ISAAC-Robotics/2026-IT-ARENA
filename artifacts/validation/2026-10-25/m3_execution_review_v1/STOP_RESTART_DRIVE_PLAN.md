# 관측 기반 복귀와 M2 정지·재개 — run19 1회

2026-10-25 GPT 설계·검토. run17에서 복귀 뒤 전진은 개선됐으나 M3 최소 횡간격은 실패로 보존한다. E3의 기존 앞차 정지·재개와 잔디 쪽 옆차가 겹치는 장면에서 선택형 관문의 상호작용을 확인한다. 더 넓은 측방 간격을 사용해 도로 내 피추월 실패를 통과시키는 시험이 아니다.

## 고정 조건

- `config/tests/g1_traffic_v1.json`의 `stop_restart`, seed 0, 3번 슬롯. 앞차는 0–4초 0.35 m/s, 4–9초 정지, 9–15초 0.35 m/s 재개다.
- 원래 `e3_stop_restart_v1.json`의 잔디 쪽 옆차(중앙선 +0.32 m, 초기 종방향 −0.50 m, 0.35 m/s)를 그대로 사용한다. 도로 안 옆차는 아니다.
- run17과 같은 M3·명령 재선택 ON·관측 기반 복귀 ON·`corner_distance_candidate`·`per_ray_candidate`·회랑 ON·최근 관측 OFF. 기본값은 여전히 OFF다.
- 센서·구동·노면·간격·시간/보호/STOP 경계는 바꾸지 않는다. `/drive` → 독립 보호층 → `/drive/safe` 유지. 정답은 평가용이다.

## 판정

기존 정지·재개 판정과 진단을 그대로 사용한다. 앞차 정지 중 실제 자차 정지, 재개 지연·진행, 최소 간격, 옆차 가림에 의한 HOLD와 해제 횟수·이유를 분리한다. 명시 STOP·MCU STOP·밀집 정지·정상 종료·소스 불변·잔존 0·판단 재생을 확인한다. 노면 any/center/full 접지와 간격을 별도 평가하되 원래 판정을 소급 변경하지 않는다.

이번 한 번의 폐루프 결과로 실차·전체 랩·양쪽 피추월·50 Hz 실시간 성능을 증명하지 않는다. 실패하면 원시 입력과 소스를 보존하고 분석한다. 같은 조건 자동 재시도는 하지 않는다.

```bash
python3 -B scripts/validate_g1_traffic.py --case stop_restart --seed 0 --mode m3 --m3-command-fallback --m3-release-to-baseline --record-follow-decisions --record-braking-diagnostics --boundary-evidence corner_distance_candidate --occupancy-timing per_ray_candidate --corridor-fallback --actor-overlay config/tests/e3_stop_restart_v1.json --output artifacts/validation/2026-10-25/m3_execution_review_v1/run19_stop_restart/run
```
