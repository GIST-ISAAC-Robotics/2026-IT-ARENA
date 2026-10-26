# 작은 중앙 보정 생략 — run27 정지·재개 1회 (사전 등록, GPT 승인 완료)

2026-10-25 Claude Opus 5.5 작성, 09:28 KST GPT 검토·승인(현재 개발 범위의 1회). 실행·회수는 Claude Haiku 5.5 보조가 맡는다. run19의 실패(최종 간격 1.255 m)를 같은 조건에서 다시 시도하는 것이 아니라, 저장 입력에서 찾은 다른 원인 가설 하나를 확인하는 1회 주행이다.

## 바꾼 가설과 이유

- run19 저장 판단에서 측방 위협이 사라진 뒤 중앙 후보는 횡오차 0.4 mm 안팎의 보정에도 0이 아닌 미세 조향(약 0.0001–0.0013 rad)을 냈다. 다음 주기에는 그 **이전각 원호**가 후방 마스크 꼬리 띠 질의 때문에 미지지가 되어 직진을 포함한 전 후보가 거절됐다(13.22, 17.02, 17.04 s). 이어진 정지·역방향 펄스·두 fresh 재개 대기가 앞차와의 간격 손실로 이어졌다는 것이 가설이다.
- 기본 false `ExecutionConfig.small_center_straight`는 위협이 없고 중앙 후보가 촉발된 단계에서, 현재 방향 직진의 후륜축 경로가 계획 거리 전체(최소 0.65 m)에서 기준 경로로부터 기존 직선 추종 오차 예산 22 mm 안이고 곡선 구간이 아닐 때만 **직진을 먼저 검사**한다. 직진도 노면·잔디 경계·측방 간격·경로 위 앞 물체·관측 지지·실제 발행각/이전각 외곽·종방향 간격·두 fresh 재개를 모두 받는다. 거절되면 기존 순서(중앙→유지→직진)로 돌아간다.
- 저장 입력(개루프)에서 run19 자기 조향 반사실의 전 후보 거절 HOLD는 4→1회, 양수 명령 481→499, 0이 아닌 조향의 활성 양수 150→1이다. 위협 기록·복귀 관문 출력은 791판단 모두 OFF와 같다. run17/run21(약 90 mm 복귀)·run24(측방 미촉발)는 출력이 OFF와 완전히 같다. 남은 1회(18.52 s)는 기준 경로가 곡선 판정이라 직진 우선이 적용되지 않은 경우다.

## 고정 조건

- run19와 같은 `stop_restart`, seed 0, 3번 슬롯, `e3_stop_restart_v1.json`의 잔디 쪽 옆차(+0.32 m, −0.50 m, 0.35 m/s). 앞차 0–4 s 0.35 m/s, 4–9 s 정지, 9–15 s 재개.
- M3·명령 재선택 ON·관측 기반 복귀 ON·`corner_distance_candidate`·`per_ray_candidate`·회랑 ON·최근 관측 OFF에 **`--m3-small-center-straight`만 추가**한다. 기본값은 OFF 유지.
- 센서·구동(모터축 28카운트·15:1·65 mm)·노면·간격 예산·0.35 m/s 시험 상한·시간/보호/STOP 경계는 바꾸지 않는다. `/drive` → 독립 보호층 → `/drive/safe` 유지. 정답 자세/actor는 평가 전용이다.
- 실행 전 설치본·launch 해시 일치(run26)와 기존 시뮬레이션 프로세스 0을 확인한다. 실행 중 소스를 바꾸지 않는다.

## 판정과 증거

- 기존 `stop_restart` 판정(정지 유지·재개 지연·재개 뒤 진행·**최종 간격 ≤ 1.2 m**·최소 간격·도로 안·뒤따름)을 그대로 쓴다. 기준을 바꾸거나 보조 평가로 원래 판정을 대체하지 않는다.
- 가설 확인용 진단: 위협 소멸 뒤 전 후보 거절 HOLD 횟수와 시각, `small_center` 적용·직진 선택 횟수, 0이 아닌 조향의 활성 양수, 직진 단계의 최대 |d0|·예측 편차, 복귀 관문 사유, 앞차 이동 중 최장 정지. run19 값과 나란히 적는다.
- 명시 STOP·MCU STOP·밀집 정지·정상 종료·잔존 0·소스 불변·판단 동일 재생·노면 any/center/full 평가를 run19와 같은 순서로 회수한다.
- 원시 증거: `run27_small_center_stop_restart/run/` 전체(판단 기록·자세·actor·접촉·보호/구동 진단·소스 사본), 실행 명령·종료 코드·로그.

## 해석 한계

1회 폐루프 결과로 M3 측방 70 mm 간격·후방 미관측·전체 랩·실차·50 Hz 실시간 성능을 증명하지 않는다. 최종 간격이 통과해도 0.35 m/s 시험 상한과 복귀 보류(가능 영역 미확인)는 남는다. 실패하면 원시 입력과 소스를 보존하고 원인을 분해하며 같은 조건 자동 재시도는 하지 않는다. 전 후보 거절이 줄지 않거나 최종 간격이 나빠지면 미채택으로 남긴다.

```bash
python3 -B scripts/validate_g1_traffic.py --case stop_restart --seed 0 --mode m3 --m3-command-fallback --m3-release-to-baseline --m3-small-center-straight --record-follow-decisions --record-braking-diagnostics --boundary-evidence corner_distance_candidate --occupancy-timing per_ray_candidate --corridor-fallback --actor-overlay config/tests/e3_stop_restart_v1.json --output artifacts/validation/2026-10-25/m3_execution_review_v1/run27_small_center_stop_restart/run
```
