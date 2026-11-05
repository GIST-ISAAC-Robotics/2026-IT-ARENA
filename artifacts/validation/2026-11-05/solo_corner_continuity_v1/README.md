# 43.61 m 실제 정지 — 최소 개선 설계 v1 (재개점)

- 상태: **미완료 재개점으로 마감**(2026-11-05 13:33 KST, 사용자 정리 요청). 설계안은 가설이며 구현·저장 입력 재생·반례·기하 대조는 하지 않았다. 런타임 공용 트리·Gazebo는 건드리지 않았다. 마감 시 실행 중인 재생·시뮬 프로세스는 없었다(`pgrep` 결과 없음). 격리 사본은 13:32에 다시 검증해 121/121 일치했다.
- 대상: [solo_straight_closed_loop_v1/run1](../solo_straight_closed_loop_v1/README.md)(`--straight-budget`, 31.850 s)에서 43.61 m 0.138 s 실제 정지. 원인 구조 분석은 [diagnosis_v1](../solo_jr60p20_baseline_v1/diagnosis_v1/README.md)을 그대로 쓴다(반복하지 않음).
- 격리 사본: `~/arena_scratch/corner_continuity/src`(WSL). 121개 파일이 폐루프 등록 해시(834개 중 해당 패키지)와 일치한다(`isolate.sh` → `isolate.log`). 저장 직전 GPT가 작업트리의 `moving_pass.py`에 r16을 적용했으므로, **WSL 격리 사본 안의 파일만** 폐루프 등록 묶음(`registered_untracked_sources.zip`)의 원본으로 맞췄다. 저장소 작업트리의 r16은 유지했다. 단독 경로는 이 파일을 쓰지 않는다.

## 확인한 사실 (폐루프 run1 판단 기록, 센서 입력만)

| 시각 s | 측정 속도 m/s | 출력 m/s | 사유 | 부분 관측 |
|---|---:|---:|---|---|
| 37.24–37.28 | 0.89 | 0.91–1.0 | 회랑 다리 | 완전 관측 |
| 37.30 | 0.98 | **0** | `follow_release_wait`(다리→기본 경로 재개 확인 틈, 후보 A 대상) | 완전 관측 |
| 37.32–37.40 | 0.98→0.08 | 0.5 | `local_follow` | `bounded_side_missing` 1광선 0.72°, 상한 0.5 |
| 37.42–37.56 | 0.0–0.57 | 0.5 | `local_follow` | `bounded_side_missing` 3광선 2.16°, 상한 0.5 |
| **37.58–37.60** | 0.33 | **0** | `follow_forward_gap_too_wide`(HOLD) | 같은 3광선 2.16°가 운동 보정 뒤 정면 ±30°로 들어옴. 양옆 반환 ≥ **8.52 m**, 요구 거리 0.6 m |
| 37.62–37.70 | 0.0–0.16 | **0** | `follow_release_wait`(fresh 2회 확인) | **완전 관측**(공백 소멸) |
| 37.72~ | 0→ | 0.89–1.04 | `local_follow` | 완전 관측 |

- 계획기 쓸림 여유는 같은 시간 0.44–0.72 m(실제 벽)였다. 기본 목표(`target_speed_before_hold`)는 0.5–1.0 m/s로 양수였다. 독립 보호층은 `clear`였다.
- 측정 속도가 37.38 s부터 0.0–0.3 m/s로 떨어진 것은 37.30 s의 0명령 뒤 부모 증속 재시작(1.5 m/s²)과 0.5 m/s 상한이 겹친 결과로 보인다(diagnosis_v1의 증폭 구조와 같음, 추정).
- 같은 방향 무반환은 기준 run1에서 기하 대조상 12 m 밖 벽(12.08–12.29 m)이었다(`diagnosis_v1/raycheck.json`). 이번 기록의 기하 대조는 아직 하지 않았다.

**구분:** 37.30 s의 0명령은 현재 경로 근거가 유효한 상태에서 생긴 근거 전환 틈이다(후보 A, 보류 중). 37.58–37.70 s의 0명령 0.14 s는 실제 무반환(짧은 정면 공백)에서 생긴 정지다. 이 폴더는 뒤의 것을 다룬다.

## 기존 기능으로 잇는 설계안 (가설, 미검증)

`PartialObservationPolicy.evaluate`에는 이미 **좁은 정면 공백을 0.2 m/s 상한으로 잇는 경로**(`bounded_forward_missing`)가 있다. 조건은 다음 세 가지다.

- 총 공백 ≤ 1.5°
- 공백 양옆 실제 반환 ≥ max(1 m, 요구 제동 거리)
- 직전 스캔의 근거리 점이 공백 안으로 사라지지 않았음(래치)

이번 공백은 양옆 반환·래치 조건을 만족했고 각도 조건(2.16° > 1.5°)만 넘었다.

- **안:** 각도 대신 이미 v4 측방 공백에서 쓰는 **거리 기준(요구 반경에서의 공백 현 길이 ≤ 0.05 m)**을 정면 공백에도 같은 보수성으로 적용한다. 대상은 위의 기존 0.2 m/s 경로뿐이다. 이번 값: (2.16°+0.72°) 폭, 반경 0.6 m에서 현 약 0.030 m.
- **필수 제한(시간·진행):**
  - 연속 2스캔(≤0.2 s) 또는 그 사이 진행 ≤ 요구 거리까지만 허용한다. 공백이 지속되면 기존처럼 HOLD한다.
  - 직전 관측 근거리 점의 소실 래치, 실제 발행·직전 조향의 이동 영역 검사(`near_sweep`), 독립 보호층은 그대로 둔다.
  - 무반환을 거리·자유 공간으로 채우지 않는다.
- **기대 효과(검증 전 추정):** 37.58–37.60 s의 HOLD가 0.2 m/s 상한으로 바뀐다. 그러면 HOLD 플래그와 그 뒤 재개 대기 5호출이 생기지 않아 0명령 0.14 s → 0 s가 된다. 부모 증속도 0이 아니라 0.2에서 다시 시작한다. 37.30 s의 근거 전환 0명령은 그대로 남는다(후보 A 범위).

## 다음 재개 단계 (계획)

1. **재생:** `isolate.sh` 사본에 정책 하위 클래스를 메모리로만 넣는다. 폐루프 run1(`solo_straight_closed_loop_v1/run1`)과 기준 run1의 저장 입력으로 OFF 동일(기록 일치)을 확인한다. ON에서는 바뀐 호출(0명령 창·사유·조향 0 변화)과 부모 발행 모델을 본다. 명령값과 차체 결과는 구분해 적는다.
2. **반례 2개(합성 스캔):**
   - 공백 안 근거리 점이 직전 스캔에 있다가 사라진 경우 → 래치·HOLD 유지.
   - 같은 공백이 3스캔 이상 지속되는 경우 → HOLD 유지.
3. **기하 대조:** 이번 기록의 무반환 3광선을 `diagnosis_v1/raycheck.py` 방식으로 확인한다. 12 m 밖이 아니면 설계를 다시 본다.
4. 결과와 미적용 패치 초안을 이 폴더에 남긴다. 단순 배치 실행은 Haiku에 맡긴다.

## 파일

| 파일 | 내용 |
|---|---|
| `isolate.sh` → `isolate.log` | 격리 사본 고정·검증(121/121 일치) |

## 재개 명령

```bash
# WSL, 이 폴더에서. 격리 사본이 없으면 현재 작업트리를 복사하므로 r16 파일의 해시 검사가 실패할 수 있다.
# 이 경우 등록 ZIP의 src/arena_autonomy/arena_autonomy/moving_pass.py를 읽어
# $HOME/arena_scratch/corner_continuity/src/arena_autonomy/arena_autonomy/moving_pass.py에만 복원한다.
# ZIP: ../solo_straight_closed_loop_v1/registered_untracked_sources.zip
# 저장소 src/의 moving_pass.py는 절대 되돌리지 않는다. 격리 사본을 맞춘 뒤 아래 검증을 다시 실행한다.
cd "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-05/solo_corner_continuity_v1"
bash isolate.sh
# 이후 '다음 재개 단계' 1–4. 재생은 PYTHONPATH=$HOME/arena_scratch/corner_continuity/src/arena_autonomy:$HOME/arena_scratch/corner_continuity/src/arena_vehicle_interface
```
