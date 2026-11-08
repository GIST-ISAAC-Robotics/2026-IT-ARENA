# r41 코너 기록의 판단 비용 분해

**r41 기록 1,628판단의 계측 없는 PC 재생 비용은 p50 28.3–29.2 ms, p95 44.5–47.6 ms였다(두 실행). 판단 함수가 약 25 ms(평균)를 차지하고, 그 가운데 `PassReleaseSelector._select` → `threat_view`가 평균 12.5 ms다. 상위 5% 꼬리는 `motion_region`(나타난 169판단에서 평균 26.8 ms)이 만든다.** 공통 전처리 `motion_scene`은 평균 4.7 ms이며 r25에도 같은 크기로 있다. 아래 좁은 후보 세 개는 명령을 바꾸지 않을 수 있지만 줄이는 폭이 작다. 차감 추정으로는 평균이 30.3 ms에서 약 27.8 ms로 줄고, 20 ms 초과 판단 수는 거의 그대로다. 20 ms 근처까지 내리려면 격자·장면 표현을 다시 설계해야 하며, 이는 GPT 주 에이전트가 판단할 일이다.

- 범위: 기존 r41 기록 오프라인 재생 2회(아래 B·C). 각 실행 안에서 계측 없는 기준 재생 A를 한 번 먼저 했다. Gazebo·ROS·Jetson은 실행하지 않았고 `src/`·설정·보호층·임계값과 이전 증거도 바꾸지 않았다. 비용은 WSL2 x86_64 단일 실행 값이며 Jetson·ROS 실시간 마감으로 환산하지 않는다.
- r41은 [주행 실패 사례](../current_corner_support_v1/README.md)다. 이 문서는 판단 비용의 분해일 뿐 주행 근거가 아니다.

## 기준선과 입력

| 항목 | 값 |
|---|---|
| 입력 기록 | `artifacts/validation/2026-11-08/current_corner_support_v1/run1/follow_decisions.jsonl` (Git 제외), SHA-256 `43bdc149…6a86555`. r42 manifest와 같고 실행 전후 해시도 같다 |
| 판단/reset | 1,628 / 381. 헤더 schema 5, `g1_g2_handoff_decision_stage_only`, `g1_g2_handoff_trial`, `bounded_local_v4`. 전체 판단이 handoff FOLLOW 상태 |
| 상태 구성 | r42 [`bundle_parity.replay_once`](../jetson_bundle_refresh_v1/bundle_parity.py)를 import해 그대로 기준 재생 A로 썼다. B·C 루프도 같은 헤더 검사, `BypassHandoffFollow` 생성, seq 순서, reset, footer 검사를 따른다. 각 판단은 `profile_follow_trace.timed_call` 1회이며 warmup은 없다 |
| 소스 | 실제 로드된 프로젝트 모듈 67개가 r42 **v3** manifest와 모두 같다([source_check.json](source_check.json)). 실행 후 디스크 재해시도 67/67 같았다. v1 manifest와는 `scripts/profile_follow_trace.py` 하나만 다르며, 이는 v3 CLI 수정(after `f3dbc219…`)이다. 제어기 runtime은 r41/r42와 같다 |
| 환경 | Python 3.12.3, NumPy 1.26.4, Linux 6.18.33.2-microsoft-standard-WSL2, x86_64, 32 CPU. `python3 -I -B`(`sys.flags.isolated=1`), PYTHON·ROS·AMENT 환경 0개 |
| 실행 | [run_profile.sh](run_profile.sh)(A+B), [run_timers.sh](run_timers.sh)(A+C). 둘 다 Claude Haiku 5.5 보조(`model='haiku'`, 자체 보고 `claude-haiku-5-5`)가 각 1회 실행했고 종료 0이었다. 분석은 Claude Opus 5.5가 했다 |

## 명령 동일성

| 비교 | 결과 |
|---|---|
| A(계측 없음) ↔ r42 `commands_repo.json` | 두 실행 모두 1,628/1,628 동일. 판단 스트림 해시 `051d7364…b1b4f`가 r42와 같다. 원 기록 출력과의 차이 0 |
| B(cProfile) ↔ A, ↔ r42 | 동일. 첫 차이 없음 |
| C(구간 타이머) ↔ A, ↔ r42 | 동일. 첫 차이 없음 |

명령 비교 항목은 seq·속도·조향·사유·판단 해시이며 r42와 같다. `commands_repo.json` SHA-256은 `52d71b44…`다.

## 측정 방법

- **B. cProfile**: `timed_call` 구간에서만 `enable/disable`했다. 호출 구조, 호출 수, 자체/누적 시간을 얻었다. 그러나 cProfile은 광선마다 함수를 부르는 전처리 구간을 크게 부풀렸다. p50 기준 projection은 18.1 ms(계측 없음 약 4.5 ms, 약 4배), decision은 28.8 ms(약 26 ms, 약 1.1배)였다. 그래서 cProfile 시간은 구조와 반복 호출을 찾는 데만 썼다. 예를 들어 tottime 1위권인 `_finite` 2,000만 회는 실제 비용보다 크게 보인다.
- **C. 구간 타이머**: 판단당 수십 회 이하로 불리는 27개 함수만 `perf_counter_ns` 래퍼로 감쌌다. 함수 객체를 참조하는 모든 모듈 전역과 클래스 속성을 같은 래퍼로 바꿨다. 래퍼 호출 수는 B의 ncalls와 모두 같았다(예: `relative_poses` 9,970, `local_path` 9,370, `compact_return_mask` 2,662). 따라서 빠진 호출 경로는 없다. 중첩 스택으로 포함 시간과 자체 시간을 나눴다. C의 total은 p50 30.9 ms, 평균 30.3 ms로, 같은 실행의 A(p50 29.2 ms)보다 약 1 ms 높다. 이 차이는 래퍼 부하와 실행 간 변동이 섞인 값이다.
- **측정 밖 보조 측정**: `timed_call`이 반환된 뒤 실행했으므로 total에 들어가지 않는다. 같은 스캔 각도에서 원래 `rear_masked`의 반복 비용, `blind_spec` 반복 비용, 사전 파싱 원형의 비용을 쟀다. 원형과 원래 함수의 판정이 같은지도 확인했다. 또 `ReturnGrid`와 `ObservationGrid`의 compact 결과가 같은지 비교했다. 원형은 이 폴더 스크립트에만 있고 `src/`에는 넣지 않았다.
- 포함 시간은 같은 부모 아래 형제끼리만 더했다. 중첩된 누적 시간은 합산하지 않았다.

## 비용 트리 (C, 판단당 평균 ms, 포함 시간)

| 구간 | 평균 | p95 | 호출 수 | 비고 |
|---|---:|---:|---:|---|
| `restore_call` | 0.24 | 0.31 | 1,628 | **재생 전용**(기록 → 객체 복원). 실제 노드 비용이 아니다 |
| `motion_scene` (전처리) | **4.74** | 5.57 | 1,628 | r25에서도 같은 크기다 |
| └ `scene_from_scan` | 4.27 | 4.90 | 1,628 | 500광선마다 Python `SensorRay` 생성·검증·마스크 |
| 　 └ (보조) 광선별 `rear_masked` | 1.78 | | 500/스캔 | 이 가운데 매 광선 `blind_spec` 재파싱이 **1.33** |
| └ `relative_poses`, `audit_front_observation` | 0.26, 0.13 | | | |
| `BypassHandoffFollow.command` (판단) | **25.34** | 41.44 | 1,628 | `_command` 자체 0.63 |
| └ `ReadyInitialReference.update` | 2.63 | 3.52 | 1,628 | `local_path`·`relative_poses` 포함 |
| └ `LocalFollow.command` | 21.73 | 37.91 | 1,628 | |
| 　 └ `PassReleaseSelector._select` | **15.43** | 32.33 | 1,628 | |
| 　 　 └ `threat_view` | **12.54** | 18.24 | 1,628 | 경로가 없는 218판단에서는 None |
| 　 　 　 └ `ObservationGrid.__init__` | **7.70** | 11.46 | 1,252 | 측방 반환이 있을 때만 만든다. 생성한 판단 평균 약 10.0 ms(p50 9.6). 전체 판단 평균 기준 자체 6.52(광선 자유공간 래스터화) + compact 1.18 |
| 　 　 　 └ `cell_coordinates` | 2.75 | 4.00 | 1,252 | 격자 전체 경로좌표 보간(`lerp`) |
| 　 　 　 └ `ReturnGrid.__init__` | 1.44 | 1.89 | 1,410 | 대부분이 compact 1.35 |
| 　 　 　 └ `side_return_masks`, `side_threats` | 0.26, 0.14 | | | |
| 　 　 └ `motion_region` | 2.32 | | 176(169판단) | 나타난 판단에서 **평균 26.8, p50 27.5, 최대 54.6 ms** |
| 　 └ `_observed_command` | 5.59 | 7.79 | 1,628 | `_path_command` 3.47(`CorridorObserver` 2.10, `_baseline_command` 1.36), `StructureAware.evaluate` 1.89 |
| 　 └ `ParallelLaneTrial.apply` | 0.67 | 1.02 | 1,628 | |

- **꼬리**: total 상위 5%(82판단, p95 46.5 ms 이상) 가운데 79판단에 `motion_region`이 있다. `motion_region`을 빼면 p95는 37.1 ms다([tail_analysis.txt](raw/tail_analysis.txt)). B 기준 `motion_region` 내부에서는 `masked_only`(질의 점마다 500광선을 numpy로 도는 Python 루프. 172회에 질의 81,405개, 회당 약 470개)가 누적의 38%를 차지했다. 이어 `motion_queries`(`body_queries`·`np.unique`)가 39%, `depth_support`가 23%다. 이 내부 비율은 cProfile 값이다.
- **고정 비용**: r42 parity 값에서 total−decision p50은 r25 4.93 ms, r41 5.03 ms로 같은 크기다. 따라서 r25 참조 재생은 새로 하지 않았다. p50 차이는 엄밀히 더할 수 있는 값이 아니다.
- **반복 작업**: ① `blind_spec`을 광선마다 다시 파싱한다(1,628×500회). ② `threat_view` 안에서 `ReturnGrid`와 `ObservationGrid`가 같은 장면의 반환 배열, 나이, compact 묶음을 두 번 계산한다(1,252판단, 결과 1,252/1,252 동일). ③ `local_path`를 판단당 약 5.8회 부른다(평균 2.72 ms). 그러나 입력이 같은지는 확인하지 않았으므로 중복 여부는 미확인이다. ④ `relative_poses`를 판단당 6.1회 부른다(1.24 ms).

## 좁은 최적화 후보

절감량은 C 측정값을 판단별로 빼서 얻은 추정이다([counterfactual_estimates.json](counterfactual_estimates.json)). 실제 구현 후 측정한 값이 아니다. 후보는 모두 `src/` 변경이므로 새 소스 계약(provenance 후속)과 관련 회귀, 이 기록의 명령 동일성 재검사가 필요하다.

| 후보 | 근거(측정) | 예상 절감 | 의미 위험 |
|---|---|---|---|
| **C3. `scene_from_scan`에서 `blind_spec`을 한 번만 파싱** (첫 광선에서 지연 파싱하고, 파싱한 값을 받는 비공개 마스크 함수 사용) | 광선 루프 1.78 → 0.43 ms, 1,628스캔 모두 판정 동일 | **모든 판단 약 1.35 ms**(r25 등 같은 센서 섹터 사용 경로 공통). 평균 30.3→29.0, p95 46.5→45.2 | 낮음. 같은 식·순서를 유지한다. 잘못된 spec의 예외는 첫 광선에서 같은 순서(자세 검사 → 각도 → spec)로 나야 하며, 광선 0개일 때 검증하지 않는 동작도 보존해야 한다. 공개 `rear_masked`는 유지한다 |
| **C2. `threat_view`에서 `ReturnGrid`의 반환·나이·compact를 `ObservationGrid`에 넘겨 재사용** (선택 인자, 기본 경로 불변) | `ObservationGrid` 안 compact 평균 1.53 ms(생성 판단), 결과 1,252/1,252 동일 | 생성 판단마다 약 1.5 ms(+반환 목록 생성 일부). C2+C3 평균 30.3→27.8, p50 30.9→28.1, p95 46.5→44.6 | 낮음. 같은 `scene`·`cfg` 객체에서 나온 순수 계산이다. 다른 호출자의 `ObservationGrid` 생성은 바꾸지 않는다. 시각 사이 캐시는 두지 않는다 |
| **C1. `masked_only` 질의 루프를 묶음(예: 128점) 벡터화** (`ExecutionFrame.project`와 같은 방식) | `motion_region`이 상위 5%의 79/82, 나타난 판단 평균 26.8 ms. `masked_only` 몫은 cProfile 비율로 약 38% | **꼬리 전용 추정**. `motion_region`이 25% 줄면 p95 약 39.6, 50% 줄면 약 36.0 ms. 평균 영향은 0.7 ms 이하다. `masked_only`만으로 몇 ms가 줄지는 구현 후 측정해야 한다 | 중간. `arctan2`·`hypot`·`where`·`inf`의 원소별 식은 같은 비트를 낼 수 있다. 다만 `argmin`의 첫 인덱스 동률, `.025` 비교, `indices` 차이와 `status`·`depth` 조건의 순서를 그대로 보존해야 한다. 묶음 크기에 따라 메모리가 든다(470×500×8 B 배열 여러 개) |

**결론:** C2·C3는 안전하게 할 수 있지만 평균 효과가 각각 약 1.2–1.4 ms에 그친다. 20 ms 초과 판단은 1,382에서 1,377 정도로만 준다(추정). C1은 꼬리(p95)에만 의미가 있다. 20 ms 목표에 실제로 영향을 주는 덩어리는 다음 세 가지다.

- `ObservationGrid` 래스터화: 생성 판단 평균 약 10 ms
- `cell_coordinates`: 약 3.5 ms
- 광선별 Python 객체 장면: 약 4.3 ms

이 셋은 근사·범위 축소·표현 변경 없이는 줄이기 어렵다. 측방 판정의 FREE 칸 의미가 바뀔 수 있으므로 좁은 후보로 제시하지 않는다. 어떻게 할지는 GPT 주 에이전트의 설계 판단에 맡긴다.

## 산출물

| 파일 | 내용 |
|---|---|
| [profile_r41.py](profile_r41.py) / [run_profile.sh](run_profile.sh) | A+B 실행기(SHA-256 `d75f646e…`) |
| [timers_r41.py](timers_r41.py) / [run_timers.sh](run_timers.sh) | A+C 실행기(SHA-256 `aa89d55a…`). 대상 함수 목록 `TARGETS` |
| [profile_result.json](profile_result.json) | B: 동일성, 단계 분포, 로드 모듈 67개 해시, 환경 |
| [timers_result.json](timers_result.json) | C: 함수별 포함·자체 분포, 부모→자식 합계, 보조 측정, 래퍼 설치 참조 |
| [summary.json](summary.json) ([make_summary.py](make_summary.py)) | 압축 트리·동일성·추정 요약 |
| [counterfactual_estimates.json](counterfactual_estimates.json) ([counterfactual.py](counterfactual.py)) | 차감 추정 |
| [source_check.json](source_check.json) ([source_check.py](source_check.py)) | v1/v3 manifest 대조 |
| [tail_analysis.py](tail_analysis.py) → [raw/tail_analysis.txt](raw/tail_analysis.txt) | 꼬리 구성 |
| `raw/r41_timed_call.pstats`(137 KB), `raw/r41_pstats_{tottime,cumulative,callers}.txt` | B 원본 pstats와 정렬 출력 |
| `raw/r41_timers_per_call.jsonl`(7.4 MB), `raw/r41_per_call_profiled.jsonl`(0.4 MB) | 판단별 원시값. 기존 `artifacts/validation/2026-11-0[78]/**/*.jsonl` 규칙으로 Git 제외됨을 확인했다 |
| `raw/*_run.log`, `*.exit` | 실행 로그와 종료 코드(둘 다 0) |

실패한 시도는 없었다. pstats와 텍스트 출력(합계 약 0.3 MB)은 Git 제외 대상이 아니다. 크기가 작아 보존 대상으로 두었고, 공용 `.gitignore`는 이 작업의 쓰기 범위 밖이라 고치지 않았다.

## 남은 확인

- 후보는 구현하지 않았다. 구현하면 새 소스 계약을 등록하고, 관련 회귀와 r41·r25 명령 동일성 재생을 다시 해야 한다.
- Jetson(ARM, Python 3.10)에서의 비율은 측정하지 않았다. numpy 비중이 큰 `ObservationGrid`와 Python 비중이 큰 `scene_from_scan`의 비율은 Jetson에서 달라질 수 있다.
- `local_path` 반복 호출의 입력이 같은지는 확인하지 않았다.
- 반복 벤치마크는 하지 않았다. A의 두 실행 차이(p50 28.3 / 29.2 ms)가 이 PC의 실행 간 변동 크기를 보여 준다.
