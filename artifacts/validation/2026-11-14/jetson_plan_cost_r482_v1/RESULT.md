# r482 Jetson plan 중복 이동 영역 계산 제거

2026-11-14, GPT. **공통 미이관 stage 후보**이며 root가 최종 통합을 소유한다. 저장된 시뮬 입력만 Jetson에서 계산했다.

가장 느린 `static_side_current_lane` 세 행에서 full Evaluator plan 중앙값을 **약20% 줄였다.** 229행은 **243.048→190.428ms**, 232행은 **209.255→167.486ms**, 235행은 **199.093→159.765ms**다. 동일 ARM 원본/후보 48회와 별도 profile warmup2회에서 명령 및 전체 중첩 출력이 모두 원 ARM 결과와 정확히 일치했다. **100ms 초과는 여전히 남으며 전체 ROS 10Hz 또는 실차 주행 성능의 증거가 아니다.**

## 원인과 변경

- 최초 가설인 512질의 묶음의 최대 각도 창 확장은 주원인이 아니었다. 229행 최대 창은 248/504광선이고 전체 광선 fallback은 0이었다. 창·수치 커널은 수정하지 않았다.
- 실제 229행에서는 동일 10,604질의·scene·padding·cycle·분류 모드로 이동 영역을 세 번 계산했다. [독립 배열/scene 대조](duplicate_input_check.json)로 원 입력의 동일성을 확인했다.
- `fresh_parallel_speed_trial.proposal`은 측정속도가 설정 상한보다 높으면 최종 target/steering의 `measured_region`을 이미 계산한다. `CurrentLaneSelector`가 바로 뒤에서 같은 영역을 다시 계산하던 부분만 재사용했다.
- 동일 `faster_proposal` 호출의 반환만 사용하며 전역 캐시·이전 프레임·다른 parallel 결과를 저장하지 않는다. 측정속도와 `self.lat == lateral_config(self.follow_config)`를 확인하고, 결과가 없거나 설정이 다르면 기존 계산을 수행한다. 같은 호출의 scene·이전 조향·dt·최종 target/steering은 생산부와 소비부 사이에 변경되지 않는다.
- 호출자의 `region_blocked` 판정은 그대로 수행한다. 광선·질의·점유 패딩·임계값·속도·차량 기하·숫자 정밀도는 바꾸지 않았다.
- 변경은 [패치](candidate.patch)의 `src/arena_autonomy/arena_autonomy/current_lane_preference.py` 한 파일이다. before SHA `e320696af3ee9b6a21c4ffc0ac9bb66a2630abcaecb08a9211fb43cb2f0e6b45`, stage SHA `f3d2b6d20c10c873b5f42ea47d688e56fe218f3582d8c525c50c52d997c8afb5`.

## 같은 ARM 비용 비교

원본은 r476의 동결 소스, 입력은 r476 plan217개 중 여섯 느린 행과 앞뒤 정상 두 행이다. 동적 제어 상태를 원 순서 prefix98회에서 보존했다. 해당 상태를 원본/후보별 새 프로세스에서 복원하여 **8행×2버전×3회=48회** 비교했다. 버전 순서는 base/stage, stage/base, base/stage이며 경쟁 부하는 없었다.

| sequence | 원본 중앙/p95/max ms | 후보 중앙/p95/max ms | 중앙 감소 |
|---|---:|---:|---:|
| 220 | 84.968/85.358/85.402 | 85.915/87.511/87.688 | -1.12% |
| 229 | 243.048/243.953/244.054 | 190.428/192.686/192.936 | 21.65% |
| 232 | 209.255/211.524/211.776 | 167.486/167.793/167.827 | 19.96% |
| 235 | 199.093/200.054/200.161 | 159.765/161.297/161.468 | 19.75% |
| 238 | 122.370/122.591/122.615 | 122.218/122.327/122.339 | 0.12% |
| 241 | 117.535/117.625/117.635 | 116.918/117.177/117.206 | 0.52% |
| 244 | 135.708/137.230/137.399 | 135.098/137.191/137.423 | 0.45% |
| 247 | 88.530/90.416/90.626 | 88.339/88.723/88.765 | 0.22% |

표의 p95는 선형 보간이며 사례당 표본3개뿐이다. 선택8행 전체24회 기준 p95는 242.874→190.406ms, max는 244.054→192.936ms이나, 느린 행을 의도적으로 고른 집합이므로 r476 전체217행 p95의 개선율로 환산하지 않는다. 선택 집합의 100ms 초과는 양쪽 모두18/24회다.

측정은 `Evaluator.__call__` 전체이며 상태 복원·constructor/import·별도 IPC/process 시작은 제외했다. 첫 round의 229행 profile1회/버전은 warmup으로만 사용했고 wall 통계에 포함하지 않았다. 이후 round는 별도 프로세스이므로 첫 timed call에 남는 지연 초기화 비용은 포함된다. 원시각·명령 만료/발행 상태를 재현한 ROS 처리율 시험은 아니다.

warmup cProfile의 229행에서 `motion_region` 3→2회, `motion_queries` 3→2회, `depth_support` 7→6회로 줄었다. 누적 profile 비용은 각각 175.564→117.945ms, 80.649→54.489ms, 107.256→76.174ms다. 이 값에는 profiler 오버헤드가 있으므로 실제 wall 비용으로 합산하지 않는다. [기계 요약](summary.json)과 로컬 보존 `compare_result/compare_results/`에 원 결과가 있다.

peak RSS는 원본 프로세스 57,864–59,144KiB, 후보 57,856–59,904KiB였다. RSS는 프로세스 전체 최고치이며 개별 allocation 절감량을 측정한 값은 아니다. 후보는 일시적인 lateral config/budget 값만 비교하고 기존 region 객체를 재사용한다. 별도 대규모 캐시·배열을 추가하지 않으며 대표 행에서 10,604×2 float64 질의 배열(169,664byte)과 관련 분류/기하 임시 배열의 중복 생성을 한 번 피한다. 전체 임시 배열 메모리 상한을 이 수치로 주장하지 않는다.

## 동등성·회귀·실패 보존

- 같은 ARM에서 **48/48 전체 JSON raw exact**, profile warmup **2/2 exact**다. speed/steering뿐 아니라 region 계수·가림·점유/clearance/padding 상세도 포함한다. 허용오차를 넓히지 않았다.
- 기존 current-lane 회귀16개와 [재사용 경계4개](tests/test_current_lane_region_reuse.py), 합계 **20 passed**. launch 인자/header 파일 문항1개는 변경 범위 밖이어서 명시 제외했다. 재사용 결과 없음·측정속도 다름·사용자 지정 기하에서는 실제 재계산과 거절이 유지된다.
- 초기 v1은 동적 `StructureAwareRecoveryPolicy` 표준 pickle 실패로 prefix88계산 후 중단했다. profile/성공 계측0이며 [실패 회수 영수증](profile_failure_receipt.json)과 원 archive를 보존했다. 88회는 실제 계측 성공 건수에 넣지 않는다.
- harness 전용 직렬화 보완 후 v2 prefix98회 중 선택8개를 profile했고, 복원된8개 비계측 출력과 원 ARM 출력이 모두 정확히 일치했다. 동적 class의 base/config/state만 저장하고 후보 코드를 원본 code closure로 대체하지 않았다. 운영 코드는 직렬화 때문에 변경하지 않았다.
- 별도 진단2회도 원 출력 exact다. r476에서 보존한 x86/ARM current2행 계수 차이를 수정·재판정하지 않았다.

## 범위·회수·실물 연결

기존 `scripts/jetson_access.ps1` 및 `isaac-jetson.local`의 기존 키/hostkey를 사용했다. ARM Python3.10.12, 기존 r470 native binary SHA `5654cd2f52af838b52384a084f159bb1319925db6690945b4403cb7ae33f5bff`를 C++/setup exact 대조 후 그대로 복사했다. 빌드·패키지 설치·센서/포트/ROS/전력/장치/네트워크 설정 변경은 없다.

격리 경로는 `/home/q/it-arena-r482-a`. pytest 및 비교 자식7개 모두 exit0, 강제종료false, 잔존 process group/PID0이다. [실행 영수증](execution_receipt.json), [소스 영수증](source_receipt.json)에서 원본/후보 소스와 기존 native 불변을 확인했다. 로컬 원본478파일과 현재 공통 소스도 대조 시 차이0이었다. root가 별도로 통합한 Wi-Fi 도구 및 SSH 선택형 stdin 확장은 이 source 범위 밖이며 이 담당의 수정이 아니다.

**Jetson 소유권은 root에 반환했다.** 이후 장치 재접근 없이 Windows에서 결과를 정리했다. 공통 적용·실제 주행·새 성능 행렬은 수행하지 않았다.

[실물 estimated 코드 대조](HARDWARE_PATH.md): r444/r446 명시 진입에서도 같은 selector/proposal을 사용한다. 다만 재사용은 측정속도>FollowConfig 상한에서 기존 measured-region이 계산된 경우에만 활성화된다. 실물 기본 개발속도0.3m/s 상태의 개선으로 확대하지 않는다. `timing_verified=false`/`host_estimated_v1`을 바꾸거나 가짜 실물 입력을 만들지 않았다.

## 재현·통합 참고

원본에 `candidate.patch` 적용 또는 stage의 해당 한 파일을 복사하기 전에 before SHA를 대조한다. [테스트](tests/test_current_lane_region_reuse.py)는 공통 `tests/`에 이관하면 기존 `test_current_lane_preference.py` fixture를 재사용한다. 본 단계에서는 공통을 변경하지 않았다.

실제 수행한 명령은 `python3 -B execute_profile_v2.py`, `python3 -B execute_diag.py`, `python3 -B execute_compare.py`이며 각 실행 wrapper·manifest·원 archive를 이 폴더에 보존했다. wrapper는 같은 출력 경로가 있으면 재실행을 거절한다. 비교 안의 pytest 명령은 다음과 같다.

```text
PYTHONPATH=stage/src/arena_autonomy:stage/src/arena_vehicle_interface \
python3 -B -m pytest -q -p no:cacheprovider \
 stage/tests/test_current_lane_preference.py stage/tests/test_current_lane_region_reuse.py \
 -k 'not explicit_launch_option_and_recorded_header'
```

비교 결과 archive SHA: `25a802203eab85a2dfd02abab80abdc8fb0f7e274b67386bfc4e3d560316db99`. 원본·후보 전체 소스 및 입력·원시 profile은 로컬 Git 제외 자료로 보존하며, 삭제하거나 과거 실패를 성공으로 소급하지 않는다.
