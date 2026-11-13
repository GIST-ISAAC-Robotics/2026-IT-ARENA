# r488 최신 공통에서 잔여 plan 비용 재확인

2026-11-14, GPT. **이전 r482의 160–193ms 병목은 최신 r481v2+r482 공통의 같은 저장 상태에서 재현되지 않았다.** 느린 세 행이 각각 **71.926 / 72.193 / 69.429ms**로 계산됐으며, 큰 `motion_region`·`motion_queries` 호출은 모두0이었다. 새 후보 구현·공통 수정·추가 성능 행렬은 하지 않았다.

## 수행과 실제 결과

r482에서 검증한 원 ARM 입력/직전 controller 상태를 그대로 사용하고 코드만 최신 공통으로 동결했다. generation360의 느린229/232/235와 정상220에 대해 profile1회·비계측1회씩, 총8회 실행했다. 비계측값은 full `Evaluator.__call__` wall 시간이며 상태 복원·constructor/import·프로세스 시작은 제외했다. 아래 값은 **사례당1회**이고 p95/일반 처리율을 추정하지 않는다.

| sequence | 최신 wall ms | 최신 speed m/s | 최신 steering rad | selector 이유 |
|---|---:|---:|---:|---|
| 220 정상 | 73.318 | 1.5 | .000651606480 | `current_route_priority_v1` |
| 229 | 71.926 | 1.5 | .000501621648 | `current_route_priority_v1` |
| 232 | 72.193 | 1.5 | .000031188431 | `current_route_priority_v1` |
| 235 | 69.429 | 1.5 | -.000731714858 | `current_route_priority_v1` |

profile/비계측 **전체 출력4/4 exact**, 원 입력4/4 불변이다. 옛 r482 출력과는4행 모두 전체 딕셔너리가 다르다. 정상220의 명령은 같지만 진단이 달라졌고, 나머지3행은 기존 `.9387749579855957 m/s`의 `static_side_current_lane` 대신 위 현재 경로 명령이 선택됐다. **이미 공통 이관된 r481의 정책 효과**이며, 이번 작업의 출력 동등 최적화 또는 r482 자체 개선율로 발표하지 않는다.

동일 저장 직전 상태로 현재 함수의 비용을 분리한 시험이다. 최신 코드로 전체 prefix/랩을 재생한 후의 상태·미래 센서 관측·발행 결과와 같다는 뜻은 아니다. 이전 r482 행별3회 중앙값과 이번1회 값을 섞어 정밀 개선율을 계산하지 않는다.

## 병목 판단

229행의 기존 r482 후보 profile은 `motion_region`2회/누적117.945ms, `motion_queries`2회/54.489ms, `depth_support`6회/76.174ms였다. 최신 profile은 `motion_region`0·`motion_queries`0, `depth_support`4회/14.711ms다. profiler 오버헤드가 포함된 누적값이며 wall 통계에 합산하지 않는다.

r481이 `_partial_command`에서 기존 현재 경로/점유 판단을 먼저 선택하므로 중복 측방 관측 정지를 제거하는 동시에 이 대표의 큰 이동 영역 재계산도 피한다. r482의 같은 호출 내 영역 재사용 코드는 그대로 유지했다. 이번 입력에서는 그 재사용 이전에 무거운 분기 자체가 필요하지 않았다.

남은 비용은 `native_geometry._geometry`·경로·측방 증거 계산 등에 분산돼 있다. 요청한 큰 병목이 사라진 상태이므로 질의 밀도/수치/보호/속도/기하를 바꾸거나 작은 최적화를 계속 시도하지 않았다. 같은 입력을 성공할 때까지 반복하지 않았다.

## 소스·종료·보존

- 최신 소스244파일 묶음과 기존 상태4개를 격리 `/home/q/it-arena-r488-a`에 보존했다. r481 v2 두 파일 및 r482 한 파일의 최종 SHA와 일치한다.
- 기존 r470 ARM native binary를 C++/setup exact 확인 후 그대로 재사용했다. 빌드·설치0, 공통 소스 변경0, 후보 patch0.
- child PID7335 exit0, 강제종료false, process group 잔존false, 최종 소유 작업0. 원/동결 source 변경0 및 기존 native 불변을 확인했다.
- 센서/포트/MCU/ROS/Gazebo/WSL 접근0, 전력·네트워크 설정 변경0. 저장 계산 전송에 기존 SSH만 사용했다. **Jetson은 root에 반환했다.**
- [요약](summary.json), [소스 확인](source_check.json), [실행 영수증](execution_receipt.json), [동결 manifest](manifest.json), [실행 함수](run_profile.py)와 로컬 `remote_result/profile_results/`의4개 원 profile/출력에 근거가 있다.

회수 archive SHA: `6a6a5427ea2bfc2dbf63ace198737fba9026ae3518bdff8fc30ece432db76769`. 이전 r482 원 결과·실패·소스는 보존했다. 이4회 비계측 계산을 전체 ROS10Hz 또는 실물 주행 성능으로 확대하지 않는다.
