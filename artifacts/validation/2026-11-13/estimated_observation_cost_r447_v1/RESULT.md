# r447 Jetson 저장 입력 관측 보완 비용

최신 공통 r440/r444/r446을 별도 ARM native 빌드에서 계산한 결과, **estimated 단일 `.1 m/s` 분기의 전체 Evaluator median은 94.40 ms, p95는 96.77 ms**였습니다. 같은 저장 입력의 simulation 원계약은 median 191.61 ms, p95 197.20 ms였습니다. estimated도 **99회 중 3회가 100 ms를 초과**했고 기존 corner 대표는 164.03 ms이므로, 10 Hz 실시간 충족이나 실제 센서 주행 가능 판정으로 확대하지 않습니다.

두 계약 총 **198회 계산을 각각 한 번 수행**했고 같은 ARM의 최종 명령·허가 필드·이유·경로는 99/99 정확히 일치했습니다. 기존 x86 전체 출력 비교는 **8행 실패**를 그대로 보존합니다. 비용 계산 완료와 전체 비교 `passed=false`를 구분합니다. 공통 코드는 변경하지 않았으며 Jetson 소유 작업·센서 포트 잔여 0을 확인한 뒤 반환했습니다.

## 입력과 측정 경계

동결 파일은 241개, Python 구문 검사 230개입니다. r444 이관 SHA와 r446 후속 이관 SHA를 대조했습니다. [prepared.json](prepared.json), [source_manifest.json](source_manifest.json)에 정확한 SHA와 출처를 보존합니다. r437 current 96개 전체와 r426 seq369 정상 양수, seq680 기존 corner, seq614 음수 측정 대표만 사용했습니다.

입력은 **저장 시뮬레이션 수치의 계약 사본**입니다. r444 변환 원문을 재사용하여 estimated 선택에 필요한 `false / host_estimated_v1 / unverified_estimate` 및 scope만 바꿨고, 원시 scan·wheel/history·epoch·기하·측정 속도 수치가 그대로임을 Windows에서 99개 대조했습니다. wheel 값을 실물 측정으로 합성하지 않았습니다.

실행 루트는 `/home/q/it-arena-r447-a`, Python 3.10.12, NumPy 1.21.5, aarch64입니다. fresh native 빌드와 실제 ABI1 import를 확인했습니다. 타이머는 `Evaluator.__call__`의 restore/scene/current recheck/freeze 전체를 포함하고, 입력 hash·직렬화·비교·생성자·프로세스 시작·IPC는 제외합니다. 두 계약은 별도 프로세스이며 **추가 warmup은 0회**입니다. simulation 99회 다음 estimated 99회로 고정했으므로 순서·주파수·온도의 영향을 제거한 교차 실험이 아닙니다.

## 비용과 호출 수

단위는 ms이며 p95는 nearest-rank입니다.

| 범위 | 계약 | n | median | p95 | max | >100 ms |
|---|---|---:|---:|---:|---:|---:|
| 전체 | simulation | 99 | 191.610 | 197.196 | 230.964 | 97 |
| 전체 | estimated | 99 | 94.397 | 96.775 | 164.027 | 3 |
| r437만 | simulation | 96 | 191.867 | 197.196 | 230.964 | 96 |
| r437만 | estimated | 96 | 94.437 | 96.747 | 131.316 | 2 |
| 계약 첫 호출 제외 | simulation | 98 | 191.543 | 197.136 | 230.964 | 96 |
| 계약 첫 호출 제외 | estimated | 98 | 94.391 | 96.747 | 164.027 | 2 |

첫 호출 r437:643은 simulation 206.311 ms / estimated 106.789 ms입니다. r437 Evaluator 생성은 각각 121.266 / 121.031 ms이며 위 표에 포함하지 않았습니다. estimated의 100 ms 초과는 r437:643 106.789 ms, r437:657 131.316 ms, r426:680 164.027 ms 세 건입니다.

| 진단 누계 | simulation | estimated |
|---|---:|---:|
| crop 추가 목표 계산 | 168 | 96 |
| crop 최종 full-region 검사 | 168 | 96 |
| crop 수락 | 72 | 72 |
| corner 추가 목표 / full-region 검사 / 수락 | 1 / 1 / 1 | 1 / 1 / 1 |
| 최종 양수 출력 | 74 | 74 |
| 최종 원0 유지 | 25 | 25 |

계측 중 계산 함수를 감싸거나 바꾸지 않았으며 호출 수는 기존 진단 필드를 합산했습니다. estimated crop 진단 96개 모두 `timing_verified=false`, `host_estimated_v1`, 단일 목표 시도 1회를 유지했습니다. 이 입력집합에서는 추가 crop 목표·전체검사가 72회 줄어든 것과 함께 비용 감소가 관찰됐습니다. 과거 r411의 다른 입력집합에 대한 개선율로 비교하지 않습니다.

## 출력 보존과 실패 분류

같은 ARM의 speed/steering/reason, reference/path source/original reference, path_points, current-route accepted, 독립 guard 요구, runtime authorization 필드는 99개 모두 정확히 일치합니다. **`independent_guard_required=true`는 99개, `runtime_authorized=true`는 0개**입니다. 저장 Evaluator 결과가 실제 운용 발행 허가가 된 것이 아닙니다.

- 정상 양수 r426:369: `.15 m/s` 그대로, 전체 출력도 정확히 동일. simulation 46.251 / estimated 47.080 ms.
- 기존 corner r426:680: `.09332669202147063 m/s`와 최종 이유·허가·경로 동일. simulation 160.873 / estimated 164.027 ms. 명시 estimated 진단 메타데이터가 달라 전체 출력 딕셔너리 동일성은 주장하지 않습니다.
- 음수 측정 r426:614: 원 측정 `-.061225042014404256 m/s` 그대로, 원 `route_current_large_motion_region_unobserved`의 출력 0과 전체 출력 동일. simulation 34.912 / estimated 35.752 ms. 이번 대표에서 음수 복구 성공을 주장하지 않습니다.

x86 대비 정확 트리 일치 0/99, 기존 `1e-10` 비교는 91/99 통과입니다. 실패 8행은 r437 seq655/665/669/719/727/735/799/827이며 기존 비교를 넘는 차이는 **region count 16개, 최대 2**뿐입니다. 나머지 경로/명령/수치 차이의 최대 절댓값은 각각 `1.11e-16 / 1.11e-16 / 4.44e-16`입니다. 이유·selector·boolean 변경은 관찰되지 않았습니다. 허용 오차나 region 판정을 바꾸지 않았고 [원 report의 passed=false](remote_result/cost/report.json)를 유지합니다.

## ROS 입구 확인과 회수

비용 계산 뒤 첫 생성자 단계는 기존 ROS 환경을 source하지 않아 `ModuleNotFoundError: rclpy`, 코드 1로 끝났습니다. 원 [execution](remote_result/execution.json)과 [실패 로그](remote_result/node_check.log)를 보존했습니다. 기존 r404 인계대로 `/opt/ros/humble/setup.bash`를 source하고 **생성자 단계만** 새 `node_check_v2`에서 실행했습니다. 비용 계산은 반복하지 않았습니다.

실제 `LocalPursuit`는 명시 estimated scope를 `registered runtime-capable planner` 오류로 거절했습니다. 실제 `G1G2Node`와 독립 `LidarSafety`는 estimated 선택 및 false/host-estimated 입력 계약을 유지했습니다. spin·센서 주입·host·장치 접근 없이 `/drive` 0 발행 3회만 있었고 양수는 0회입니다. plan/current/vision 자식 3개 모두 exit0, 강제 종료 0이며 [생성자 결과](remote_node_v2/report.json)와 [영수증](remote_node_v2/receipt.json)에 남겼습니다.

ARM 빌드·import·simulation·estimated·summarize는 각각 exit0입니다. 최종 `/proc` 점검에서 소유 잔여 0·센서 포트 0·동결 변경 0, Windows 원본 출처 SHA 변경 0입니다. [최종 회수 점검](final_audit.log), [작은 전체 요약](summary.json)에 근거가 있습니다. 전력·governor·thread 환경은 바꾸지 않았고 기존 `schedutil`을 유지했습니다. 부수 환경 메타데이터는 원 execution에 보존했습니다.

준비 과정의 과거 입력 `timing_policy` 키 생략에 대한 검증 helper 오류 1건과 첫 전송 discovery 실패 1건도 각각 원 기록을 보존했습니다. 전자는 absent-key 복원만 수정했고, 후자는 기존 도구가 유일한 기존 host key를 다시 확인한 주소로 전송했습니다. 어떤 계산 실패를 성공으로 소급하거나 완료된 비용 실행을 반복하지 않았습니다.
