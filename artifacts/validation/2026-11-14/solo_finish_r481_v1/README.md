# r481 단독 변형 출발 정체의 중복 측방 정지 분리

원 r477 `start_left20_yaw_plus3`는 최대 41.96m 진행 후 정체했고 사용자 중지 요청으로 회수됐다. 원 미완주/STOP 결과는 변경하지 않는다. 이 폴더는 저장 센서 입력 분석과 격리 소스 비교이며 새 주행 결과가 아니다.

## 원인과 변경 범위

최초 지속 정지 계획 `(generation=355, sequence=1293, reference_s=51.22)`에는 현재 경로 36점, 관측 경로 1.898222m, swept clearance .80m, 보류 전 목표 .80m/s가 있었다. 그 뒤 부분 관측 정책에서 측방 결측의 환산 폭 .062552162m가 기존 .05m 문턱을 넘어 `lateral_metric_outside_trial`로 0이 됐다. 선택기는 이를 `parallel_hold_baseline_unavailable`로 전달했다. 따라서 r474의 현재 경로 우선 선택은 이미 0이 된 baseline 때문에 실행되지 않았다. 실제 후속 current 또는 독립 guard가 최초 0을 만든 경우와 구분한다.

`local_follow.py`의 `_partial_command`에서 원 `_path_command`가 경로·점유·제동 계산을 마친 직후, 기존 명시 r474 운용 선택이면 `current_route_priority.observed_command`로 판단한다. 현재 경로가 양수이고 원 입력 무결성이 성립하면 별도 측방 gap veto를 중복 적용하지 않는다. 나중에 실패 reason을 양수로 바꾸거나 입력/상태를 덮지 않는다. 새 임계값·속도 상한·per-ray 예외표는 만들지 않았다.

원 경로/점유 계산의 0, 전방 반환 전체 소실, 경로 소실, 입력 metadata/near-invalid 및 STOP 경로를 보존한다. 원 ray/finite occupancy/ReturnGrid/free 및 측방 추적 상태를 수정하거나 미관측을 free로 채우지 않는다. v2는 기존 requested/evaluate의 입력·순서·기억 갱신을 그대로 수행하고, `_hold` 부작용에 들어가기 전에 미리 선택한 현재 경로 명령을 반환한다. 측방 선택기 상태와 최종 경로가 실제 순서 재생에서 어떻게 달라지는지는 결과 비교에 별도로 기록한다.

변경은 stage의 `local_follow.py`, `race_runtime_policy/current_route_priority.py` 두 파일이다. r477 동결 archive에서 복사한 231개 파일 중 나머지 229개는 동일하다. 원본 2파일, 원 archive/trace SHA, diff 및 변경 SHA를 보존한다. 공통 소스·상위 문서는 수정하지 않는다.

## 실행과 근거

- `changes.json`, `change.patch`: 원본/후보와 최소 diff.
- `prepared.json`: stage 231개, 원본 2개, helper/native/trace SHA 및 고정 명령.
- `windows_tests_v1.log`, `windows_tests.xml`: Windows 7개는 기존 2차원 `np.cross`와 설치 NumPy의 호환성 오류로 중단, STOP 2개 통과. 이를 후보 실패나 전체 검사 통과로 바꾸지 않는다.
- `native_run_v1/`: baseline 계획 540/current 854 exact. v1 후보는 기억 갱신까지 생략해 기존 양수 144개가 0으로 바뀌었으므로 미채택이다. 첫 current unsupported에서 비교를 중단하고 exit1을 보존했다.
- `stage_v1_failed.zip`, `stage_v1_preservation.json`: 실패 초안 소스/helper/해시 보존. 결과를 성공으로 소급하지 않는다.
- `native_run_v2/`: 기존 기억 갱신을 유지한 후보만 재생하며, 이미 exact인 v1 baseline 계산은 반복하지 않는다. 최종 판정은 `RESULT.md`와 각 summary/원 outputs를 따른다.

WSL 전용 자원 배정 후 고정 명령:

```powershell
wsl.exe -d Ubuntu-24.04 -- bash '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-14/solo_finish_r481_v1/execution_v2/run_once.sh'
```

스크립트는 결과 폴더가 있으면 중복 실행하지 않는다. 빌드·설치·ROS/Gazebo·장치 포트 접근은 없다. 원 기록의 parent 감속 cap은 raw 근거를 보존하고 child 계산 결과와 분리해 비교한다. 동일 current 계산의 재전달은 input/원 child 출력/계산시각의 동일성을 확인해 한 번만 계산한다.

실제 새 발행·가속 회복·완주는 이 저장 비교로 입증하지 않는다. 정체 후 새로운 양수 계획의 같은 입력 current 예는 실제 후속 current 기록이 없는 별도 연결 진단이다. 이후 공통 통합 및 주행 여부는 주 담당이 결정한다.
