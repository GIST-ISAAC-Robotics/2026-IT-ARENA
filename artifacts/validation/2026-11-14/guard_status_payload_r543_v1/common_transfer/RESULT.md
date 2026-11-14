# r543 공통 이관 및 실행 인계

선택형 guard 상태 축약을 공통에 이관했습니다. 기본 `full`·상태 발행주기·구동 경계는 유지합니다. Windows 집중 검사19개가1.05초에 통과했고, 이후 root가 WSL 독점 자원을 별도 실행 담당에게 인계하여 **실제 ROS 검사2개를 포함한21개가 모두 통과**했습니다(JUnit2.149초, 프로세스 wall4.09초). 정상 종료·소스 불변·잔여 프로세스0을 확인했습니다. 이 구현 담당은 WSL/Jetson에 접속하지 않았습니다.

## 변경 범위와 보존 근거

- 신규 `src/arena_autonomy/arena_autonomy/guard_status_payload.py`: 정적 latch 좌표의 별도 상세 기록·참조·복원, 경고와 full fallback.
- `src/arena_autonomy/arena_autonomy/lidar_safety.py`: 선택 파라미터 두 개·상태 인코더 호출·종료 회수만 연결.
- 신규 `tests/test_guard_status_payload.py`, `tests/test_guard_status_payload_ros.py`: 실제 공통 구현 대상으로 단위/actual control 계약과 선택형 ROS 연결 검사.

통합 직전 소스는 `before/`, 이후 네 파일은 `after/`에 보존했습니다. `manifest.json`에 SHA-256, `r543_common.patch`에 정확한 변경이 있습니다. **상태 발행 전까지 `control()` 전체 AST는 통합 직전과 동일**합니다. guard memory/LocalFollow/current wall 코드·r544·selected_v2 아카이브는 변경하지 않았습니다. 구문 컴파일과 `git diff --check`가 통과했습니다.

| 파일 | 통합 후 SHA-256 |
|---|---|
| lidar_safety.py | `e21a9b3f2332b60173e7d6ac00f4e970438872ac2d67612e82cb92edc5cd4b13` |
| guard_status_payload.py | `54c2e97df27759f48dae417321c87ae14df7c598e5b45a3088cd87df824c51bb` |
| test_guard_status_payload.py | `40dfbcab0f4fa5647831abe325337afb774b2529f484fcabb3035ac44060a2ea` |
| test_guard_status_payload_ros.py | `ec28a7e64aae6557ebe67b12a0d74020d325d9ba7e4ae2240337c212f19e914f` |

## 동작과 Windows 확인

`safety_status_detail=full`이면 archive·파일·writer thread를 만들지 않고 기존 `json.dumps`를 그대로 사용합니다. 실제 control의 양수/0명령/입력 만료/누락/현재 장애물 대표에서 구동 속도·조향·stamp·frame과 상태 JSON 문자열이 기존 full 경로와 같습니다.

명시 `compact_v1`에는 기존 실행별 고유 폴더 안의 새 절대 `safety_status_evidence_path`를 전달합니다. x 모드로 원파일을 보존하며, 같은 경로 재시작·잘못된 경로·초기화 오류는 경고 뒤 full로 시작합니다. 최초 write+flush 확인 전에는 원 좌표를 계속 보내고, 확인 뒤에만 ID/SHA 참조로 전환합니다. 기록 실패·queue drop·인코더 실행 예외는 경고1회 뒤 full로 유지합니다. 경고 출력 자체의 오류도 제어 콜백으로 전달하지 않습니다.

19개 검사에는 실제 flush를 지연시켜 참조가 먼저 발행되지 않는 경계, 쓰기/flush 실패 뒤 양수와 다음0명령 유지, queue drop/경고 실패의 원 full JSON 반환을 포함합니다. partial 반환 상세만 작은 fixture로 고정했으며 projection·관측 감사·sweep·제동·시각·출력은 실제 `control()` 본문을 실행했습니다. ROS 운용과 실제 센서 증명은 아닙니다.

## 별도 실행 담당의 실제 ROS 검사

root가 기존 ROS/overlay와 독점 WSL 자원을 전달했습니다. 저장소 source가 앞서는 PYTHONPATH와 별도 ROS domain223·`ROS_LOCALHOST_ONLY=1`을 사용했으며 설치/재빌드를 하지 않았습니다.

```bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/tests:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
python3 -m pytest tests/test_guard_status_payload.py tests/test_guard_status_payload_ros.py \
  -q --tb=short -p no:cacheprovider \
  --junitxml=artifacts/validation/2026-11-14/guard_status_payload_r543_v1/common_transfer/ros_pytest.xml
```

21개에서 실패/오류/skip 모두0입니다. 실제 ROS 노드 생성·입력 없는0명령/상태 수신·상세 writer 종료·동일 파일 재시작의 full fallback을 확인했습니다. pytest PID38717은 exit0으로 정상 종료했고, 강제종료·source 변경·domain223 잔여 소유 프로세스는0입니다. WSL 자원은 root에 반환했습니다. `ros_pytest.log`·`ros_pytest.xml` 및 상위 `wsl_ros_verification_v1/{preflight,started,result}.json`에 환경·실행·회수 근거가 있습니다. Gazebo/주행/Jetson 실행은 없었습니다.

## 유지되는 한계

원 r543의85.95% 자료 감소와 Windows 반복 인코딩 비용 감소·최초 캡처5.8631 ms는 당시 stage 표본의 결과입니다. 공통 이관과 새 예외 검사를 그 수치의 Jetson/실시간 재검증으로 확대하지 않습니다. 원1,015행 복원·계측은 반복하지 않았습니다. 상세는 상태와 함께 회수해야 하며 file flush는 전원 상실 fsync 보증이 아닙니다.

공통 기능과 비구동 ROS 연결 검사까지 완료했으며 실행 설정의 opt-in 채택·설치·장비 출력·커밋·푸시는 하지 않았습니다. 네 수정 파일의 소유권은 root에 반환합니다.
