# r559 optional 빈 문자열의 ROS CLI 전달 수정 후보

**stage만 준비했다. 공통·r555 원 실행/로그 수정0, Windows9검사 통과, 실제 ROS 미실행이다.** 요청에 언급된 `src/arena_bringup/launch/marker_course.launch.py`는 현재 존재하지 않는다. 실제 실패 launcher는 `r555_sparse_actor_ready_v1/marker_course.launch.py`이며 이 파일을 원본으로 사용했다. 새 실행 폴더의 wrapper로 이관하는 최소 patch이며 root가 최종 적용 위치를 정한다.

## 실제 원인

원 `r555_sparse_actor_ready_v1/run1/simulation.log:29`는 다음 오류를 기록한다.

```text
Couldn't parse parameter override rule: '-p vehicle_marker_profile_path:='
```

같은 파일39행의 실제 bridge 명령도 `--ros-args -p vehicle_marker_profile_path:= -p vehicle_marker_observer_enabled:=False -p vision_execution:=process ...`를 포함한다. Python 노드도 `rclpy.init` 전에 같은 인수 해석 오류로 실패했다. 정확한 원문·행번호·원본/후보 SHA는 `source_evidence.json`에 보존했다.

빈 문자열을 `ParameterValue(..., value_type=str)`로 만든 것만으로는 global `SetParameter`의 ROS CLI 빈 값 표기를 처리하지 못했다. 실패는 마커 검출/제어 판단 이전의 launcher 인수 오류다. 원 시험의 중단·실패 결과는 유지하며 이 후보를 성공으로 소급하지 않는다.

## 최소 변경

`OpaqueFunction`에서 `r537_marker_profile`을 실제 문자열로 평가한다. 빈 값이면 **해당 `SetParameter`만 생략**하고, 비어 있지 않으면 기존 `ParameterValue(profile, value_type=str)` 전달을 유지한다. 임의 dummy 경로·공백 제거·bool 변환·shell 표현식은 추가하지 않는다.

OFF 소비자인 `MarkerChannel`은 `declare_parameter('vehicle_marker_profile_path', '')` 뒤 빈 값이면 즉시 반환하므로 기존 구독/발행 없는 OFF 기본값을 사용한다. `vehicle_marker_observer_enabled=False`, `vision_execution='process'`, launch argument 기본값과 `BASE=A/'course.launch.py'` 연결은 그대로다. 상위 launcher에서 같은 profile을 따로 강제하는 중첩 구성은 이번 원 실행에 없었고 검증 범위도 아니다.

유사 optional 직접 전달은 공통 `g1_follow.launch.py:135`, `r0_plus.launch.py:15`의 `g1_trace_path`에서 확인했다. 둘 다 빈 default의 `SetParameter` 형태이므로 추후 trace OFF 실행에서 같은 종류의 문제가 가능하다. **r555 실제 명령에는 유효한 trace 파일 경로가 있었으므로 이번 실패 원인은 아니다.** 이 두 공통 파일은 수정하지 않았으며 추가 launch 옵션 전수 변경도 하지 않았다.

## Windows 근거와 root 인계

`test_optional_launch_parameter.py`: **9/9 통과,0.19초**. 원본에서 빈 override가 생성되는 조건, 기본/명시 OFF 생략, ON 문자열 보존(공백/따옴표 포함), bool 타입/vision/include 불변, 실제 `MarkerChannel` 기본 OFF의 구독·발행0을 확인했다. launch/ROS 동작 대역 검사이므로 실제 RCL parser 통과를 대신하지 않는다.

`marker_course.launch.py`를 새 실행 레시피에 복사하면 그 폴더의 `course.launch.py`를 포함한다. 원 r555 파일을 덮어쓰지 않는다. 최소 diff는 `optional_marker_parameter.patch`, 원본은 `marker_course.launch.py.before`, 결과는 `pytest.log`/`pytest.xml`이다. 새 stage와 신규 regression 파일만 작성했다.

## 필요한 native 확인 한 번

root/runner가 WSL 자원을 받은 뒤 다음 **한 명령**을 실행한다. `native_parameter_smoke.py`는 실제 launcher의 include만 제외하고 실제 `launch_ros.actions.Node`가 만든 global 인수를 별도 parameter 선언 노드에 전달한다. 한 호출에서 OFF/ON 각1회 확인하며 시뮬레이터·제어기·센서·구동 topic publisher는 생성하지 않는다. 각 child는 선언/값 확인 후 node destroy와 shutdown으로 종료한다.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=224
export PYTHONDONTWRITEBYTECODE=1
timeout 40s python3 artifacts/validation/2026-11-14/r559_optional_launch_parameter_v1/stage/native_parameter_smoke.py --output-dir artifacts/validation/2026-11-14/r559_optional_launch_parameter_v1/native_smoke_v1
```

실행 전 domain/WSL 소유권은 root가 배정한다. 예상 산출물은 `off.json`, `on.json`, `result.json`이며 실제 child `--ros-args`·빈 override 부재·OFF 빈 기본값/ON 정확 경로·bool false/process를 확인한다. parent exit0과 남은 소유 프로세스0을 함께 회수한다. 40초 초과 또는 오류 시 자동 재시도하지 않고 소유 child를 회수한다. **이 native 명령은 아직 실행하지 않았으며 실행기 자체도 Windows AST 확인만 했다.**
