# r559 후속 — 기록 OFF 일반 시작의 빈 trace 인수

**stage 완료, 공통·r560 동결 입력 수정0, Windows11검사 통과. 실제 ROS/WSL 실행은0이다.** 기존 마커 수정 stage와 별도인 `trace_stage/`에만 작성했다.

## 문제와 변경

`g1_follow.launch.py`와 `r0_plus.launch.py`는 `g1_trace_path` 기본값이 빈 문자열인데 global `SetParameter`를 무조건 만든다. r555 실제 실패에서 확인된 빈 global 값의 ROS CLI 형식 `name:=`와 같은 경로다. r555는 trace 경로를 실제로 설정했으므로 원 실패 자체는 마커 profile 때문이었으며, 이번 trace OFF 실패는 Windows 코드 재현과 같은 직렬화 경로에 근거한 판단이다. 실제 일반 trace OFF ROS 시작은 아직 실행하지 않았다.

두 파일에 `_trace_path_parameters(context)`와 `OpaqueFunction`을 추가한다. 실제 `g1_trace_path`가 빈 값이면 parameter action을 만들지 않고 소비자의 빈 기본값을 사용한다. 유효한 경로이면 **원래의 `SetParameter(name='g1_trace_path', value=LaunchConfiguration('g1_trace_path'))` 자체를 그대로 반환**한다. 새 타입 변환·경로 정규화·dummy 파일·자동 기록은 없다.

`LocalPursuit`는 `declare_parameter('g1_trace_path', '')`를 사용하고 `create_follow_trace('')`는 기록 객체 없이 `None`을 반환한다. 이 실제 함수도 AST 대역으로 실행해 확인했다. 명시 M3처럼 기록을 필수로 요구하는 기존 조건은 변경하지 않았다.

## 보존과 검사

Windows **11/11 통과,0.13초**다. 두 launcher의 기본/명시 빈 trace 생략, 일반/공백/따옴표 경로의 전체 parameter·타입·launch argument 동일, 소비자 OFF 동작을 검사했다. optional 분기를 원래 action으로 되돌린 전체 AST가 각 원본과 같아 다른 launch argument·namespace·`GroupAction(scoped=False)`·bool·include는 그대로임을 확인했다.

`manifest.json`에 원본/후보 SHA, `*.before`에 원본, `trace_optional_parameter.patch`에 두 파일 최소 diff, `pytest.log`/`pytest.xml`에 결과가 있다. 정식 신규 regression 후보는 `test_optional_trace_launch_parameter.py`다. 현재 이 검사는 같은 stage 사본을 대상으로 하며 공통 채택 시 대상 경로를 공통 launcher로 연결한다.

## r560 종료 뒤 필요한 native 확인 한 번

root/runner가 WSL 독점권을 받은 뒤 아래 한 명령으로 두 launcher×OFF/ON 네 가지를 검사한다. 실제 `GroupAction`과 `SetParameter` 및 `launch_ros.actions.Node`의 인수 직렬화를 사용한다. 하위 include만 제외해 Gazebo/제어기/센서 노드를 시작하지 않으며 trace 파일도 열지 않는다.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=224
export PYTHONDONTWRITEBYTECODE=1
timeout 60s python3 artifacts/validation/2026-11-14/r559_optional_launch_parameter_v1/trace_stage/native_trace_smoke.py --output-dir artifacts/validation/2026-11-14/r559_optional_launch_parameter_v1/native_trace_v1
```

최종 domain과 실행 시점은 root가 배정한다. 수용 조건은 parent exit0, `result.json`의4개 case 통과, 실제 argv의 `g1_trace_path:=` 빈 항목0, OFF 기본값 `''`, ON 지정 문자열 정확 일치, g1/r0 bool 모드 보존, child 정상 종료와 소유 PID0이다. 60초 초과/오류 시 자동 반복하지 않고 소유 child를 회수한다. 실행기는 Windows AST 확인만 완료했으며 native 검사는 **r560 종료 뒤 별도 인계 대기**다.
