# r446 estimated planner 입구 계약 — 독립 stage

2026-11-13, GPT. r444 최종 정독에서 확인한 일반 `LocalPursuit` 실행 파일의 입구 누락을 보완했다. stage 검증 후 주 담당이3파일을 검토·공통 이관했다([이관 영수증](integration.json)). 실물 기본 활성화는 하지 않았다. Windows stage만 수정했으며 WSL/ROS/Jetson/장치·주행은 사용하지 않았다.

## 변경

`LocalPursuit` 클래스의 `estimated_observation_runtime_capable`은 기본 false, `G1G2Node`의 명시 override만 true다. 일반 planner에서 `g1_observation_input_scope=host_estimated_crop_v1`을 선택하면 scope를 읽은 직후 `ValueError`로 거절한다. crop 플래그가 꺼졌다고 이 입구를 건너뛰지 않으며, 뒤의 부분관측 정책/vision 자식 설정 전에 거절한다. 기존 `WallFollow` 기반 ROS 초기화 이전에 거절한다는 뜻은 아니다.

capability는 G1/G2의 진입만 열어 준다. 이후 기존 `validate_selection`과 `validate_corner_selection`이 실제 native2slot/r396 구성을 검사하는 코드는 그대로다. capability=true만으로 inline/python·기존 submission·reference 미보존·recheck 미선택·runtime off·corner off·verified 위장을 수용하지 않는다. 클래스 이름 문자열 검사나 기본 scope/속도/시간 정책 변경은 없다. `lidar_safety`의 독립 guard 선택과 r444의 입력/proof/0.1 후보 로직도 수정하지 않았다.

이관 대상은 아래 **3파일**뿐이다. [원본·stage SHA](changes.json), [적용 diff](apply.patch)를 기준으로 주 담당이 이관한다.

- [local_pursuit.py](stage/src/arena_autonomy/arena_autonomy/local_pursuit.py): capability=false와 선택 직후 입구 검사.
- [g1_g2_node.py](stage/src/arena_autonomy/arena_autonomy/g1_g2_node.py): capability=true 선언. 기존 실제 runtime 검증은 불변.
- [test_estimated_observation_entry.py](stage/tests/test_estimated_observation_entry.py): 신규 의미 검사6개.

## 검증과 한계

Windows Python에서 **6개 통과, 실패/skip0**([로그](verify_windows_v1/unittest.log)). 일반 planner 거절, 두 클래스의 기존 simulation 입구 보존, G1 estimated 진입 뒤 실제 validator 수용/반례, 독립 guard 선택을 확인했다.

ROS 없는 Windows 환경이므로 시험은 **stage 클래스의 실제 capability 선언·scope 선언/거절 AST와 subclass의 기존 validator 호출 AST를 추출해 실행**한다. 조건을 시험에 따로 재작성하지 않았으며 실제 공통 validator 함수로 잘못된 runtime/r396 조합8개를 거절했다. ROS 노드 전체 생성자나 자식 준비·종료를 실행한 검증은 아니다. r444의 이전 실제 ROS 생성자 결과를 r446 새 입구의 실행 확인으로 소급하지 않는다.

두 공통 원본은 작업 전후 SHA 동일하며, 독립 guard와 참조한 r444 helper/runtime 파일 SHA도 변경 목록에 기록했다. `git apply --check`와 stage3개 AST 구문 검사를 통과했다. 이번 수정은 입력 선택 입구만 보완하며 전체 저장409개 비교·주행·센서 시험을 반복하지 않았다. 필요하다면 다음 ROS 구성 확인에 일반 LocalPursuit 거절/G1 정상 생성 두 경우만 함께 확인할 수 있다.
