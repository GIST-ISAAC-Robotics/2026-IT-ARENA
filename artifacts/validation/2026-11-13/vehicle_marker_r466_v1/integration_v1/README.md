# r466 v2 공통 통합

승인된 **22파일의 before SHA가 모두 일치**하여 충돌 없이 v2 stage를 정확히 복사했다. 기존 10파일의 원본은 `originals/`에 보존했고 새 12파일도 manifest에 구분했다. 통합 후와 검사 후에도 22파일 모두 stage SHA와 동일하다.

공통 경로를 직접 import하는 Windows 집중 검사 **94 passed, exit0**이다. geometry/실제 합성 pixel 검출/중첩 quad 반례·CLI·TTL/clock/epoch·plan/current/기존 후보 sweep·독립 guard 본문 I/O double·기존 marker observer OFF 동작·VisionProcess 정상/고장 회수를 포함한다. `import_paths.json`에서 stage를 import하지 않았음을 확인한다. Python 파일의 AST 문법 검사도 통과했다. anyio 플러그인의 pytest API 폐기예정 경고1개는 실패가 아니다.

이번 통합에서는 WSL/ROS/Jetson/장치/Gazebo를 실행하지 않았다. [기존 v2 실제 ROS 대표](../execution_v2/run1/result.json)를 재실행하지 않았으며 이관 이후 새 ROS 검증으로 쓰지 않는다. 해당 실행의 정상 runner/child 종료와, 후속 별도 WSL PID 감사 진입 실패는 원 기록 그대로다.

기능은 명시 profile이 없으면 OFF다. 근사 K/FOV·CAD와 원 image stamp 계약을 유지하고 실제 인쇄물 크기/장착 교정/노출 시각의 정확도를 확인한 것으로 쓰지 않는다. 별도 점유는 wall fit/free 입력에 섞지 않는다. 현재 감속·기존 추월 후보 sweep/current/독립 guard까지 연결하며, 마커만으로 새 추월 후보를 시작하는 기능은 아직 없다.

주 담당의 `pass_release_trial.py`, `race_runtime_policy/observation_policy.py`, `current_region.py`, 새 current-route 모듈과 native에는 쓰지 않았다. `PROJECT_CONTEXT.md`, `DEVELOPMENT_PLAN.md`, 활동 문서도 수정하지 않았다. 이번 작업이 변경한 공통 파일은 [integration.json](integration.json)의22개뿐이다.

근거: [통합 영수증](integration.json), [검사 후 SHA](post_check.json), [기계 결과](result.json), [Windows 로그](windows_focus.log), [JUnit](windows_focus.xml). 원 stage/ROS source archive와 v1 실패는 보존했다. 후속 통합·주행 자원은 root 소유다.
