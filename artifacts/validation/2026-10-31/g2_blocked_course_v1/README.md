# 정지 유지 코스 v1: 실행 전 설정 실패

2026-10-31. 검사5건·고정본1,124파일·사전검사는 통과했지만 실제 Gazebo는 시작되지 않았다. 신규 `blocked_full_course`에 붙인 옆차 overlay의 `base_case`가 이전 `stop_restart_full_course`여서 `traffic_actors.overlay_case`가 거절했다. 원시 `run1_trial.log`와 소스ZIP·단계별 실패를 보존했다. `run1/`과 주행/판단 기록은 생성되지 않았고 잔류 프로세스0이다.

기존 래퍼가 설정 실패 뒤에도 재생·평가를 호출하여 없는 입력의 `FileNotFoundError`가 추가 기록됐다. 차량 실패나 재생 불일치가 아니다. [v2](../g2_blocked_course_v2/PLAN.md)는 별도 옆차 설정의 case 연결과 실제 `overlay_case`·`actor_specs` 검사를 사전검사에 추가하고, 입력이 생성되지 않으면 후속 분석을 건너뛴다. 물성·보호·주행 판정선은 바꾸지 않는다.
