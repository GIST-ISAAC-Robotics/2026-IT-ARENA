# G1/G2 관측 통로 전환의 명시 런타임 연결

2026-10-30. [오프라인 후보3](../g2_handoff_corridor_v1/README.md)의 제한 합성 근거로 `entry_policy=corridor_local_v1`을 연결한다. 기본 `registered_v0`는 그대로이며 G1/G2 전용 시뮬레이션 노드에서만 명시 선택한다.

필수 조합은 `g1_g2_simulation_trial=true`, `use_sim_time=true`, G1 관측 입력·절대 기록 경로와 `g1_g2_entry_policy=corridor_local_v1`, `g1_g2_corridor_policy=corridor_v2_signed_hold`, `g1_g2_preparation_policy=observed_entry_space_v1`, `g1_g2_persistence_window=trim_before_compare_v1`, `g1_g2_follow_entry_space=true`, `g1_g2_reacquire_policy=known_hold_v1`이다. 검증하지 않은 조합은 거절한다.

우회 경로와 같은 편이/전이로 다음 관측 간격의 남은 시간까지 실제 명령/정지 영역을 검사한다. 전체 전이 경로를 미래 비점유로 보장하지 않는다. 정상 준비 중에는 경로 허가와 면 관측 기억을 분리하고, active BYPASS의 음수 피드백은 내부 signed0 대기로 전달한다. 현재 경로에서 상한 명령이 불허되면 실제0.25/0.15/0.08 명령을 다시 검사하되 입력 측정속도·원시 광선·여유/물성은 바꾸지 않는다.

`bypass_handoff.py`의 기본 분기를 보존하고 명시 옵션일 때만 보조 모듈을 사용한다. 노드의 파라미터와 기록에도 선택을 남긴다. `/drive`→독립 보호→`/drive/safe`·STOP은 유지한다. 이전 코드 두 파일은 `before_connection.zip`과 후속 소스 계약에 보존했다.

## 검증 상태

연결 직전 현재 런타임은 옛 Gazebo run1/run2의641/656호출을 재생해 출력 해시를 기록했다. 역사적 원본 일치601/603은 기존 불일치이며 새 연결의 결과로 소급하지 않는다. 관련 회귀·연결 후 기존 출력 해시·오프라인 후보4기록의 새 명시 런타임/기록 어댑터 동일성·빌드 확인을 진행한다. 새 Gazebo는 아직 미실행이다.


## 연결 검사 결과

관련357검사가 통과했다. 기존 두 기록의 현재 출력 해시는 연결 전후 동일하다(원본 일치601/641·603/656의 기존 한계도 같다). 후보3의 네 합성 입력은 각각800/800 런타임 출력이 같고, 새 명시 옵션을 넣은 기록 어댑터의 schema5 재생도 일치했다. 원래 과거 헤더를 새 옵션 사용 이력으로 덮어쓴 것이 아니다. `arena_autonomy` 빌드가 완료됐고, underlay 동명 패키지·bytecompile 비활성 경고는 별도로 남았다. 다음 제한 Gazebo는 [실행 전 등록](../g2_handoff_corridor_gazebo_v1/DRIVE_PLAN.md)에 따른다.
