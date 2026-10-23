# 사용자 승인 후 선택형 연결의 사전 회귀

사용자가 기본 비활성 선택형 연결과 공식 코스 단독 주행 1회를 승인했다. 이 폴더는 **주행 전 검사만** 수행한다. 주행은 이후 별도 고정 명령/폴더에서 1회 진행한다.

`FollowConfig.corridor_fallback_enabled=false`, ROS `g1_corridor_fallback_enabled=false`를 기본으로 유지한다. true에는 `corner_distance_candidate`와 `per_ray_candidate`를 모두 요구한다. 현재 기하·점유·서로 다른 스캔 확인이 성립하고 기존 사유가 `follow_boundary_unconfirmed`일 때만 대체한다. 나머지 HOLD와 기존 양수 출력은 보존하고 회랑 경로를 기존 벽 기억에 쓰지 않는다. 신호/영상·MCU 허가 및 `/drive`→독립 보호→`/drive/safe`는 그대로다.

전체 pytest 1회와 기존 실패 입력 2,021판단 동일 재생을 수행한다. 실제 재생은 새 옵션 false인 과거 입력을 그대로 사용한다. 전이 의존성과 새 선택값을 snapshot/기록에 포함하고 기존 소스 계약은 새 후속 계약으로 연결한다. 이전 계약/실패를 덮어쓰지 않는다. 관련 빌드/ROS 실행은 아직 하지 않는다. 실패하면 자동 수정/재시도 없이 회수한다.
