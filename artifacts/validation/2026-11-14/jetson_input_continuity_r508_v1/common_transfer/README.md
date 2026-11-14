# 공통 이관 후보

`common_transfer.patch`는 공통 4파일에 대한 검토용 패치다. 공통 파일은 이 담당이 직접 수정하지 않았다. 원본/후보 SHA는 `manifest.json`에 있다. 혼재 CRLF 때문에 `git apply --check --ignore-space-change`로 검사했다.

1. `lidar_motion_ros.py`: 기존 두 구독 참조를 보관하고 stage의 동일 bounded drain 함수와 `MotionInput.drain_pending`을 추가한다.
2. `g1_g2_node.py`, `lidar_safety.py`: control 진입 직후 drain을 호출하고 기존 함수가 그 뒤 실제 now를 읽는다.
3. `current_route_priority.py`: 현재 기하/점유/전방 관측에서 양수인 결과만 기존 consumer proof 필드로 연결한다. TTL과 consumer 검사는 그대로다.

control과 두 subscriptions가 같은 기본 mutually-exclusive callback group인 현재 구조를 전제로 한다. 병렬 direct 호출을 지원하지 않는다. 최대 64개이며 2 ms는 메시지 사이 검사다. 한 callback 시간만큼 초과할 수 있고 남은 표본을 삭제하지 않는다. SHM 환경 설정은 이 패치에 포함하지 않는다.

대표 회귀는 상위 `test_motion_ingress_candidate.py` 5개 및 `candidate_shm16_drain_proof/test_current_route_proof_candidate.py` 6개다. 후자는 같은 폴더 bootstrap을 통해 고정 설치본 위 독립 후보를 읽어 ARM에서 검사했다. 실제 ROS 검증은 동일 동작의 test-only wrapper/모듈 교체로 수행했다. 공통 이관 형태는 문법과 적용 가능성까지 확인했으며, 공통 적용 후 회귀는 통합 담당이 별도로 수행한다.

후속 통합 담당은 이 4파일의 SHA 대조·공통 반영·신규 회귀 승격과 고유 196검사 통과를 전달했다. 최초 fake MotionInput fixture 누락 실패는 보존하고 fixture만 보완했다. Jetson r500 원본은 그대로이며 새 공통 설치 검증은 후속이다.

효과와 남은 active 1.326초 공백은 상위 `RESULT.md`/`stage_comparison.json`을 따른다. 실물 작동이나 무계측 runtime 완전 통과를 뜻하지 않는다.
