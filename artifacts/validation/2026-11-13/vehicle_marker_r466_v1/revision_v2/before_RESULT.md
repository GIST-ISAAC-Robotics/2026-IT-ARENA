# r466 인계 결과

선택형 metric footprint부터 기존 plan/current 및 독립 guard 소비까지 격리 stage를 구현했고 Windows 검사를 통과했다. 이후 **실제 ROS 대표 1회는 정면 마커 감속 실패로 끝났으며 사용자 요청으로 일시중지, 공통 미통합**이다. 실물 계측은 하지 않았다. 최신 재개 지점은 [PAUSED.md](PAUSED.md)다.

OpenCV 4.6이 같은 정면 ID1을 중심 동일한 79 px/68.007 px 중첩 quad로 반환했다. 기존 2 px 동일성 조건에서 `ambiguous_duplicate`가 되어 빈 점유 packet을 발행했고, 정면에서도 원 .75 m/s가 유지됐다. 원 ROS import 2개는 통과했지만 대표 전체는 실패다. 중복 처리 수정은 아직 적용하지 않았고 추가 ROS 재시험도 없다.

- `windows_tests_v11.log`: 197 passed, 2 deselected, exit 0. 제외 2개는 rclpy가 필요한 기존 노드 import 검사다. 실제 Windows VisionProcess spawn의 정상 close/비정상 child 회수도 기존 검사로 통과했다.
- Windows에서 실제 LocalFollow/M3 후보 점유 함수와 Evaluator plan/current 연결, guard 본문+I/O double을 확인했다. 이후 실제 VisionProcess/ROS packet 전달·guard timer는 동작했으나 위 중복 검출 문제로 정면 cap은 실패했다.
- ContextVar는 한 plan 호출에만 설정·복구한다. 빈 입력/OFF 결과, 기존 path/선택 벽/원시 scene 보존, 중첩·예외 복구를 검사했다. source integrity의 18개 원본 항목은 모두 그대로다.
- 별도 cloud는 기존 sweep 결과에 추가 제한만 한다. 마커가 안 보이거나 불확실할 때 새 정지 상태를 만들지 않고 기존 LiDAR 판단을 유지한다. 보조의 최대 수명은 원 image stamp부터 .25 s이다.
- 가상 차체는 추정 15×20 cm이며, K/FOV·CAD와 인쇄 크기 근거를 명시한 approximate profile이다. 거리/각도 보장 오차·설치 교정·노출 시각 동기화는 미검증이다.
- 공식 actual image 기준 크기와 사전을 구분했다. DGIST PNG 11 vs 주최 표 14 충돌은 남겼고, 자기 GIST 49는 runtime에서 제외했다. 전체 50 mm를 일괄 검은 코드 50 mm로 넣지 않는다.
- 파일 기반 이미지 측정/1~2거리 간이 조정 CLI를 제공한다. 미측정 칸은 null이며 새 output만 생성한다. 실측값을 만든 적이 없다.

**기능 한계:** 마커만 보고 새 추월을 개시하는 후보 생성은 아직 없다. 현재 단계는 감속·기존 추월 후보의 충돌 대조 및 최종 명령 제한이다. 단독 완주나 전체 추월 요구를 대체하는 결과가 아니다.

이관 범위는 `changes.json` 21파일이며 `original_sha256.json`, `stage.diff`, `stage_source.zip`으로 고정했다. archive SHA256은 `23ac47b62f40d53a755acf40748d09f0482c74eb8a4670a8feba3ea410004363`, diff SHA256은 `5b94db3c9c215ea0e2699126f636f3ee98dd659a534d75c214336a686eb651e6`이다.

최종 `execution_v1/pause_pid_audit.json`에서 원 runner/영상 child 모두 absent, r466 명령줄 매칭 0개다. 영상 child 정상 join, ROS 3노드 destroy를 기록했고 WSL을 root에 반환했다. Jetson은 사용하지 않았다. 명시 재개 전 추가 실행/수정은 하지 않는다. 원 실패/중간 harness 오류 로그와 이전 Windows 결과(`RESULT_windows_v11.md`)를 보존했다.
