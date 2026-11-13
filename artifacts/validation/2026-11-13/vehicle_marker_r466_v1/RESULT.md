# r466 v2 인계 결과

**후속 공통 통합 완료:** [integration_v1](integration_v1/README.md)에서22파일 SHA 일치·Windows94검사 통과를 확인했다. 아래 v2 대표 당시의 공통 미통합 표기는 당시 상태이며, ROS/장치 검증을 추가한 것은 아니다.

선택형 차량 마커의 중첩 quad 처리를 수정했고, **Windows 관련 32검사 및 실제 ROS import 2검사·5.2초 대표가 통과**했다. 공통 소스는 미통합이며 WSL 소유권은 root에 반환했다. 원 v1 실패·아카이브는 그대로 보존했다.

- 원 OpenCV 4.6의 79/68 px 중첩 검출은 크기비·중심·IoU로 묶은 뒤 원 RGB의 검은 코드 border·bit·경계 표본으로 대표를 선택한다. 큰 quad 자동 선택은 아니다. 실제 ROS packet에서도 두 후보 중 하나가 지지되고 원 79 px 코드 외곽이 선택됐다.
- 실제 `VisionProcess → MarkerChannel → LidarSafety.timer`에서 부재 `.75 m/s`, 정면 `.067812~.20 m/s`와 `vehicle_marker_braking`, 측방 `.75 m/s`, TTL 만료 뒤 `.75 m/s`를 확인했다. 마지막0도 관측했다. RGB20개/packet20개, wheel/gyro 각517개, 거절0이다.
- 같은 실행 묶음에서 exit0, child 강제 종료 없이 exit0/join, 3노드 destroy, 생존 reported child `[]`, 232개 입력 불변을 확인했다. **후속 별도 `/proc` 감사 호출은 `Wsl/Service/E_UNEXPECTED` exit1로 진입하지 못했다. 전역 잔류 PID0을 별도 재확인했다고 쓰지 않는다.** 재호출·재시작 없이 반환했다.
- 프로젝트 `numeric native_v1`은 실행 소스/명령상 미선택이다. OpenCV 4.6의 detector/PnP는 사용했다. native build·Jetson ARM 비용·10 Hz 통합 부하 검증이 아니다. 추가7×7/PnP/중복 처리는 `stages_ms.markers` 밖이지만 전체 `duration_ms`에 포함된다.
- 단일 정면 `.2~.5 m` 표본의 간이 초점거리 CLI 계약을 확인했다. 실측값을 채우거나 절대 거리 정확도를 주장하지 않았다. 기존 추월 후보의 점유 검사까지 연결하며 **마커만으로 새 추월 후보를 시작하는 기능은 범위 밖**이다.

최신 근거: [v2 상세](revision_v2/README.md), [기계 결과](revision_v2/result.json), [실제 ROS 결과](execution_v2/run1/result.json), [회수](execution_v2/completion.json), [추가 감사 실패](execution_v2/final_pid_audit_failure.json), [최종 SHA 확인](revision_v2/final_seal.json).

이관은 [22파일 변경 목록](revision_v2/changes.json)과 [v2 stage archive](revision_v2/stage_source.zip)를 기준으로 한다. 이번 v2 runtime 수정은 geometry 한 파일이며 [v1 대비 diff](revision_v2/delta_from_v1.diff)를 제공한다. 이전 문서·상태는 `revision_v2/before_*`와 `PAUSED.md`에 남겼다. 실제 차량·설치 교정·완주 검증의 증거로 확대하지 않는다.
