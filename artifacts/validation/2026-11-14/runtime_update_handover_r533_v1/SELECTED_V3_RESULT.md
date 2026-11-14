# selected_v3 source-only 생성 완료

**후속 선택 정정 — 현재 실행 권장 대상 아님 / 미채택:** 추가 Y갈림길 반례에 따라 이 ZIP에 포함된 r549 초기 opposite fallback을 철회했다. 아래 생성·검증 기록과 ZIP/manifest/기존 acceptance는 당시 결과로 보존한다. 현재 선택 상태는 별도 [상태 기록](SELECTED_V3_STATUS.json)을 따른다. revision2는 짧은 선호 벽을0.1m/s 상한 내에서 차체 길이까지 활용하며 반대 벽 자동 전환을 제거했다. revision2의 r550 실제주행은 진입 예정이며 완료로 표기하지 않는다. v4는 root가 주행 결과를 확인하고 요청할 때만 생성한다.

2026-11-14, GPT. label **`r549-r543-common-native42-ros21`**. r550 새 단독주행은 이 소스 선택 시점에 미실행이며 본 묶음의 실행 성공 근거가 아니다.

- [ZIP](selected_v3/runtime_handover_sources.zip): 소스736파일,737멤버(내부 manifest 포함). SHA256 **`125cb4efbed5032984bb490c7ec6048a8b9608d8570dedf696e14636379746b7`**.
- [manifest](selected_v3/manifest.json) SHA256 **`59a3002b07695d8cbf45c73aed7c86747eff6d12a940a7b70ddcb5576677077e`**.
- root 지정 r549 `current_wall_reacquire.py=db429c5a...`, r543 `lidar_safety.py=e21a9b3f...`, `guard_status_payload.py=54c2e97d...`가 묶음과 완료 시점 원본 모두 일치한다. 수집 중 변경은 검출되지 않았다. 코드 변경·전체 시험 재실행 없이 기존 prepare의 CRC·멤버/manifest SHA 검사를1회 수행했다.
- 실측 영점2811·운영2600–2980counts, 대칭 중앙 상한0.28475436568260193rad, vehicle 내륜 상한0.30666234131856146rad과 `handover/steering_launch_selection.py`를 포함했다. 오른쪽43counts 미사용·바퀴각 CAD 환산 구분은 유지한다.
- **guard `safety_status_detail` 기본 full**, compact는 별도 명시 선택이며 이번 overlay에서 선택하지 않았다. 기존 planner의 `g1_trace_detail=compact_v1` 후보와 다른 설정이다. 실행/설치 완료를 뜻하지 않는다.
- v2 대비 [변경6경로](selected_v3/delta_from_selected_v2.json): 위 제어3경로, launch helper 추가, 선택 안내 변경, 다른 세션 `scripts/probe_esp32_motor.py` 변경. probe의 수집 SHA는 `0aefbacab31293567a8e051040236753009a39e4fb71caaa9939a33ad0331e45`이며 **원형 snapshot만 포함하고 이번 운영 선택/검증으로 채택하지 않았다**. 원파일을 수정·복구하지 않았다.
- 실제 board/PI/포트 입력은 null, private board 생성0. **install_pending=true; Jetson·장비·WSL·설치·flash·출력0**. 기존 v2 ZIP `92aa7039...`와 manifest `167abb7d...`, r500 원본은 보존했다.

전체 SHA·상태는 [인수 영수증](selected_v3/acceptance_receipt.json), r500 대비 변경은 [delta](selected_v3/delta_from_r500.json), 생성 출력은 [로그](selected_v3_prepare.log)를 따른다.
