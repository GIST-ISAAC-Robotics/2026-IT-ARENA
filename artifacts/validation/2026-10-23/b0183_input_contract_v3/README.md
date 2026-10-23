# B0183 입력 형식·영상 작업자 연결 검사 v3

2026-10-23. **73통과·실패/skip 0·exit 0·소스 해시 차이 0**. [tests.log](tests.log), [junit.xml](junit.xml), [실행기](run_checks.sh), [전](sources_before.sha256)·[후](sources_after.sha256) 해시를 보존한다. 전체 소스 snapshot은 로컬 `source_snapshot.tar.gz`이며 Git 제외다.

- 새 14회귀: 1280×720 RGB/BGR×행 패딩 0/64 B의 픽셀/인식 일치 4조건, 잘못된 형식/행/본문/크기 거절 7조건, 변환 없이 이름만 바꾼 영상의 픽셀 불일치, 실제 spawn 작업자에 5개 혼합 형식 영상의 신호/ID 49 전달, 공유 버퍼 초과 거절·기존 본문 불변.
- 기존 영상 시각/세대·고장 고정/초록 상태·프로세스 종료와 기본 인식 회귀도 함께 실행했다. 실제 child process를 사용한 검사마다 종료를 확인했다.
- 실제 CSI/Argus/ROS 토픽·Jetson·노출/FPS/검출 거리 시험은 아니다. 파이프라인의 기본 timeout/약 10 Hz 제한·버퍼 용량·운영 코드는 유지했다.
- [v1](../b0183_input_contract_v1/README.md)의 xacro 누락과 [v2](../b0183_input_contract_v2/README.md)의 workspace 경로 누락은 수집 실패로 보존한다. 같은 4개 test 파일을 기존 Jazzy+overlay+소스 우선순위 환경에서 실행한 결과다. 패키지를 새로 설치하지 않았다.

재현 시 기존 증거를 덮어쓰지 말고 새 출력 경로를 준비한다. 이 검사는 `bash`에서 `/opt/ros/jazzy/setup.bash`, `install/local_setup.bash`를 source한 뒤 `src/arena_autonomy`, `src/arena_vehicle_interface`, `scripts`, `tests`를 상속 `PYTHONPATH` 앞에 추가한다.

해석은 [B0183 이관 §5](../../../../docs/sensors/B0183_INPUT_HANDOFF_2026_10_23.md#5-1023-pc-입력-형식과-실제-영상-작업자의-대조)에 모았다. 기능 검사 14.46초는 카메라 FPS나 영상 한 장의 처리 지연 측정값이 아니다.
