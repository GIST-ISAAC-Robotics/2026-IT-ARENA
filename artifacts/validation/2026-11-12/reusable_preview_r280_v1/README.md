# 재사용 실제 센서 관측 실행 r280

작업 폴더별 복사본을 공통 실행 명령으로 옮기기 위한 후보를 만들었다. 기존 `probe_g1_hardware_shadow`의 기본 카메라 미연결을 유지하고, 명시 `--camera`에서만 실제 RGB/별도 영상 프로세스를 연결한다. C1은 tracking, IMU는 FIFO tick 추정이며 시간 검증false를 유지한다. `probe`/`sensor-link`는 선택한 하나만 UART를 소유하고 자동 flash/ARM/DRIVE/가상 엔코더는 없다. 센서 취득 정확도·실제 구동·장착 후 인식 검증이 아니다.

## 검증 결과

- 명령 선택·기본 유지·장치 없는 `--plan-only`·종료 경계7검사 통과.
- 실제 Jetson20초 캡처: C1 준비 후183배열/93,240점·공통 gyro3,746개·거부0. C1 추적 reset0, 실제 시각 교정 완료는 아니다.
- 카메라1,201장 발행/1,190장 수신·11장 미수신, 중복/형식/시각 대응 오류0. 영상196회 완료·채택, 처리p95 46.07ms·수신 header→채택p95 80.25ms. 영상 픽셀은 저장하지 않았다.
- private1,301명령 전부0, 엔코더/모터축 피드백0. 계획 부하가 없으므로 전체 실시간 주행 증명이 아니다.
- 네 자식과 계획/영상 자식 정상 종료·IMU inactive/C1 stop/카메라NULL·포트 해제. 펌웨어/전력 변경 없음. 이 실행에서는 `probe` 원 IMU 앱을 사용했다. sensor-link 선택의 새 실물 캡처는 하지 않았고 기존r254 근거를 유지한다.

실행 묶음188파일 SHA256 `fc0e310e710f6ab4c8b3078d82497282e65bf9b1751bcf87e46e6fed41d9bb6b`, 전체 소스 불변이다. Git 제외 원본 `build/r280_recovered/run`, 회수 ZIP SHA256 `781e2e62343bada7455faa6163e2edd39087260d99c0d213b70e5c130e78d951`이며 [audit.json](audit.json)에 독립 수신 대조를 남겼다.

r279 종료·소스 불변 확인 후 공통 `scripts/probe_g1_hardware_shadow.py`, `scripts/bridge_b0390_preview.py`, `scripts/run_hardware_sensor_preview.py`로 옮겼다. 카메라는 바이트 동일, 노드는 저장소 기준 경로만, 감독기는 경로/스크립트 이름만 바뀌었음을 검사했다. 공통7검사·계보2검사·CLI3개를 통과했다. 새 수신은 반복하지 않았다. [common_move.json](common_move.json).

사용 방법은 [실물 이관 절차](../../../../docs/hardware/PHYSICAL_HANDOVER_APPROVED_LAYOUT.md#조립-전-관측-실행)를 따른다. 이 도구는 관측/실행 연결이며 운영 구동 펌웨어나 실제 주행 준비 완료를 대신하지 않는다.
