# MCU 혼합 송신·호스트 복원 검증 r293

출력 권한이 없는 r289의 `RuntimeTx`를 PC에서 검사했다. 펌웨어 구현은 수정하지 않았고 보드에 설치하지 않았다. 구동계/엔코더 환산·보호 조건·GPIO는 그대로다.

- 분할된 IMU·관측 프레임 도중에 IA 프레임이 끼지 않는 것, 관측 덮어쓰기 거절,64개IMU 큐의65번째 입력 거절/계수와 순서,송신 sink의 잘못된 길이 반환 이후 고정 오류를 확인했다.
- 실제 `ActuationMotorEndpoint`를 사용한5초 생성+0.5초 배출의 합성 시계에서 IMU400프레임/s·관측50프레임/s·주기 offer20ms/진단1초를 결합했다. 921600baud 8N1에 해당하는 매1ms92/93바이트 예산이며 실제 FIFO/스케줄링/USB 지연 모델은 아니다.
- 정상 조건은 IMU2,000·관측250·offer275·상태1·진단6개가 복원됐고 누락0이다.25ms 송신 불가 조건은 IMU2,000개 보존·관측248개 복원/2개폐기계수 일치다. 프레임 중간에 다른 magic이 들어 있는 합성 payload도 그대로 보존했다.
- 기존 Python `SensorLinkDecoder`와 IA/IMU decoder로1/37/4096바이트 수신 단위를 바꿔 같은 결과·CRC/비프레임오류0을 확인했다. 제어1,100호출은 모두effort0·미ARM이고 원PCNT 관측은 `encoder_motion_verified=false`다.
- 최적화 C++ 빌드와 ASan/UBSan 실행을 통과했다. 첫 빌드의 시험 코드 한 줄for문이 `-Werror=misleading-indentation`에 걸려 명령을 두 줄로 분리했다. 펌웨어 경계를 완화한 수정은 없다.

[기계 판정](assessment.json)·[sanitizer 로그](sanitizer.log)·[사용 소스](source_hashes.json). 원 wire·빌드 파일은 Git 제외하며 PC의 생성/복원 결과를 MCU 실시간 구동/제동 검증으로 부르지 않는다. 고정5.5초 합성 시계는 실제 실행시간 측정이 아니다.

다음 실제 장치 재허용 뒤에는 IMU·PCNT·서보 관측 동시 수신/주기와 오류 처리를 확인해야 한다. 엔코더 부호/계수,실장 조향 환산·출력 감시가 갖춰지기 전까지 `output_ready=false`를 유지한다. 현재 제한 주행 결과와 r282지연 교통 실패도 별개다.
