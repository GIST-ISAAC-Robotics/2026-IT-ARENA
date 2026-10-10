# 호스트 전원 구성표 기록 (G1-b)

- Windows System 로그: 2026-10-07 00:56:41 ArmouryCrate.UserSessionHelper가 전원 구성표를 Turbo(6fecc5ae-f350-48a5-b669-b472cb895ccf)에서 다시 적용, 00:56:45 Silent(64a64f24-65b9-4b56-befd-5ec1eaced9b3)로 변경.
- 01:15경 `powercfg /getactivescheme` = Silent 확인(읽기 전용). nvidia-smi: P4, 34 W/42 W.
- g1b_pilot_v2(00:54:50 시작)는 00:56:49 주행 시작 직후 sim 4.02→4.34 s에 약 28 wall s가 걸려(실시간 비율 약 0.011, v1 약 0.046) 영상 벽시계 감시(30 s)로 SENSOR_STOP이 고정됨. 성능 환경 무효로 분리하며 제어기 실패로 세지 않는다.
- 사용자가 속도 저하를 보고 **직접 Turbo로 전환**했다(사용자 전달). 이후 승인 범위 안에서 01:18:36 `powercfg /setactive 6fecc5ae-...`를 실행했으나 실행 직전 값이 이미 Turbo였으므로 **도구 실행은 같은 값의 재적용(무변화)**이다. 실행 후에도 Turbo. 다른 OS/전원 설정은 변경하지 않음.
