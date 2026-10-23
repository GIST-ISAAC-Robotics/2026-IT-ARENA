# 센서 기록의 시계별 간격·기록 완료 지연

`run_checks.sh` exit 0, 관련 회귀 **161통과·1.15 s**. 저장된 system/정상 simulation CDR 두 묶음의 무결성을 다시 확인하며 진단했다. 원시 입력과 대상 소스 해시는 검사 전후 같고 새 ROS 노드/시뮬레이션·장치를 시작하지 않았다.

| 보존 입력 | 토픽별 표본 | callback 단조 시계의 메시지율 | recorder ROS 메시지율 | callback→writer receipt 최대 |
|---|---:|---:|---:|---:|
| ROS v6 system, IMU / scan | 각각 80 | 92.086 / 92.104 Hz | 92.087 / 92.104 Hz | 1.064 / 1.063 ms |
| ROS v4 sim_valid, IMU / scan | 각각 80 | 91.122 / 91.136 Hz | 각각 100 Hz | 1.340 / 1.614 ms |

이는 과거 합성 발행기의 짧은 기록을 설명하는 수치다. C1/IMU 실제 주파수나 Jetson 처리량이 아니다. 원래 header는 의도적으로 0/1/2/1초를 반복했으므로 토픽마다 0 시각 20개·역전 39회이며 header 메시지율은 `null`이다. recorder ROS와 header를 빼서 센서 지연이라고 출력하지 않았다.

writer receipt는 큐·역직렬화·write 호출·해시/통계 갱신 이후에 찍힌다. 위 수치를 순수 큐 대기나 물리 디스크 flush 시간으로 해석하지 않는다. 정확한 정수 합/제곱합으로 간격 min/max/평균/표준편차를 계산하고 표본 전체를 메모리에 모으지 않는다. 분위수/물리 주파수 승인·자동 교정은 없다.

회귀에는 균일/묶음 전달·header 0/음수/중복/역전·서로 다른 epoch·멈춘/가속된 simulation 시계·토픽 분리·알 수 없는/여러 publisher·늦은 digest 실패와 기존 출력 보존을 포함했다. `report.json`·각 입력의 `system.json`, `simulation.json`·`junit.xml`·소스 snapshot을 보존했다.
