# C1 ROS 출력의 취득 순서/시각 감사

`run_audit.py` 1회 exit 0, C++17 compile/native exit 0, native assertions 통과. 고정 SHA의 선택 함수만 타입 stub에서 실행했으며 ROS·SDK 통신·장치 포트·회전 모터·Jetson은 실행하지 않았다.

- 원문 SHA/URL/해시: `source_manifest.json`. 원문은 local ignored `upstream/`에 LICENSE와 함께 보존한다.
- 추출 함수와 검사기: `probe.cpp`. `run_audit.py`에 추출 경계와 stub을 명시했다. [원문 저작권·라이선스](THIRD_PARTY_NOTICE.md)를 함께 보존한다. 전체 ROS/SDK 호환 빌드가 아니다.
- 결과: `report.json`, 컴파일/실행 stdout·stderr·exit 파일. 입력 500점→720칸, 원본 500개·추가 중복 220개. 합성 대기 인수로 같은 4점의 시간 간격이 달라지는 것을 확인했다.
- 원문/운영 코드 불변, hardware/timing/ready_to_drive 모두 false. 데이터시트 수치·실제 수신률·실차 오차 측정이 아니다.
- [해석·이관 조건](../../../../docs/sensors/C1_DRIVER_TIME_CONTRACT_2026_10_23.md)
