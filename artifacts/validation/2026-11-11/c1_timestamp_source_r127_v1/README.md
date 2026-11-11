# C1 Standard 시각의 SDK 산출 경로

2026-11-11. 장치에 접속하거나 회전수를 바꾸지 않고, 11/10 진단에 사용한 드라이버와 같은 공식 SDK commit `99478e5fb90de3b4a6db0080acacd373f8b36869`의 관련 소스만 확인했다. [출처·SHA·계산](report.json), [감사 코드](audit.py).

## 확인 결과

- Standard 응답 `0x81`의 5바이트 점에는 동기/품질·각도·거리만 있고 센서 취득 timestamp 필드는 없다.
- [normal node 해석기](https://github.com/Slamtec/rplidar_sdk/blob/99478e5fb90de3b4a6db0080acacd373f8b36869/sdk/src/dataunpacker/unpacker/handler_normalnode.cpp)는 점을 해석할 때 호스트 시각을 읽고 일정한 지연 추정치를 뺀다. C1/200μs 조건에서 `200 + 100 + floor(1,000,000 × 5 × 10 / 460800) = 408μs`다. 이 값은 실제 USB·호스트 큐 지연을 측정한 결과가 아니다.
- [공통 해석기](https://github.com/Slamtec/rplidar_sdk/blob/99478e5fb90de3b4a6db0080acacd373f8b36869/sdk/src/dataunpacker/dataunpacker.cpp)의 시각 호출과 [Linux timer](https://github.com/Slamtec/rplidar_sdk/blob/99478e5fb90de3b4a6db0080acacd373f8b36869/sdk/src/arch/linux/timer.cpp)는 호스트 단조 시계 기반이다. 센서 자체 시계와 동기화한 값이 아니다.
- [드라이버](https://github.com/Slamtec/rplidar_sdk/blob/99478e5fb90de3b4a6db0080acacd373f8b36869/sdk/src/sl_lidar_driver.cpp)는 회전 시작 sync 점의 해석 시각을 저장하고 다음 sync에서 완성 회전을 반환한다. `grabScanDataHqWithTimeStamp`가 반환하는 것은 이 시작점의 SDK 추정 시각이다. `native_timestamp_support`도 false로 설정한다.

따라서 **SDK 첫 점 시각 + 점수 × 명목200μs**는 실제 취득시각을 그대로 복원한 공식이 아니다. [r111](../c1_time_consistency_r111_v1/README.md)의 배치 겹침은 이 둘을 함께 사용했을 때의 불일치다. 호스트 해석 시각의 흔들림과 실제 측정 주기의 차이가 각각 얼마나 기여했는지는 현재 기록으로 분리할 수 없다. 약196μs라는 배치 평균을 교정값으로 바로 채택하지 않는다.

## 적용과 다음 단계

현재 native 시뮬은 알려진 취득 시각을 사용하는 모델로 유지한다. 실제 입력 어댑터는 각도/거리/순서와 함께 **호스트 수신·SDK 추정·선택한 취득시각 추정 방식**을 구분해 보존해야 한다. 시뮬의 검증된 시각 표시를 실물에 그대로 넘기지 않는다.

점별 상대 시각 후보를 비교하려면 같은 원시 회전에서 얻은 경계/각도와 호스트 지연을 함께 기록하고, 실제 회전·이동으로 정적 벽이 얼마나 펴지는지 확인해야 한다. 이 물리 시험은 사용자가 센서를 움직일 수 있을 때 수행할 후속 항목이다. 이번 묶음은 불확실성의 출처를 확인한 것이며 새로운 시간 필터·교정계수·운영 드라이버를 도입한 작업이 아니다.
