# 실제 C1 시각의 SDK 근거와 저장 대조

## 확인한 소스 경계

11/10에 확인했던 공식 SDK 커밋 `99478e5fb90de3b4a6db0080acacd373f8b36869`의 네 파일을 직접 읽었다. URL/해시는 `sources.json`, 로컬 열람본은 Git 제외 `build/c1_timestamp_reference_r213/`에 보존한다. 현재 장치의 펌웨어나 SDK 바이너리를 바꾸지 않았다.

- Standard의 일반 노드 처리기는 점을 디코딩한 **호스트 `getus()` 시각에서 명목 필터 지연·샘플 중앙·직렬 전송 추정치를 뺀 값**을 넘긴다. 센서의 점별 하드웨어 시각을 읽는 구조가 아니다. [handler_normalnode.cpp](https://github.com/Slamtec/rplidar_sdk/blob/99478e5fb90de3b4a6db0080acacd373f8b36869/sdk/src/dataunpacker/unpacker/handler_normalnode.cpp), [dataunpacker.cpp](https://github.com/Slamtec/rplidar_sdk/blob/99478e5fb90de3b4a6db0080acacd373f8b36869/sdk/src/dataunpacker/dataunpacker.cpp).
- scan holder는 회전 시작 sync 점의 추정 시각을 배치 첫 시각으로 보관한다. `grabScanDataHqWithTimeStamp`는 그 앵커와 정렬 전 배열을 돌려주며, 각 점의 SDK 추정 시각 배열을 함께 주지 않는다. timing descriptor도 `native_timestamp_support=false`다. [sl_lidar_driver.cpp](https://github.com/Slamtec/rplidar_sdk/blob/99478e5fb90de3b4a6db0080acacd373f8b36869/sdk/src/sl_lidar_driver.cpp).

따라서 현재 preview가 그 첫 앵커 뒤에200μs씩 붙인 시각은 이중으로 추정된 값이다. 호스트/전송 지연과 명목 주기의 차이가 배치 겹침에 기여할 수 있다는 코드 근거를 확보했지만, 각 원인의 기여량이나 실제 취득 오차를 실측한 것은 아니다. SDK를 수정해 모든 호스트 디코딩 시각을 노출해도 USB 묶음 수신·스케줄링 지터가 없어지는 것은 아니다.

## 같은 r204 원시203회전의 오프라인 비교

| 정책 | 적용 배치 | 추정 점 간격 중앙 | 이전 끝과 현재 첫 시각 겹침 | `grab` 완료보다 늦는 끝 시각 |
|---|---:|---:|---:|---:|
| 현재 명목200μs | 203 | 200μs | 201회·최대4.088ms | 183회·최대3.649ms |
| 이미 확보한 이전 최대8개 배치 간격/점 수의 비율 | 192 | 196.061μs | 67회·최대2.085ms | 52회·최대1.643ms |

이 표는 SDK 함수 완료 시각을 기준으로 한다. r204 보고서의 Python 수신 시각 이후130회·최대2.686ms와 기준 시점이 다르므로 서로 모순되지 않는다. 후보는 현재 배치의 다음 시각을 기다리지 않으며, startup/비정상 배치 뒤에는 이전 근거가 없어 계산하지 않는다. 원 거리/각도/순서나 실물 데이터는 변경하지 않았다.

**운영 코드에는 채택하지 않았다.** 모순 감소만으로196μs가 정확한 센서 교정값이라고 할 수 없다. 현재의 `unverified_estimate`, 미장착 frame, 구동 미연결 상태를 유지한다. 후속 운영 추정 정책은 수신 완료 앵커/회전 위상/호스트 지연을 명시해 구동 없는 실행에서 먼저 비교하고, 조립 후 정적 벽을 보며 회전시키는 시험으로 왜곡을 확인해야 한다. 현재 센서를 움직이지 못하므로 그 검증을 완료로 표시하지 않는다. 실제 통신과 원시 보존은 r204에서 끝났으므로 같은 정지 수신을 반복하지 않는다.
