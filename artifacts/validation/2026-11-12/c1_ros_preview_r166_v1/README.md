# C1 원천 배열의 격리 ROS 진단 연결

2026-11-12. 실제 C1을 원래 Standard/기본 회전수로 읽었으며 차량 모터·서보·ESP32 펌웨어·운영 ROS launch는 변경하지 않았다.

## 결과

- 실제 SDK native 배열204회전·102,780점 → `/hardware_preview/c1/native` → 공통 `decode_measurements` 수신204개, 누락0. 원시 배열부터 수신 전 payload까지 각도/순서/거리/추정 시각을 별도 대조했다(`roundtrip_audit.json`). 정렬·720칸 보완·복제 없음.
- 기동 직후 부분 회전까지 포함한 점 수364/508/544(min/median/max), 수신률10.113 Hz, 최대수신간격107.835 ms. 안정화 구간을 제외한 회전수 정확도 측정은 아니다.
- SDK 추정 첫 시각+명목200μs를 같은 호스트 단조→ROS 시각 차이로 옮겼다. 200회전에서 다음 배치와 명목 시각이 겹쳤고 최대4.358 ms였다. 131회전의 명목 끝 시각은 수신보다 늦었고 최대2.983 ms였다. 이 모순을 지우거나196μs로 맞추지 않았다.
- 모든 payload는 `unverified_estimate`, 진단은 `time_contract_verified=false`다. 운영 `scene_motion`의 취득시각 승인은 바꾸지 않았다. 이 경로는 실제 보정 주행 준비의 수신 연결이지, 실차 보정·주행 허가가 아니다.
- 격리domain216/localhost에서만 실행한다. 기본/운영 토픽을 발행하지 않으며 제어 명령도 없다. 구동부를 실행하지 않았다.
- SDK 정상 종료0·sensor stop/motor zero 반환0·잔여 장치 소유자0·임시ACL 원복·158개 소스 불변을 확인했다.

## 초기 실패와 보완

첫 실행은 초기 첫 스캔 대기2초의 SDK timeout(0x80008002)으로0개 수신 뒤 종료했다. stop/ACL 원복은 이때도 확인했다. 이전 진단 probe의 기동 정책대로 **첫 데이터 이전 timeout만 최대15초 이내** 기다리도록 수정했다. 이후 회전에서의 timeout이나 다른 SDK 오류는 즉시 실패로 유지한다. 원 `initial_*` 결과와 `source_manifest_initial.json`을 보존하고 별도원격폴더에서 후속1회를 실행했다.

## 구현·재현

- `scripts/stream_c1_native_preview.cpp`: 기존 제조사 SDK의16개 object에 링크, Standard raw 각도/거리/quality/flag·호스트begin/end·SDK추정첫시각을 JSONL로 전달. 별도RPM명령/SDK변경 없음.
- `c1_native_preview.py`: 공통 native schema 변환, 품질0/거리0은 unknown으로 보존, 실제 시계/순서 재시작에 epoch 변경, 겹침/수신후명목시각을 진단한다.
- `scripts/bridge_c1_native_preview.py`: 구동 없는 격리 ROS 송수신·공통 decoder·종료 감사. 원 raw와 변환 기록은 Git 제외 `build/r166b_recovered`에 보관한다.
- Python3.12/NumPy1.26.4 관련20검사 통과. JetsonHumble/Python3.10에서 실제 송수신까지 확인했다. 호스트 파일은 `source_manifest.json`, SDK object/실행파일은 `compile_manifest_final.json`에 고정했다.
- 최초 bundle SHA `d18a54aa7d06046d922d137133ed803a4f71ffb20b475fb64785298a860a247c`, 최종 `b6405dec509a55446c0b02ef864af2baa1f3f0704a7f32a88edeb0aff1f5ede7`. 원격 `/home/q/it_arena_c1_preview_r166b_20261112`.

남은 일은 시간 추정 오차를 허용할 운영 모드의 명시 설계와 실제 움직임/최종 장착 검증이다. 정지 수신으로 그 오차를 교정했다고 하지 않는다. 아직 r161 IMU와 이번 C1을 하나의 구동 launch로 합치지 않았다.
