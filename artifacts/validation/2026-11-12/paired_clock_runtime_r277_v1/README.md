# 짝 읽기의 공통 tracking 연결 r277

r276의 세 번 짝 읽기를 공통 `host_clock_pair.py`로 옮기고 `bridge_c1_native_preview.py --scan-clock tracking`에 연결했다. 기본 `sdk_nominal`의 기존 읽기/변환 의미는 유지한다. 모든 읽기 구간·선택·구간 반폭은 `clock_reads`에 기록하며, 원 수신 시계 값과 실제 clock step/epoch/발행 시 미래 검사를 보존한다. 시계 offset 평활이나 센서 취득 시각 검증 승인은 아니다.

관련54검사, 이전/새 출처2검사와 CLI 진입을 확인했다. r276 실제203배열의 세 번 읽기를 새 공통 함수로 재생해 선택·메타데이터와 SDK payload/진단/ready 시각이 전부 같았다. 새 센서 취득이나 주행 시험을 다시 수행한 것은 아니다. 실제 결합 대표/카메라11장 누락·엔코더 미입력 한계는 [r276](../paired_clock_r276_v1/README.md)에 남긴다.

원 브리지 바이트는 `before_sources.zip`, 새 출처는 `config/tests/paired_clock_r277_sha256.json`에 보존한다. 별도 장치 설정/펌웨어·전력·구동은 변경하지 않았다.
