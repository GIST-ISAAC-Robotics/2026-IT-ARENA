# B0183 입력 검사 v1 — 수집 단계 실패

`test_basic_autonomy.py`가 요구하는 `xacro`를 찾지 못해 pytest 수집 단계에서 exit 2로 중단했다. 실행기 exit는 1이며 시험 본문은 실행되지 않았다. [stdout](stdout.txt)·[보고서](report.json)·[당시 소스 해시](hashes_before.json)를 보존한다. 카메라나 운영 코드 결함의 재현이 아니라 ROS Python 환경 누락이다.

후속 [v2](../b0183_input_contract_v2/README.md)는 다른 누락도 보존했으며, 검증된 기존 환경 진입점을 사용한 [v3](../b0183_input_contract_v3/README.md)에서 마무리했다. 설치/운영 코드 변경 없음.
