# G1 비용·입력 감사 최소 묶음

2026-10-13, 로컬 패키지 준비. Jetson 접속·전송·실물 구동은 하지 않았다.

- `run.sh`: 관련 회귀 **34개 통과**, 종료 코드 0.
- `build/jetson_g1_profile_20261013_v1.tar.gz`: **22파일 / 6,680,245바이트**, SHA-256 `e3a01b09791a42f1ba91a27baed248dfd1b0d39999d6ed7b219f322dc05a972d`.
- [manifest](manifest.json), [내용 검증](verification.json): archive와 모든 멤버의 해시/크기 일치, 누락·비정상 멤버 없음. 이 단계는 추출하지 않았다.
- 원시 판단 두 파일은 기존 Git 제외 정책을 유지하며 묶음은 `build/`에 보관한다. 공개 저장소의 manifest/검증 기록과 로컬 원시 자료를 구분한다.

`run_smoke.sh`는 검증을 다시 확인한 뒤 별도의 새 `build/follow-profile-smoke.JCenu7` 디렉터리에 **로컬에서만** 추출해 실행 진입점을 확인했다. ROS overlay/기존 Python 경로를 해제하고 세 import가 모두 새 추출본 내부임을 확인했다. 종료 코드 0, [격리 판단 재생](isolated_profile_report.json) **374/374 일치**, 출력 불일치 0·입력/소스 불변이다. [합성 입력 감사](isolated_capture_report.json)는 엔코더/IMU 각 5개·기록 무결성 참·구동 비허가다. 이번 실행은 성능 측정의 재검증이 아니라 의존성/진입점 검사다. PC 환경이므로 Python 3.10/ARM/Jetson 실측을 뜻하지 않는다.

묶음의 내용은 생성 당시 고정본이다. 후속 C2 감사기 변경을 자동 포함하지 않으며, 추후 이관 시에는 필요한 변경만 새 버전 묶음으로 별도 검증한다.

[측정 계약·한계](../../../../docs/simulation/G1_DECISION_PROFILE_2026_10_13.md).
