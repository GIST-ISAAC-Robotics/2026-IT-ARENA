# 최신 G1 판단 비용 시험의 로컬 이관 묶음

상태: **77파일·19,591,201바이트 v2 묶음의 해시, 격리 import, 2,176판단 일치**를 확인했다. Jetson에 전송하거나 실행하지 않았다.

오늘 추가된 bypass_yield_progress/front_patch_motion/released_rear_sweep/turn_preview 네 파일이 기존 계측 스냅숏과 최소 이관 목록에서 빠져 있었다. 두 목록에 포함했고 기존 전이 import 회귀를 사용했다. 관련 **69개 검사 통과**. [수정 전](before.zip), [후속 소스 계약](../../../../config/tests/follow_bundle_refresh_r1_sha256.json).

첫 v1 묶음에는 단독 G1과 G1/G2 교통 입력을 함께 넣었다. 해시·파일은 맞았지만 기존 비용 도구와 묶음 검증기가 G1 schema 1만 지원해 교통 입력의 schema 5를 거절했다. 이는 준비 단계의 범위 오류다. [첫 검증 실패](verify.json)와 build의 v1 archive를 보존하고, 지원 범위를 늘리는 대신 **단독 입력만 넣은 v2**를 새로 만들었다. v1은 사용하지 않는다.

- 사용 묶음: 로컬 Git 제외 build/jetson_follow_20261101_v2.tar.gz
- SHA-256: f3f0b7aa1cf740ad50949af47d5c0daa8320723f651594193f478baa14b4b194
- [파일 목록·해시](bundle_manifest_v2.json), [내용 검증](verify_v2.json)
- [격리 재생 결과](isolated_result.json): import가 모두 새 추출 폴더의 모듈을 사용했고 모든 멤버 파일은 불변이었다. 단독 완주 입력 2,176회 모두 일치했다.

[격리 실행기](isolated_replay.py)는 검증한 로컬 묶음을 새 build 폴더에만 풀고 Python -I로 원래 저장소/PYTHONPATH와 사용자 패키지 경로를 배제한다. 이 실행기는 WSL Python 3.12로 검증한 보조이며 Jetson에 설치할 프로그램이 아니다. 원시 판단 입력과 archive는 공개 Git에 추가하지 않는다. 원격 장치·ROS/영상·독립 보호·MCU/모터 동시 실행, ARM 패키지 호환성과 실시간 성능은 별도다.

이후 제어 소스가 바뀌면 이 묶음이 자동 갱신되는 것이 아니다. manifest가 가리키는 현재 시점의 별도 보존본이다. 제어기 최적화를 반복하기 위한 새 관문으로 삼지 않고 대표 교통 개발로 돌아간다.

이후 착수한 [후반부 1 mm 공백 예산 후보](../released_fringe_budget_v1/README.md)는 이 묶음에 포함하지 않는다. v2는 그 변경 전의 판단 비용/재생 비교용 보존본이며 최신 작업 트리 전체의 이관본이 아니다.
