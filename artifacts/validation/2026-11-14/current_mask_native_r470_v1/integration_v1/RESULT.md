# r470 공통 이관과 WSL native 회귀

**사용자 재개 후 검증된 r470 구현5파일과 신규 pytest1파일을 공통에 이관하고, WSL native 재빌드와 관련36회귀를 통과했다.** 새 최적화·성능 행렬·Jetson 작업은 수행하지 않았다.

이관 전6개 대상의 before hash를 원 `changes.json` 및 회귀 hash와 대조했다. 기존4개 소스와 구형 x86 binary는 Git 제외 `before/`에 보존한 뒤 stage 원문을 바이트 그대로 복사했다. 새 C++ 파일과 신규 pytest는 원 경로가 없음을 확인했다. `body_gap`·`pass_release`·`current_region` 및 루트 문서는 변경하지 않았다. [소스 이관 receipt](source_receipt.json).

WSL `Ubuntu-24.04`의 관련 실행0을 확인한 뒤 기존 명령 `python3 -B setup.py build_ext --inplace`를 `src/arena_autonomy`에서 실행했다. 기존 NumPy/컴파일 도구만 사용했으며 패키지를 설치하지 않았다. **구형 binary의 실제 entrypoint 누락 오류를 먼저 보존**하고 재빌드 후 공통 source 경로의 새 함수 import와 호출을 확인했다.

| 구분 | 확인 결과 |
|---|---|
| 구형 x86 binary |새 함수 없음, `RuntimeError: native current mask occupancy kernel missing; rebuild arena_autonomy` 보존|
| 새 x86 binary |`current_mask_occupied` 존재, 실제 호출 결과 true|
| 신규 native 회귀 |10 passed|
| 기존 current structure 회귀 |9 passed|
| 기존 numeric 회귀 |17 passed|
| 합계 |**36 passed, 실패0/skip0**, pytest 표시4.44초|

pytest 표시 시간은 회귀 실행시간이며 current 제어 비용 계측이 아니다. 재빌드 뒤 실제 import는 공통 `src/arena_autonomy/arena_autonomy/`의 Python 연결부와 `_numeric_kernels.cpython-312-x86_64-linux-gnu.so`였다. 별도 `colcon` 설치 갱신이나 주행은 이 작업의 범위가 아니다.

구형 binary SHA는 `99bb9a7def33470a6cabfe5195660cc178f16f60c72d4133d389a10ee285d434`, 새 binary SHA는 `924bb2500d9e87ab85cc8ffdc4aa63e1c82fcdee646ee0dc0776c72e6fc9e975`다. [구형 실패 기록](old_binary_probe.json), [새 함수 import](new_binary_probe.json), [WSL 회귀·실행 receipt](wsl_receipt.json), [최종 통합 receipt](integration_receipt.json).

실행 중 대상6파일 변경0, 단계 PID735/736/752/753 모두 exit0·강제 종료0·잔존 group0이다. 종료 시 관련 process0을 확인하고 **WSL 자원을 주 담당에게 반환**했다. source diff의 whitespace 검사도 통과했다. commit/push는 하지 않았다.

Jetson은 이번 작업에서 사용하지 않았다. 기존 r470 ARM kernel23/버퍼6/helper13쌍, full current24회+warmup2회의 exact와 [실제 ARM pytest10](../pytest_run_v1/RESULT.md)을 그대로 재사용한다. 같은 사례의100 ms 초과와 heavy GUARD 비용은 이전 결과로 유지하며 공통 이관을 실차·ROS 전체 마감이나 주행 개선 증거로 확대하지 않는다. 이후 실행 자원 배정과 루트 현황 갱신은 주 담당에게 인계했다.
