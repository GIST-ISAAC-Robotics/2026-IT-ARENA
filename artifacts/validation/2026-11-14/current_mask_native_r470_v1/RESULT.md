# r470 실제 반환 패딩 검사의 좁은 native 최적화

**후속 공통 이관 완료:** 사용자 재개 뒤 구현5파일+신규 회귀1파일을 before hash 대조로 공통 반영했고, WSL native 재빌드 및 관련36회귀(신규10/구조9/numeric17)를 통과했다. 구형 binary 실패와 새 entrypoint import를 보존하고 WSL 자원을 반환했다. Jetson 재측정·새 최적화는 하지 않았다. [통합 결과](integration_v1/RESULT.md).

**r467의 판단을 유지하면서 실제 반환과의 점유·패딩 검사 한 계산만 C++로 옮겼다.** 같은 ARM의 SOLO current 중앙값은 **125.23→102.40 ms(−18.2%)**, **75.56→64.01 ms(−15.3%)**다. 전체 출력과 이전 r467 ARM 출력은 exact였다. 다만 첫 대표 최대102.75 ms와 기존 heavy GUARD 최대119.72 ms는 여전히100 ms를 넘는다. 판정선을 완화하거나 반올림해 마감 통과로 처리하지 않았다.

아래 비용 시험 당시에는 독립 stage와 [검토용 패치](candidate.patch)만 준비했고 공통 소스는 변경하지 않았다. 이후 이관 상태는 위 후속 결과를 따른다. 실제 센서·ROS·구동·전체 제어 마감 검증이 아니다.

## 변경 범위

기준은 r467의 batch 최적화 버전 `known_structure.py` SHA `ad62d55cdb67cf2a7fca1b0e24d8a2283bfc32de2e1cf23f38817d6b3d2d2dad`다. 원 r463/r467 자료는 그대로 보존했다.

새 `current_mask_kernel.cpp`는 기존 **`any(norm(query - returned_hit) <= padding[hit])`**만 계산한다. 모든 질의와 모든 실제 반환의 차이를 같은 순서의 multiply/add/sqrt로 계산하며, inclusive `<=`를 유지한다. 임계값·허용오차·질의/광선 수·가시점·거리 패딩을 바꾸지 않았다. 조기 true 반환은 원 전체 any의 동일한 `current_known_occupied_padding` 거절이며 어느 점이 먼저 걸렸는지에 따른 추가 진단은 없다.

bracket/취득 원점·방향·첫 거절 우선순위·성분 폭·최대값 동률 코드는 r467 그대로다. helper 전체를 native로 재작성하지 않았다. Python 선택은 원 NumPy 경로를 유지하고, 명시 native 선택에서 새 함수가 없는 오래된 binary는 오류로 거절한다.

구현 차이는5파일이다: 새 C++ kernel, 기존 Python C API 연결부, `numeric_kernels.py`, `known_structure.py`, `setup.py`. 컴파일 옵션은 기존 **`-O3 -std=c++17 -ffp-contract=off -fno-fast-math`**를 유지했다. 점유 bool 반환용1바이트 외에 C++ heap이나 질의×반환 임시 행렬을 만들지 않는다. [파일별 SHA](changes.json), [생성 절차](prepare_stage.py).

**공통 채택 시 native 확장 재빌드가 필요하다.** 예전 binary를 그대로 두고 새 Python 연결부만 배포하면 명시 native 경로에서 오류가 난다. ABI_VERSION은 기존1을 유지하는 추가 함수이며, 함수 존재 검사로 오래된 binary를 검출한다.

## 한 번의 동일 ARM 비용 비교

신규 `/home/q/it-arena-r470-a`에서 기준은 r462의 검증된 binary를 재사용했고, 후보는 stage 안에서 기존 설치 도구만으로 `setup.py build_ext --inplace`를1회 수행했다. 패키지 설치·시스템 경로 설치는 없었다. 빌드/검증을 끝낸 뒤 각 버전4사례×3회와 warmup1회씩을 순차 측정했다. **전체 Evaluator 측정24회+warmup2회, 추가 전체 평가0회**다.

입력은 r467의 실제 저장 r457 seq1360/1590/1986과 r452 seq519다. `current_request(..., retained=..., crop_current_selection=True)`를 그대로 사용했으며 입력 SHA는 `991a034a64a4b9b00fe7f98dbde8cb0b50d99f9c07187da3c5aedf89d01ed9cb`다.

표는 **중앙값 / p95 / 최대(ms), 각 n=3**다. p95는 작은 저장 표본의 선형 보간이며 장기 마감 보장이 아니다. 전체 `Evaluator`의 restore/scene/current/freeze를 포함하고 import·생성자·request 생성·hash·결과 저장·SSH·IPC는 제외했다. round0은 원함수, round1/2는 반환값을 그대로 전달하는 계측 wrapper다. wrapper 비용은 전체 wall에 포함했다.

| 실제 입력 | r467 batch 기준 | native 점유 후보 | 출력 |
|---|---:|---:|---|
| r457:1360 GUARD |50.58 / 50.69 / 50.70|50.13 / 50.38 / 50.40|.15, 전체 exact|
| r457:1590 SOLO |125.23 / 125.65 / 125.70|102.40 / 102.72 / 102.75|.75, 전체 exact|
| r457:1986 SOLO |75.56 / 75.57 / 75.58|64.01 / 64.30 / 64.33|.75, 전체 exact|
| r452:519 GUARD |120.35 / 120.67 / 120.71|119.57 / 119.71 / 119.72|.35, 전체 exact|

원 r467 실행에서 같은 batch 코드가132.26/79.61 ms였던 값과 이번125.23/75.56 ms를 직접 최적화 효과로 빼지 않는다. governor/주파수를 고정하지 않은 별도 실행이므로 **이번 행렬 안의 같은 조건 기준/후보**로 비교했다. 경쟁 부하는 추가하지 않았다.

아래는 계측2회의 호출당 중앙값(ms)이다. native 점유 시간은 `current_mask_shadow` 안에 포함된 값이며 둘을 더하면 중복 계상이다.

| 항목 | r457:1590 | r457:1986 |
|---|---:|---:|
| 원 `current_mask_shadow` |79.82|42.63|
| 후보 `current_mask_shadow` |57.78|31.61|
| 후보 내부 점유 kernel 연결부 포함 |2.49|1.28|

같은 행렬의 전체 Python process peak RSS는 기준44,332/후보41,696 KiB다. 단일 process 표본이며 helper 메모리 절감의 독립 측정으로 확대하지 않는다. 입력 변환이 필요하면 연속 float64 사본은 만들 수 있지만, native 점유 계산 자체는 쌍별 dense 배열을 할당하지 않는다.

## exact·경계·빌드 검증

[검증 스크립트](verify_native.py)는 새 binary에서 다음을 수행했다.

- **수치 kernel23건 exact:** 빈 질의/반환, 경계와 바로 위·아래 `nextafter`, 대각 거리, signed zero, subnormal 제곱 underflow, overflow, NaN/Inf, 고정 seed의 부재/마지막 쌍 반환. 입력 배열 불변도 확인했다.
- **raw buffer6건 거절:** 잘못된 dtype/길이, 읽기 전용 출력, 비연속 입력, 최대 크기 초과, 음수 크기. C++ buffer 읽기 전 기존 연결부 규칙으로 거절한다.
- **helper13쌍 전체 exact:** 실제 저장4입력과 기존 자기 가림 반례9개. 기존 assertion body를 그대로 직접 실행했으며 pytest 실행으로 표시하지 않는다. helper 검증은 전체 Evaluator 횟수에 섞지 않았다.
- Python 선택 시 fallback 유지, 명시 native 선택에서 새 entrypoint 누락 거절을 확인했다.
- full current24회와 warmup2회 모두 계측 전후 및 이전 r467의 같은 ARM 결과와 전체 exact였다. 명령·경로·시각·진단·독립 보호 필수와 runtime 비허가 플래그가 유지됐다. x86 수치 차이를 새로운 허용오차로 통과시키지 않았다.

기존 depth/polygon 수치 소스와 컴파일 옵션은 변경하지 않았다. 새 ARM binary SHA는 `5654cd2f52af838b52384a084f159bb1319925db6690945b4403cb7ae33f5bff`다. 기준 binary SHA는 기존 `a2afb70525be1e86f63ae072abc4fbaeec10d65f0596e5f642f135ba6a1de394`다. 정확한 원 출력·case별 시간·검증 항목은 [전체 요약](summary.json)에 있다.

## 회수·남은 범위

로컬/원격 manifest464파일 변경0, 빌드/검증/측정 PID61721·61738·61744·61750 모두 exit0, 강제 종료0, 잔존 group0이다. 관련 실행 및 열린 센서 포트0을 확인하고 [Jetson 자원 반환](resource_return.json)을 완료했다. 센서·ESP32·ROS·ARM/flash·모터·서보·전력 설정·WSL은 변경하지 않았다.

전송 전 mDNS 발견이 두 차례 실패했다. 첫 실패 후 추출/실행 명령도 대상 파일이 없어 종료했으며 빌드나 계산은 시작되지 않았다. [실패 기록](preparation_attempt1_failure.json)을 보존하고 기존 SSH 호스트 키와 일치하는 장비 주소를 확인한 뒤 같은 묶음을 전송했다. 영구 IP 설정이나 키 변경은 하지 않았으며 실제 빌드/행렬은 한 번만 실행했다.

추가 native bracket 구현이나 전체 helper 재작성은 이번 승인 행렬을 넘어 진행하지 않았다. **100 ms 초과는 해결 완료가 아니며**, 이번 한 수치 kernel의 실익과 exact 근거를 주 담당이 검토해 공통 채택 여부를 결정한다. 원 manifest/ZIP/log/source는 Git 제외 경로에 보존했고, [manifest](source_manifest.json)와 [패치](candidate.patch)를 통합 진입점으로 제공한다.

## 후속 회귀 실행과 일시 중단 인계

재사용 가능한 신규 pytest10개를 준비한 뒤, 별도 승인으로 기존 `/home/q/it-arena-r470-a/stage`와 같은 binary를 이용해 한 번 실행했다. **10 passed, 실패0/skip0**이며 source464파일·native hash가 유지됐다. pytest PID62051은 exit0·강제0·잔존0이고 Jetson 자원을 반환했다. 새 빌드/성능 행렬/개선/공통 통합은 하지 않았다. 사용자 일시 중단 지시에 따라 **미통합 stage·패치와 회귀 근거를 보존하고 종료**한다. [실행 결과](pytest_run_v1/RESULT.md), [통합 준비 명령](REGRESSION_HANDOFF.md).
