# r467 같은 ARM의 current 비용과 자기 가림 함수 최적화

**r463는 저장된 SOLO 정체의 0명령을 .75 m/s로 복구하지만, 새 자기 가림 계산 때문에 두 대표 current 비용이 증가했다.** 원 결과를 보존한 뒤 `known_structure.py` 한 파일만 별도 stage에서 최적화했다. 같은 Jetson의 전체 current 중앙값은 **182.93→132.26 ms(−27.7%)**, **104.67→79.61 ms(−23.9%)**이며, 최적화 전후 전체 출력은 동일했다. 첫 대표와 기존 무거운 GUARD는 여전히 100 ms를 넘는다. 현재 stage 제안이며 공통 반영·실제 센서·ROS 주기·주행 검증은 아니다.

## 비교 대상과 측정 범위

최초 비교는 r457 `sources.zip`에서 추출한 r463/base와 r463/stage다. 전체 원본 파일을 ZIP과 대조했고 차이는 `known_structure.py`, `race_runtime_policy/current_region.py`, `solo_low.py`, `solo_crawl.py` 네 파일뿐이다. 후속 비교는 이 r463/stage와 helper 최적화 한 파일이다. r464 벽 갱신이나 진행 중인 r465 입력은 변경하지 않았다.

실제 기록에서 얻은 r457 seq1360/1590/1986과 r452 seq519를 썼다. 모두 원 plan output/input, 후속 current 및 retained 값을 그대로 사용해 **`current_request(..., retained=..., crop_current_selection=True)`**를 만들었다. 현재 코드를 임의 current로 우회하거나 전체 plan을 재생하지 않았다. 입력은 저장 시뮬레이션이며 실제 취득 시간을 위장하지 않았다.

각 비교는 버전별 4사례×3회, 총24회와 첫 r457:1360 warmup 각1회다. 최초 `/home/q/it-arena-r467-a`, 후속 `/home/q/it-arena-r467-b`에서 순차 실행했다. 합계는 **측정48회+warmup4회**, 초과 반복0이다. 새 native 빌드 없이 r462 binary를 복사했으며 C++3개와 setup.py의 SHA 일치를 먼저 확인했다.

시간은 `Evaluator('current', header)(request)`의 wall time이다. restore/scene/current 검사/freeze를 포함하고, import·생성자·request 구성·hash·결과 저장·SSH·IPC를 제외했다. round0은 원함수, round1/2는 원함수를 한 번 호출해 반환값 그대로 전달하는 wrapper다. 매 사례의 round0/1/2 전체 출력과 warmup 출력의 동일성을 검사했다. 후속24회와 warmup은 이전 r467-a의 동일 ARM 출력과도 exact다. wrapper 기록 비용은 전체 wall에 포함하며 빼지 않았다.

각 셀은 **중앙값 / p95 / 최대(ms), n=3**이다. p95는 이 세 표본의 선형 보간값으로 장기 분포나 마감 보장이 아니다. 첫 버전을 모두 측정한 뒤 다음 버전을 측정하는 고정 순서이며, governor/주파수/전력을 바꾸거나 경쟁 부하를 추가하지 않았다.

## 최초 결과: r463의 실제 비용 증가

| 실제 입력 | r463 이전 | r463 통합 | 명령 변화 |
|---|---:|---:|---|
| r457:1360 GUARD |50.44 / 50.72 / 50.75|50.33 / 50.61 / 50.64|.15 유지, 전체 exact|
| r457:1590 SOLO |70.90 / 71.19 / 71.22|182.45 / 183.07 / 183.14|0→.75|
| r457:1986 SOLO |51.44 / 52.15 / 52.23|104.86 / 105.37 / 105.43|0→.75|
| r452:519 GUARD |118.41 / 119.53 / 119.65|118.74 / 119.94 / 120.07|.35 유지, 전체 exact|

두 SOLO의 영역 호출은3→1회다. 그러나 새 `current_mask_shadow`는 각각 약137.83/71.68 ms를 사용했다. 단순 호출 횟수 감소를 전체 비용 개선으로 해석할 수 없다. 경로·조향·기준시각·독립 보호 필수 및 runtime 비허가 플래그는 네 사례 모두 유지됐고, SOLO의 0→양수는 의도한 판단 변경이다.

ARM의 새 helper 질의 수는1284/641개다. 저장된 질의는 모두 서로 다른 좌표여서 단순 중복 제거로 해결되지 않는다. helper는 모든 광선의 취득 원점·방향으로 bracket을 계산하고 모든 실제 반환에 대한 패딩 검사도 수행한다. Windows 한 사례 cProfile에서는 전체51 ms 중 bracket 누적23 ms, norm 누적11 ms, 함수 자체 Python 실행15 ms가 관측됐다. 이는 [국소 Windows 프로파일](local_helper_profile.txt)이며 ARM 세부 시간으로 환산하지 않는다.

## 한 파일 최적화와 후속 ARM 결과

[검토용 패치](optimization_v1/candidate.patch)는 원 helper 안의 계산만 바꾼다.

- 최대32질의씩 전체 광선의 radius/angle/bracket을 계산한다. 원점 이동을 유지하며 searchsorted·광선 생략·질의 샘플링은 하지 않는다.
- 2차원 norm과 dot의 원 연산 순서를 명시해 임시 3차원 배열과 반복 reduction을 줄인다. hypot/제곱 임계값/허용오차 변경은 없다. `np.sum`의 signed-zero도 대조했다.
- bracket·반환 상태·양옆 깊이·성분 폭 판정을 batch로 계산한다. 첫 실패 질의와 그 질의의 원 거절 우선순위를 유지한다.
- 인접 masked 광선은 같은 연속 성분에 속한다는 기존 조건을 사용한다. 성분 최대폭의 첫 동률, 그룹 삽입 순서와 batch 간 strict `>` 갱신도 보존한다.
- 실제 반환의 패딩 충돌 검사는 원 코드 그대로다. 일반 미관측·profile·시각 계약·숨어 있는 물체 위험 및 독립 보호 경계는 바꾸지 않았다.

| 실제 입력 | r463 원 helper | 최적화 helper | 중앙값 변화 |
|---|---:|---:|---:|
| r457:1360 GUARD |50.25 / 51.15 / 51.25|50.25 / 50.41 / 50.43|약0%|
| r457:1590 SOLO |182.93 / 183.94 / 184.05|132.26 / 132.36 / 132.37|−27.7%|
| r457:1986 SOLO |104.67 / 105.37 / 105.44|79.61 / 79.75 / 79.76|−23.9%|
| r452:519 GUARD |120.66 / 121.18 / 121.23|120.59 / 120.71 / 120.73|약0%|

후속 비교는 네 사례 모두 명령·경로·시각·진단까지 **전체 exact**다. 초기 비교의 0명령으로 되돌려 시간을 줄인 것이 아니다.

아래는 계측 round1/2의 **호출당 합계 중앙값(ms), n=2**다. `motion_region` 호출 수는 SOLO 각각1회, 두 GUARD 각각2회로 유지됐다.

| 항목 | r457:1590 원→최적화 | r457:1986 원→최적화 |
|---|---:|---:|
| `motion_region` |27.37→27.39|16.09→15.67|
| `profile_evidence` |3.25→3.28|3.28→3.32|
| `current_mask_shadow` |137.70→87.18|71.54→46.63|

최적화 후에도 r457:1590 최대132.37 ms와 기존 heavy GUARD 최대120.73 ms가 남는다. 검사 의미를 유지한 추가 비용 개선과 실제 통합 부하/마감 검증은 후속이다. 이번 행렬을 넘어 원인을 추정해 native 확장이나 반복 시험을 추가하지 않았다.

## 출력·메모리·반례 검증

[국소 검증](optimization_v1/local_verification.json)은 원 helper와 후보의 전체 출력18개를 대조했다: 실제 입력4개, 기존 테스트9개, 배치 경계·서로 다른 첫 거절 순서·최대폭 동률·빈 질의5개다. 실제 입력에서 모든 bracket 반경·각도 비트와 lo/hi 선택도 exact였다. 별도 signed-zero 검사도 통과했다.

Windows에는 pytest가 없어 원 테스트 파일의 import/decorator만 AST에서 제외하고 **원 assertion body와 parameter9개를 그대로 호출**했다. pytest 실행으로 표기하지 않으며 패키지는 설치하지 않았다. 원 테스트 SHA는 `e742acd67a6f8f4ff5b4b174d30bb824d6ef2e9501a55c96908d7a2174f48172`다. Windows NumPy의 실제 기하 query 수는 ARM native와 일부 다르므로 국소 비교는 같은 배열끼리만 했고, 별도로 후속 ARM 전체 Evaluator의 exact를 확인했다.

큰 배열은 bracket의 최대32×광선수, 기존 점유 검사의 최대64×반환수×2 범위다. 대표504/512광선에서 하나의 bracket float 행렬은 최대128 KiB, 대표 최대482반환의 점유 delta는 약482 KiB다. 전체 질의×전체 광선을 한꺼번에 유지하는 배열은 만들지 않았다. 가장 큰 국소 사례의 helper 내 Python/NumPy traced peak는 원/후보 모두**1,503,432 bytes**였으며 시간 측정과 분리했다. ARM 전체 Python 프로세스 peak RSS는 원44,588/후보42,536 KiB였다. 이 한 번의 프로세스 표본을 helper 메모리 절감으로 확대하지 않는다.

원 x86 저장 결과와 ARM의 비교는 기존 comparator 기준 r457 세 사례가 통과했고, r452:519는 실패를 그대로 보존했다. 해당 heavy 입력의 오래된 정책 진단 차이와 region count 차이(`queries`5550→5551, `masked_left`266→267)가 섞여 있다. 같은 ARM의 원/최적화 결과는 전체 exact이며 허용오차를 넓히지 않았다. [전체 출력 차이와 통계](summary.json)에 원 차이를 남겼다.

## 불변성·정상 종료·인계

입력 SHA는 두 비교 모두 `991a034a64a4b9b00fe7f98dbde8cb0b50d99f9c07187da3c5aedf89d01ed9cb`다. 원 r463/base/stage 및 r462 입력 출처, 로컬 묶음460/461파일과 원격 묶음 모두 변경0이다. 기존 ARM native SHA는 `a2afb70525be1e86f63ae072abc4fbaeec10d65f0596e5f642f135ba6a1de394`다.

최적화 `known_structure.py`의 SHA는 **`ad62d55cdb67cf2a7fca1b0e24d8a2283bfc32de2e1cf23f38817d6b3d2d2dad`**, 대상 원본은 `e20e6a1506080530efcd9308308182e1110c225655592df5db4b785047448ecc`다. 주 담당이 이 파일/패치만 최종 검토·통합할 수 있도록 공통 소스는 수정하지 않았다.

PID61119/61125와61345/61354는 모두 exit0, 강제 종료0, 잔존 process group0이다. 사전·사후 관련 실행과 열린 센서 포트0을 확인했다. ROS·센서·ESP32·모터·서보·ARM/flash·패키지·전력 설정·WSL 변경은 없었다. [자원 반환](resource_return.json)대로 Jetson 계산 자원은 반환했다.

첫 최적화 묶음 준비에서는 Windows 기본 cp949로 UTF-8 스크립트를 읽다가 실패했다. 원 partial `optimization_v1/bundle/`과 [실패 기록](optimization_v1/prepare_v1_failure.txt)을 남기고 새 `bundle_v2/`를 만들었다. SSH/정책 계산 전 실패라 측정은 중복 실행되지 않았다.

재현·검토 진입점은 [최초 준비 manifest](source_manifest.json), [최적화 manifest](optimization_v1/source_manifest.json), [검증 스크립트](verify_optimized_local.py), [집계 스크립트](summarize.py), [검토용 패치](optimization_v1/candidate.patch)다. 원 JSON·ZIP·로그·소스 사본은 Git 제외 경로에 보존했다. 새 주행 결과나 실차 안전의 증거로 합산하지 않는다.
