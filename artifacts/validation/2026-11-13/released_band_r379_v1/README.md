# RELEASED 측방 띠의 기존 광선 분류 공유 r379

2026-11-13. **399건의 판단·선택기 상태·띠 판정이 일치했고, RELEASED 343건의 합계 계산 시간이 4.07% 줄어 공통 `band_check`에 이관했다.** 새 수치 커널·캐시·관측/보호 기준은 추가하지 않았다. Jetson 전체 비용이나 10Hz 달성을 확인한 결과는 아니다.

수정 후 [`verify_native_v2/report.json`](verify_native_v2/report.json)은 1,380개 띠 호출·325,567개 표본의 배열까지 대조했고, [`cost_native/report.json`](cost_native/report.json)은 배열 재계산 없이 별도로 시간을 측정했다. RELEASED 합계13.146→12.611초, 중앙40.772→39.392ms, p95 46.331→44.354ms다. 양쪽 순서를 교대하는 단일 직렬 저장 재생이며 실제 동시 부하의 개선률로 확대하지 않는다. 원 기록과의 기존 전체 출력 차이30건도 두 비교에 그대로 남겼다. 수정 전후 두 제어기 사이에는 차이가 없었다. 공통 이관 뒤 실제 호출 경로를 포함한 관련124회귀도 통과했다([이관 영수증](../receipt_r379_integration_v1/)).

후속 실행에서 관련 122개 테스트는 통과했으나, `verify_native`의 첫 선택기 상태 직렬화가 `set` 미지원으로 실패했다(`calls=0`). 따라서 전체 판단·상태 정확성은 아직 미검증이며 비용은 실행하지 않았다. 원 실패와 `../receipt_r379_v1/verify.log`, 당시 준비 manifest는 유지한다. 실패 당시 스크립트는 [`compare_failed_v1.py`](compare_failed_v1.py)에 보존했고, 현재 `compare.py`는 `set/frozenset`의 타입과 모든 원소를 정렬하여 보존하도록만 수정했다. 상태 비교 범위는 동일하다. 별도 [`compare_v2_manifest.json`](compare_v2_manifest.json)에 수정 전후 해시를 기록했다. 재실행은 기존 결과를 덮어쓰지 않는 아래 한 명령으로 수행한다.

```sh
python3 artifacts/validation/2026-11-13/released_band_r379_v1/compare.py --backend native_v1 --verify-arrays --out artifacts/validation/2026-11-13/released_band_r379_v1/verify_native_v2
```

## 원인과 범위

`pass_release_trial.band_check`는 현재 띠/측방 창의 `depth_support(q)` 뒤 미지지 질의에 `masked_only(q[~free])`를 호출한다. 같은 질의의 양옆 광선 탐색을 중복한다. 반면 같은 파일의 `motion_region`은 이미 `depth_and_masked_support`를 사용한다. r359 제한 프로파일 index60/120에서 `band_check`4회 비용은26.44/25.74ms였다. 계측 오버헤드 포함의 소수 표본이며 이 시간 전부를 절감할 수 있다는 뜻은 아니다.

[`candidate.py`](candidate.py)는 `band_check`에만 기존 공유 분류를 적용한다. 새 캐시·수치 커널·주기/TTL 변경은 없다. Python에서는 기존 함수가 `None`을 반환하므로 원 분리 계산을 유지한다. native에서는 공유 분류의 `free`를 쓰고 `masked`는 기존처럼 `~free`인 질의에만 반영한다. 짧은 띠와 전체 측방 창, 질의 순서·간격·원점·패딩·미관측 개수·clear/passed 상태 전이는 유지한다. 가림을 자유로 세지 않는다.

이 변경은 계획의 fresh 측방 판정에 적용되며 현재 검사63–67ms를 통째로 없애는 변경이 아니다. 계획과 현재 검사는 기준 시각·운동 이력·반환점·조향이 다르므로 계획의 관측 허가를 현재 검사로 넘기지 않는다.

## 준비한 검증

- [`test_candidate.py`](test_candidate.py): Python/native,500점CCW/508점CW, 유효 반환/무반환/근접 반환·NaN·장착 마스크 혼합에서 좌우 띠와 전체 창을 대조한다. 각 `free/masked` 배열 dtype·shape·바이트와 최종 band 사전을 검사한다. 빈 띠와 예외 후 함수 복구도 검사한다. native 모듈 부재는 skip 통과로 처리하지 않는다.
- [`compare.py`](compare.py): r353399입력·reset을 두 독립 현재 제어기에 같은 순서로 적용한다. 원 r278 정책도 양쪽에 동일 설치한다. 전체 출력과 **선택기의 전체 내부 상태**(sides/clear/passed/vanish/last_present, epoch/시각, guard_anchor/last_choice, 정적 측방 tracker 포함), 모든 띠/창 호출 사전을 정확 비교한다. 표면 정합 `processing_ms`의 기존 정규화 이외의 진단은 제거하지 않는다.
- `--verify-arrays`는 후보의 매 비어 있지 않은 띠/창에서 원 분리 계산을 추가 수행하여 분류 배열도 정확 대조한다. 이 실행의 경과시간은 성능값이 아니다. 별도 비용 실행은 baseline/candidate 선행 순서를 교대한다.
- 모든 비교는 실행 전후 Python 소스와 입력·후보 해시를 검사한다. 원 r353 출력과의 기존 차이는 별도로 보존한다. 비동기 원 실행의 상태 순서를 재현했다는 주장이 아니다.

## 실행 명령과 중단 조건

아래는 당시 준비한 명령이다. 첫 `verify_native`는 직렬화 오류로 실패했고, 수정된 `verify_native_v2`와 별도 `cost_native`를 정상 종료했다. 기존 결과 경로에 재실행하지 않는다. 공통 소스 이관 뒤에는 양쪽이 수정본을 가리키므로 원 비교를 반복할 때 반드시 당시 동결 소스를 사용한다.

```sh
python3 -m pytest -q artifacts/validation/2026-11-13/released_band_r379_v1/test_candidate.py tests/test_shared_ray_classification.py tests/test_pass_release_trial.py
python3 artifacts/validation/2026-11-13/released_band_r379_v1/compare.py --backend native_v1 --verify-arrays --out artifacts/validation/2026-11-13/released_band_r379_v1/verify_native
python3 artifacts/validation/2026-11-13/released_band_r379_v1/compare.py --backend native_v1 --out artifacts/validation/2026-11-13/released_band_r379_v1/cost_native
```

Python fallback은 첫 회귀의 실제 분류 대조로 확인한다. 원리상 같은 분리 계산을 다시 하는 Python 전체399호출 비용 비교는 우선 실행하지 않는다.

배열·띠 사전·전체 판단·상태 중 차이1건, 입력/소스 변경, 미완료면 즉시 실패로 보존하고 비용 채택을 중단한다. 정확일치 뒤 비용 결과로 주 작업 담당자가 Jetson 대표 비교 여부를 결정한다. 효과가 미미하면 종료하며, 더 큰 캐시/커널 탐색을 이어 붙이지 않는다. 전체10Hz·명령 만료·ROS/실차 성립은 별도 검증이다.

관련 근거: [r359 제한 프로파일](../latest_plan_profile_r359_v1/README.md), [기존 공유 분류 r357](../shared_ray_classification_r357_v1/README.md), [r369 상태별 비용](../retained_schedule_r369_v1/README.md), [희소 캐시 r374 미채택](../released_cost_r374_v1/README.md).
