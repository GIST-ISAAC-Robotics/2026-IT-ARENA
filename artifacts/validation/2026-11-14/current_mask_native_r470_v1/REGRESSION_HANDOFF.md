# r470 재사용 pytest 회귀 준비

**후속 실제 실행 완료:** 기존 ARM stage/binary로 신규 회귀10개를 한 번 실행해 **10 passed, 실패0/skip0**을 확인했다. [실행·회수 결과](pytest_run_v1/RESULT.md), [원 receipt 요약](pytest_run_v1/summary.json). 공통 통합은 하지 않았고 일시 중단 인계 상태다.

신규 파일은 `stage/tests/test_current_mask_native.py`다. 기존 `tests/test_numeric_kernels.py`의 autouse backend reset, 명시 native 선택, `pytest.importorskip`, C API buffer 거절 형식을 따른다. Python fallback/오래된 binary 거절2개는 확장이 없어도 수집·실행할 수 있도록 확장 import를 fixture 안에 뒀다. 확장이 있지만 새 entrypoint가 없으면 skip하지 않고 재빌드 오류로 실패한다.

예상 수집 수는10개다: Python fallback1, 오래된 binary 거절1, inclusive 거리 경계3, 빈 질의/반환2, raw buffer 대표3. 23개 수치 사례를 모두 중복하지 않고 연결 계약과 핵심 경계만 남겼다. 경계 검사 안에서 입력 불변도 확인한다.

**준비 시점에는 새 pytest 파일을 실행하지 않았다.** 그때는 AST/구문·예상 parameter 수·기존 source/patch SHA만 정적으로 확인했다. 실제 native kernel23건·buffer6건·helper13쌍의 선행 검증은 r470 `verify_native.py`의 ARM 실행 결과다. 이후 별도 승인으로 실행한 신규 pytest10개의 성공은 상단 후속 receipt로 구분한다. 준비 당시 `regression_receipt.json`의 `pytest_executed=false`는 원 기록으로 보존한다.

## 동결 해제 뒤 주 담당이 실행할 통합 명령

다음은 **준비한 명령이며 이번 작업에서 실행하지 않았다.** r467 최적화가 반영된 checkout을 전제로 원 r470 구현5파일과 신규 pytest1파일을 적용한다. 변경 파일/해시는 `regression_receipt.json`과 원 `changes.json`을 따른다.

저장소 루트에서:

```powershell
git apply --check artifacts/validation/2026-11-14/current_mask_native_r470_v1/candidate.patch
git apply --check artifacts/validation/2026-11-14/current_mask_native_r470_v1/regression.patch
git apply artifacts/validation/2026-11-14/current_mask_native_r470_v1/candidate.patch
git apply artifacts/validation/2026-11-14/current_mask_native_r470_v1/regression.patch
```

실행 자원을 확보한 Linux/ARM checkout의 저장소 루트에서 native 확장을 먼저 재빌드하고 회귀를 실행한다. 설치 패키지를 새로 추가하는 명령은 포함하지 않는다.

```bash
cd src/arena_autonomy
python3 -B setup.py build_ext --inplace
cd ../..
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface" \
  python3 -B -m pytest -q -p no:cacheprovider \
  tests/test_current_mask_native.py tests/test_current_structure_shadow.py tests/test_numeric_kernels.py
```

실제 공통 검증·배포·주행 재개는 주 담당이 동결/자원 상태를 확인한 뒤 수행한다. 이 파일은 추가 실행 승인이나 성공 결과를 대신하지 않는다.
