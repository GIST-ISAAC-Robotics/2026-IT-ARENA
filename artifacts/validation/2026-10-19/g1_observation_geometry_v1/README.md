# G1 고정 실패 입력의 기하 분해 — 2026-10-19

이 묶음은 **저장 입력의 원인 분해**다. 새 공식 주행, 복구 구동, 물성/센서 변경은 없다. 원래 실패를 성공으로 바꾸지 않는다. 상세 해석은 [기술 보고서 §12](../../../../docs/autonomy/G1_OBSERVATION_ENVELOPE_2026_10_16.md#12-1019-고정-실패-입력의-기하-분해)를 따른다.

## 보존 자료

- `run/report.json`, `run/witnesses.json`, `run/source_snapshot.zip`: Claude가 만든 최초 분해. seq 3151/3156/3181/3501까지 2,960판단 재생, 불일치 0. `source_unchanged`는 도구 끝부분에서 수집한 소스 해시의 최종 일치 검사이며, 실행 시작부터의 전체 변경 감시와 같지는 않다.
- `world_eval/`: 기록 자세·벽 충돌체·공식 중심선을 이용한 **평가 전용** 거리 대조 코드와 로그. 대회 제어기에 지도/정답 자세를 전달하지 않았다. `worldray.log`의 거리는 현재 자세 근사이며 실제 각 광선 취득 자세에서의 정밀 렌더러 재현이 아니다. 재생 중간 캐시 pickle은 로컬 탐색 임시물로, 원본 입력에서 `extract.py`로 재생성할 수 있다.
- `handoff_witness_pytest.log`: 인계된 새 검사 9개 중 2개 실패. 좌우 동시 취득과 단일 실패 원인을 잘못 가정했다.
- `witness_pytest_v2.log`: 첫 정정 뒤 16개 중 2개 실패. fixture의 후방 마스크를 측방 실패와 혼합한 기대값이 남아 있었다. 두 실패 기록은 삭제하지 않았다.
- `witness_related_v3.log`: 실패 광선의 실제 상태·취득 시각과 후방 swing-out을 구분한 뒤 관련 **40회귀 통과**, `PYTEST_EXIT_CODE=0`. 운영 판단 코드는 변경하지 않았다.
- [후속 감사](../g1_observation_geometry_v2_retry/run/report.json): 동일 2,960판단/불일치 0, 기존 지지 개수와 원인 분해 일치. `unknown_bracket`/`masked_bracket`을 추가해 쐐기 내부와 인접 깊이 결측을 구분한다. 원래 입력 hash 유지, 운영 함수 변경 없음.
- [후속 그림](../g1_observation_geometry_v2_retry/run/seq3156_recorded.png): 한글 글꼴 누락을 고쳐 PNG/SVG를 함께 저장했다. 회색은 현재/예측 명목 몸체, 주황은 실제 실패한 질의 위치다. 실제 주행 사진이 아니다.

후속 감사의 첫 준비는 출력 폴더를 미리 만들어 `FileExistsError`로 끝났다([로그](../g1_observation_geometry_v2/audit.log)). 재시도는 보고서와 그림을 만들었으나 외부 실행 래퍼가 종료값 변수를 잘못 인용해 코드 1을 반환했다([로그](../g1_observation_geometry_v2_retry/audit.log)). 따라서 **보고서의 재생/해시/일관성 결과와 래퍼 정상 종료는 구분**한다. 생성 단계 오류와 셸 오류를 주행/관측 실패로 합치지 않는다.

## 재현

WSL 저장소 루트에서 `PYTHONPATH=src/arena_autonomy:src/arena_vehicle_interface:scripts`를 사용한다. 출력은 존재하지 않는 새 하위 폴더를 지정한다.

```bash
python3 scripts/audit_observation_geometry.py \
  artifacts/validation/2026-10-17/g1_recent_solo_v1/run/follow_decisions.jsonl \
  --output artifacts/validation/새_날짜/새_기하_감사/run
```

코드 위치는 `scripts/observation_witness.py`, `scripts/audit_observation_geometry.py`, `tests/test_observation_witness.py`다. 현재 운영 코드와 저장 입력 hash를 별도로 확인해야 한다. 원시 입력은 기존 로컬 보존/Git 제외 정책을 따른다.
