# r414 fixture revision2

후보 코드는 변경하지 않았습니다. 첫 실행은24개 중20통과/4실패로 중단됐고 저장254대조는 시작하지 않았습니다. 원 시험·manifest·실패 로그는 원 위치와 `reference_v1/`에 바이트 그대로 보존했습니다.

F1의4실패는 `anchor_probe.anchor`를 최초 capture 시각으로 기대한 시험 오류입니다. 기존 Recovery는 옛 위협의 전체 점/시각을 capture한 뒤 같은 evaluate에서 `anchor_probe.observe`를 실행합니다. rolling anchor는 현재 reference/source로 이동할 수 있고, 최초 위협 근거는 `anchor_threat_summary`와 `anchor_probe.created`에 남습니다.

revision2는 F1에서 다음을 구분해 확인합니다. 입력 위치·거리·폭·속도·시각·여유·래치 기대와 capture3점 조건은 바꾸지 않았습니다.

- `anchor_threat_summary.reference_s=.1`, `source_first_s=0`, `anchor_probe.created=.1`: 최초 근거 보존.
- `anchor_probe.anchor[1:]=(.2,.1)`, retained3점: 기존 재관측에 따른 rolling anchor.
- 기존 `previous_point_in_missing_sweep` latch/불허·전체3점 capture·현재점 기억 검사는 그대로.

시험 파일/runner가 하위 revision 경로에서도 같은 동결 staged/header/input을 읽도록 경로만 조정했습니다. 원 비교·판정·실행 순서는 변경하지 않았습니다. Windows AST·SHA 확인만 수행했고 revision2 검사는 아직 실행하지 않았습니다.

```bash
python -X utf8 -B artifacts/validation/2026-11-13/crop_edge_component_r414_v1/revision_fixture_v2/compare_saved.py \
  --output artifacts/validation/2026-11-13/crop_edge_component_r414_v1/verify_native_v2
```

새 manifest의 입력 불변 확인 → tests24 → baseline254 → OFF254 exact → ON254 → 제한 저장 비교입니다. 실패하면 중단하고 원 로그를 보존합니다. 첫37 oracle/이후217 open-loop 한계와 새 허가 위험은 상위 README와 같습니다.

staged partial SHA는 계속 `576966260591adf169c426f33c921df0586ca75f31e9e973c69be94b2c268425`입니다. 원 manifest SHA도 `1aa8c8cf9408689fcb355e33adac3e43b43dbc4198cd23bf77a2343c09a9260a` 그대로입니다.
