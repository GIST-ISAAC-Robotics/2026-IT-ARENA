# r507 최신 소스 후보 이관 묶음 — 설치 대기

2026-11-14. **Source candidate, 설치/채택 미승인(`install_pending=true`, `adoption_approved=false`)**이다. 생성 당시 r502 `moving_pass.py` 후보는 공통 소스에 있으나 r503 실제 시뮬레이션 결과의 주 담당 검토를 기다리고 있다. 해당 검토 뒤에만 설치 여부를 판단한다. 이번 작업은 Windows 파일 준비/검증만 했고 Jetson·장치·ROS·WSL·flash·구동에 접근하지 않았다.

## 묶음과 검증

- [후보 ZIP](candidate_v1/runtime_handover_sources.zip): **722개 소스＋embedded manifest1개**, 2개 패키지와 scripts의 기존 재귀 수집 및 명시 공개 리소스6개. 필수 누락0, CRC 오류0, ZIP 전체 파일 SHA 일치, embedded manifest 일치, 수집 전후 공통 소스 변경0. ZIP은 로컬 보존하며 이 폴더의 `.gitignore`로 제외한다.
- ZIP SHA-256: `a6e29226f44d89efe8e7e49230f464ce5b503f7623e510702acec04a0504db77`.
- [manifest](candidate_v1/manifest.json) SHA-256: `75bd201e707899ce24d2445ee7e071e1cf169cde4fcfbce9243fefe6a52ce9a1`.
- 실제 assembly ID·영점·board/installed profile·선택 YAML·clock policy·motor-feedback·runtime port는 **모두 null**. 지상/공중 수치는 기존 지시의 메타데이터이며 실장값 채택이나 검증 결과가 아니다.
- private/빌드/설치/캐시·인증파일·원시 sensor 기록 경로, 실행 바이너리 확장자/ELF·PE·Mach-O 헤더는0개다. 알려진 private key/일부 provider key 서명도0개다. 이 제한된 서명 검사를 모든 비밀의 부재 증명으로 확대하지 않는다.
- 공개 리소스6개와 참고용 r401 템플릿은 원본과 같다. 템플릿은 활성 운용 설정으로 선택하지 않는다. 최종 실측·보드 값·포트·시각 정책을 임의로 넣지 않았다.

[전체 검증 영수증](verification_receipt.json) · [생성 전 소스 목록](sources_before.json) · [도구/기준 계보](provenance.json).

## r500 설치 기준과 다른 원본

기준은 r500이 설치 검증한 r491 `final_v1`의721파일이다. **추가1·변경6·삭제0**이며 각 파일의 전체 이전/이후 SHA-256은 [delta_from_r500.json](delta_from_r500.json)에 보존했다.

| 상태 | 원본 경로 | 변경의 출처/경계 |
|---|---|---|
| 추가 | `scripts/mounted_servo_setup.py` | 별도 세션의 기존 현 파일을 그대로 포함. 이번 작업에서 수정/실행하지 않음 |
| 변경 | `scripts/steering_zero_profile.py` | r504 오프라인 목표 계산의 선택 조향 범위 입력 |
| 변경 | `src/arena_autonomy/arena_autonomy/local_follow.py` | r498 계산 중복 감소 |
| 변경 | `src/arena_autonomy/arena_autonomy/pass_release_trial.py` | r498 계산 중복 감소 |
| 변경 | `src/arena_autonomy/arena_autonomy/traffic_solo_gap.py` | r498 계산 중복 감소 |
| 변경 | `src/arena_autonomy/arena_autonomy/race_ops_page.py` | r505 운영 패널 수정 |
| 변경 | `src/arena_autonomy/arena_autonomy/moving_pass.py` | r502 후보, 생성 당시 r503 주행 결과 검토 대기 |

생산 소스·기존 r491 도구·원본 ZIP/manifest는 수정하지 않았다. r491 ZIP SHA `73bebdcdc176eef04b641dc78c986b76ec32431b194c787cba0f351ad6f95965`, manifest SHA `ee450fef0d971d2b94e75544420de5e13d6e9e694b4d2a12bdf6f848581098a5`가 생성 전후 r500 영수증과 일치한다. 새 묶음을 r500 설치 완료 증거로 소급하지 않는다.

## 사용 도구와 재개 지점

기존 r491 도구는 출력 폴더를 도구가 있는 artifact 하위로 제한한다. 따라서 [동일 도구 사본](stage/scripts/prepare_runtime_handover.py)을 r507에 **byte-identical** 복사하여 재사용했다. 양쪽 도구 SHA는 `1d44f8f1cffc218538347406805451599731ec83514ca039abbb1ce5b45fa226`이다. 내부 schema 이름은 기존 `runtime_handover_source_candidate_r491_v1`을 유지했고 임의 생산 도구 변경은 없다.

완료한 생성 명령은 아래와 같다. 출력은 이미 존재하므로 재실행하지 않는다.

```powershell
python -B artifacts/validation/2026-11-14/runtime_handover_r507_v1/stage/scripts/prepare_runtime_handover.py --repo . --output artifacts/validation/2026-11-14/runtime_handover_r507_v1/candidate_v1 --archive
python -B artifacts/validation/2026-11-14/runtime_handover_r507_v1/verify_candidate.py
```

[검증 스크립트](verify_candidate.py)는 CRC/내용 SHA·전후 원본·기준 SHA·제외 경로·delta를 직접 대조하며 새 검증 결과의 덮어쓰기를 거절한다. 이후 공통 소스가 바뀌면 이 후보를 최신이라고 재명명하지 말고, 주 담당이 변경과 r503 결과를 검토해 새 버전 생성 또는 현 후보 사용을 판단한다. 원격 전송·overlay 빌드·ARM binary 설치·실장값 선택·firmware build/flash·실물 검증은 이번에 수행하지 않았다.
