# r386 WSL 실행 인계 — 준비만 완료

**실행 보류 중이다.** 현재 검사 윤거0.135m·바퀴폭0.012m와 최종 CAD0.1239m·0.0261m의 차이가 r377정체에 미친 영향을 주 에이전트가 먼저 감사한다. 아래 명령은 준비 기록이며 지금 실행하지 않는다. 후보·pytest·native·WSL 실행은 모두0회다. [README의 필수 후속](README.md)을 확인한다.

주 에이전트가 배정한 단일 WSL 담당자에게만 인계한다. 원 제어 소스/후보/시험 결과를 덮어쓰지 않으며 새 ROS/Gazebo/Jetson 작업을 시작하지 않는다.

| 항목 | 고정 값 |
|---|---|
| archive | `bundle_sources.zip` |
| bytes | 115,923,593 |
| archive SHA256 | `4a116fd85b60a9ece7bb9dbdeb197aa872314f595b7219ed9b0b3dafe025ea04` |
| manifest SHA256 | `845f9a1d4e47d17c95af08efd987162123d6a3e8c7c1defa398dbaa6035856da` |
| execute_isolated.py SHA256 | `40c0d16c419ca4dd18ae28ac03412a32020f0efb2dce21a4059113188136f7a9` |
| 원 r377 sources.zip SHA256 | `91cda3f3f394a89107ec84db53963f4c245cf39364acd59b6be241037663a54c` |
| 전체 파일 / 원 r377 소스 | 268 / 251개 바이트 동일 |

단일 실행 명령은 다음과 같다. 새 사용자 홈 루트가 이미 있으면 중단한다. 다른 실행을 임의로 종료하거나 루트를 지우지 않는다.

```bash
python3 "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/current_corner_budget_r386_v1/execute_isolated.py" --archive "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/current_corner_budget_r386_v1/bundle_sources.zip" --sha256 4a116fd85b60a9ece7bb9dbdeb197aa872314f595b7219ed9b0b3dafe025ea04 --root "$HOME/it-arena-r386-20261113"
```

실행 순서는 새 격리 루트 native 빌드120초 → 준비된26개 회귀120초 → native 저장 입력795개와15반례 비교900초다. 각 단계를1회 수행하며 실패하면 후속 단계로 넘어가지 않는다. `execute_isolated.py`는 자기 자식 PID/process group만 관리한다. root 설치/빌드/기존 WSL 작업 디렉터리나 장치 포트를 변경하지 않는다.

결과는 새 루트의 `artifacts/validation/2026-11-13/current_corner_budget_r386_v1/` 아래 `execution.json`, 단계 로그, `results_native_v1/report.json`, `rows.jsonl`이다. 자동 회수 archive는 입력 archive 옆 `bundle_sources-results.zip`으로 생성되며 기존 파일이 있으면 덮지 않는다. 원 raw행의 Git 제외 정책을 유지한다.

확인할 항목:

- 원 r377223개 재현·원10.01m 주행 실패 보존, 새 저장 후보 적용 수/마지막 속도/거부 이유.
- r371217+r364249+과거106 보존,15반례 정지. 변화가 있으면 비교 실패로 기록하며 조건을 바꾸지 않는다.
- 적용 시 측정속도·현재/직전 조향·원 경로/시각 불변, `D(measured)`와 최소질의10mm, 한 개 목표와 최종 검사1회, unknown 수/공백 위험 보존.
- baseline/제안기하/제안분류/최종core·region·gap/추가합계 비용. 비교용 baseline+추가를 향후 통합 최적화 비용으로 오인하지 않는다.
- 소스 변경0·자식 종료·강제회수 유무. 계산 실패/반례 실패/비용 한계는 원 결과대로 남긴다.

Windows 준비 검증은 AST 문법/문서 링크와 압축 내부268개 해시·원251개 source byte 보존까지다. 후보 호출0·pytest실행0·native빌드0이다. 전체랩·실주행 정체 해소·실차 허가는 이번 시험 대상이 아니다. 이미 관측된 바깥 공간에 새10mm 제한을 걸지 않았고, 제안격자의 보수성이 커서 효과가 없거나 추가비용만 커질 수 있음을 [README](README.md)에 명시했다.

archive 생성 후 실행 보류와 기하 필수감사 설명을 이 문서 및 외부 README에 추가했다. archive/manifest/코드는 변경하지 않았다. archive의 README는 생성 당시 문서이며 현재 README의 해시는 원 manifest와 다르다. 주 에이전트가 재개를 배정하기 전 이 준비 명령을 실행하지 않는다.
