# A2·P1 오프라인 판별 v1

[E3 §8.3](../../../../docs/autonomy/MULTI_ACTOR_E3_2026_10_24.md#83-a2p1-오프라인-판별-v1)의 근거 산출물이다. 순수 분석 시제품이며 운영 src/launch 연결·Gazebo·ROS 실행·판정선/물성/기본값 변경은 없다.

## 구성

- `candidate.py`: 명시 인자만 받는 후보 함수. `side_state`(측후방 존재·차체 바깥 판정·unknown·창 관측 가능성), `p1_path`(곡선별 법선 오프셋과 역전·곡률 접힘·자기교차·정적 도로 끝 선별), `existing_checks`(src의 `pursue_path`·`swept_limit`·`braking_speed`를 G1과 같은 순서로 호출), `A2Machine`(P0/P1·해제·무효 전이 기록).
- `test_candidate.py`: 저장 기록·정답을 쓰지 않는 합성 단위 회귀 10개.
- `screen.py`: 고정 판 1회 대조. 후보 열은 정답 파일 열기를 막은 상태에서 계산하고(`TruthBlocked`), 정답 자세는 `eval_*` 열에만 붙인다. G1 창(3.8–7.0 s)에서는 정적 반사실·누락 주입·거울 두 종류도 계산한다.
- `run_fixed.sh <run>`: 새 폴더에 스크립트 사본과 src·입력 해시를 먼저 남긴 뒤 단위 회귀와 대조를 실행하고, 끝난 뒤 해시를 다시 대조한다. 기존 폴더는 덮어쓰지 않는다.

## 개발 중 결함과 보존

- 최초 스모크(G1 창)에서 두 결함을 찾았다. (i) 반사실 결과의 상태 표기가 기존 검사 결과로 덮어써져 요약에서 0건으로 집계됐다. (ii) P1의 목표 조향은 약 0인데, 기록된 G1의 직전 조향에서 3 rad/s 변화율로 깎여 0.06–0.2 rad로 보였다. [디버그 스크립트](dev/debug_p1_steering.py)로 원인을 확인했고, P1 활성 중에는 후보 자신의 직전 조향을 잇도록 고쳤다. 최초 스모크 출력은 보존하지 않았고 수정 뒤 스모크만 [`dev/`](dev/)에 있다.
- 고정 판 실행은 아래와 같다(해석은 보고서에만 둔다). 각 폴더에 그때의 스크립트 사본, `hashes_before.txt`, 실행 뒤 `hashes_after_check.txt`를 둔다.

| 폴더 | 명령 | 결과 |
|---|---|---|
| `run1/` | `run_fixed.sh run1` | pytest 10/10, screen exit 0, 해시 재대조 0. 저장 3기록·정적 반사실·거울 두 종류 유효. **누락 주입 두 결과는 무효**: `'<U6'` 상태 배열에 `'unknown'`을 넣어 `'unknow'`로 잘렸고, unknown 수가 0으로 세어졌다 |
| `run2/` | `run_fixed.sh run2 --missing-only` | **실패**: 패치 문자열의 줄바꿈 오류로 SyntaxError, screen exit 1(pytest 10/10, 해시 0). `FAILED_NOTE.txt` |
| `run3/` | `run_fixed.sh run3 --missing-only` | pytest 10/10, screen exit 0, 해시 재대조 0. 누락 주입 두 변형만 다시 계산(`report_missing.json`) |
| `run4/` | `run_fixed.sh run4 --g1-recheck` | pytest 12/12, screen exit 0, 해시 재대조 0. GPT 최종 검토의 결함 2건(거울 직전 조향 미반전, 재촉발 시 이전 후보 조향 재사용)을 고친 뒤 G1 창만 다시 계산(`report_recheck.json`). candidate `5ba9a19d…`, screen `143ff503…`, test `77c14869…` |

재현: 저장소 루트에서 `MSYS_NO_PATHCONV=1 wsl -d Ubuntu-24.04 -u jinhyeong bash -lc 'bash "/mnt/c/.../m3_p1_offline_v1/run_fixed.sh" <새 폴더> [--missing-only]'`. 기존 폴더 이름을 주면 덮어쓰지 않고 종료한다. 실행·회수는 Claude Haiku 5.5 보조(자체 보고 `claude-haiku-5-5`)가 맡았다. `__pycache__/`는 pytest가 만든 캐시이며 Git 제외 대상이다.

`candidate.py`는 run1~3이 같고(`752e17dd…`), run4에서 `SteeringChain`을 더해 `5ba9a19d…`(현재본)가 됐다. `screen.py`는 run1 `1bc7dc90…` → run3 `72faf20c…`(누락 주입 dtype 수정, `--missing-only`) → run4 `143ff503…`(현재본: 체인 리셋, 거울 직전 조향 반전, `--g1-recheck`)으로 바뀌었다. 재현 선택 인자는 없음/`--missing-only`/`--g1-recheck`다. `run_fixed.sh`는 run1 뒤에 선택 인자 전달을 추가했다. run1 당시의 `run_fixed.sh` 사본은 따로 남기지 않았다.

## 한계

- 저장 궤적은 원래 G1/R0+ 정책의 것이다. P1 검사는 원래 자세에서 계산한 개루프 판정이다. 정적 반사실은 실제 점을 P1 유지 자세로 옮긴 것으로, 그 자세에서 새로 보였을 면과 다른 종방향 진행은 반영하지 않는다.
- 거울 두 종류와 누락 주입은 합성 입력이다. 실제 오른쪽 주행이나 실물 무응답의 검증이 아니다.
- P1 `valid`는 기하 무결성과 정적 도로 끝 선별만 뜻한다. 차체 swept footprint의 도로 포함은 계산하지 않았다(`unverified`).
