# 인계 — M3 후방 가능 영역 첫 묶음 (10/26, Claude → GPT)

사용량 경고로 중단했다. 실행 중인 프로세스는 없다(WSL `compare_release|counterexamples|decompose|measure_tail` 0개 확인). 운영 소스는 바꾸지 않았다(`m3_execution_trial.py` sha256 `1851e27f…` 작업 전후 동일). 커밋·푸시·외부 게시·Gazebo/ROS 주행·Jetson/전원 변경은 없다.

## 결론(근거는 E3 §8.27)

- 해제 차단은 L1 정보 한계와 L2·L3 장부 보수성으로 갈린다. L1은 후면 평면 뒤 60° 마스크와 뒤 모서리 꼬리 띠(0–17.5 mm)다. L2는 기록별 1 s 확인 만료로 모든 입력이 탐지+1.1 s에 래치되는 문제다. L3는 16기록 상한이다.
- 구별불가 장면: 정상 통과 S1과, 숨은 14×6 mm 물체(S2a)·뒤 0.45 m 숨은 차(S2b)를 더한 장면의 81스캔이 모두 같았고 판단도 같았다. 현 관측 계약에서는 관측만으로 복귀할 수 없다.
- 미연결 합집합 후보: 엄격 판정에서 해제 0·명령 차이 0이다. P-follow+P-size 2 cm에서만 해제했다. 전제는 미채택이다.

## ⚠ 마지막에 발견한 결함 — 전제 판본 결과는 재실행 필요

`release_premises.corner_zone_admits`의 캐시가 `id(scene)`만 키로 썼다. 해제된 장면의 id가 다음 장면에 재사용되면 이전 장면의 모서리 판정을 돌려줄 수 있다. 장면 객체를 함께 보관하고 동일성까지 확인하도록 **수정만 했고 재실행은 하지 않았다**(수정 전 sha256 `5d529996…`, 수정 후 `5fc9559f…`).

- **영향 없음:** run2/run4의 `plain`·`strict`·`diag_no_expiry`·`union_strict`는 모서리 판정을 쓰지 않는다. 따라서 엄격=현행 출력 동일, L2/L3 래치 시각, 합집합 엄격의 해제 0은 유효하다. run1(분해)·run6(꼬리 띠)도 영향이 없다.
- **재확인 필요:** `follow_tail20(_noexp)`, `union_follow*`의 해제 시각·명령 수·비용(run2·run4), run3 반례 결과(구별불가 스캔 동일성 검사는 판단기와 무관하게 유효)가 대상이다. E3 §8.27·현황의 해당 수치는 결함판 결과로 표시했다.

## 다음 정확한 작업

1. 새 번호로 재실행한다. 기존 디렉터리는 보존하고 덮어쓰지 않는다.
   - `run7`: `compare_release_premises.py --plan run2_plan.json --workers 8 --output run7_premises_cachefix`
   - `run8`: `counterexamples.py --workers 12 --output run8_counterexamples_cachefix`
   - `run9`: `run4_plan.json`, `--workers 1`
   - WSL `python3 -I -B`, 작업 디렉터리 저장소 루트. 실행 스크립트 형식은 `run2_premises.sh`를 따른다.
   - 이어서 `analyze_run2.py`의 `run = HERE/'run2_premises'`를 새 경로로 바꾼 사본을 실행한다.
2. run2/3/4와 run7/8/9를 비교한다. 같으면 §8.27에 "캐시 수정 뒤 동일"을 적고, 다르면 수치를 교체하되 결함판 결과는 보존한다.
3. §8.27.6의 GPT 결정 ①–④를 내린다. 승인되면 `m3_execution_trial.py`로 기본 false 이관한다. 이때 후속 출처 계약과 4개 시험 사슬 목록을 갱신하고, 반례 17개 회귀화·저장 5입력 OFF 동일성을 확인한다. 그 뒤 run17/21/35형 제한 폐루프를 각 1회 사전 등록한다.

## 변경 파일

- 신규: 이 폴더 전체(스크립트·실행 산출물·README·HANDOFF). `run5_syntax_error/`는 실패 보존이다.
- 문서: `docs/autonomy/MULTI_ACTOR_E3_2026_10_24.md`에 §8.27을 추가했다(기존 CRLF 유지). `docs/PROJECT_CONTEXT.md`는 '현재 구현·한계'를 교체하고 재개 지점 문장을 바꿨다. `docs/reports/DEVELOPMENT_PLAN_2026_10_23.md` §9.6에 10/26 단락을 추가했다. 이 편집으로 그 파일의 기존 CRLF 2줄이 LF가 됐다(`eol=lf` 정책과 같은 방향). `docs/HARDWARE_AND_RULE_QUESTIONS.md`에는 10/26 수치 보충을 넣었다.
- 활동 기록(`docs/activity/2026-10-26.md`)은 최종 마감용이라 쓰지 않았다.
